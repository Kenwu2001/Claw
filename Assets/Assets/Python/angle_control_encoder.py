# -*- coding: utf-8 -*-
"""
整合版控制：
  指令：
    - cal abd / cal add            # 校正：關扭矩→手動到基準→存檔→再開扭矩
    - t abd / t add                # 只收線到位後停止（用張力/電流邏輯）
    - 整數(±度)                    # 先收線（跑幾何補償角），再用 encoder-PID 對到「起始+輸入」的目標

  整數角度命令流程（重點）：
    1) 讀起始 encoder，設定 target = 起始 + 輸入
    2) 依輸入方向選擇要收緊的側（delta<0→ABD；delta>0→ADD）
    3) 跑幾何補償角（預設「走滿」；可用 LEAVE_COMP_DEG 留一段不走）
    4) 用 encoder-PID 把 encoder 對到 target（因此收線造成的位移自然計入）

  備註：
    - t abd/t add 仍使用「張力PID + 即時跳增剎停」的邏輯，單純把線拉緊就停。
    - 整數角度命令的「收線」改為只做幾何補償（預設走滿），接著直接進 encoder-PID。
"""

import os, sys, json, time, math, re, serial
from typing import Optional, Tuple
from dynamixel_sdk import *  # PortHandler, PacketHandler
import utils as ut
import argparse

# ================== 連線 / 控制表 ==================
PORT_NAME = "COM4"
BAUDRATE  = 57600
DXL_ID    = 7

ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_PROFILE_ACCEL    = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_CURRENT  = 126

# ================== Encoder 串流 ==================
ENCODER_PORT = "COM5"
ENCODER_BAUD = 115200

# ================== 檔案 ==================
CAL_FILE = "calibration.json"

# ================== 幾何比例 ==================
# motor +120° ≈ encoder -30° → 比例 -4（motor+ 會讓 encoder 減小）
K_MOTOR_PER_ENC = -4.0

# ================== 速度 / 超時（角度 PID 用） ==================
DT = 0.01                 # 100 Hz
PID_TIMEOUT_S = 5.0
PROFILE_VEL   = 800       # 可用 'speed <v>' 調
PROFILE_ACC   = 100       # 可用 'acc <a>' 調

# ================== 速度 / 安全（收線補償用） ==================
VEL_TIGHT   = 500         # 幾何補償角的 profile
ACC_TIGHT   = 180
I_STOP_HARD_mA = 300.0    # 硬保護（電流過大立刻停）
POS_EPS_TICK   = 20       # 判定到位的 tick 容差
LEAVE_COMP_DEG = 0.0      # ★ 預設「走滿」（=0），你可改成 >0 讓補償角留一段不走

# ================== 張力-PID（僅 t abd/t add 用） ==================
# 目標張力（N）→ 目標電流（mA）≈ (F * r / Kt) * 1000 * 效率
FORCE_TARGET_N      = 0.8
SPOOL_RADIUS_MM     = 2.5
KT_NM_PER_A         = 0.347
TENSION_EFF_GAIN    = 1.5
SPIKE_MARGIN_mA     = 3.0
OVERSHOOT_STOP_mA   = 4.0
T_KP, T_KI, T_KD    = 0.10, 0.015, 0.02
T_INTEG_CLAMP       = 300.0
T_MICRO_STEP_DEG    = 4.0
T_PID_TIMEOUT_S     = 4.0
T_SMOOTH_BETA       = 0.40
EMA_TAU_S           = 0.25
EMA_HOLD_COUNT      = 3

def forceN_to_mA(FN: float) -> float:
    return (FN * (SPOOL_RADIUS_MM/1000.0) / KT_NM_PER_A) * 1000.0 * TENSION_EFF_GAIN

I_TARGET_mA = forceN_to_mA(FORCE_TARGET_N)

# ================== 校正資料 ==================
CAL = {
    "TIGHT_ABD_L": None, "TIGHT_ABD_M": None,
    "TIGHT_ADD_L": None, "TIGHT_ADD_M": None,
    "RADIUS_L": 10.0, "RADIUS_M": 2.5,
    "TIGHT_DIR": {"abd": +1, "add": -1}
}

# ================== 小工具：Encoder 串流 & 展開 ==================
class EncoderStream:
    """常開的編碼器讀取：read_deg() 回傳最新角度（deg）"""
    def __init__(self, port: str, baud: int, warmup_s: float = 1.2, first_value_timeout_s: float = 2.0):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.last = None
        try:
            self.ser.setDTR(False); time.sleep(0.15); self.ser.setDTR(True)
        except Exception:
            pass
        time.sleep(warmup_s)
        try: self.ser.reset_input_buffer()
        except Exception: pass
        t0 = time.time()
        while time.time() - t0 < first_value_timeout_s:
            v = self._read_latest_value(max_lines=64)
            if v is not None:
                self.last = v; break
            time.sleep(0.02)
        if self.last is None:
            print("[ENCODER] 首筆資料逾時，請檢查 COM/鮑率/輸出格式")

    @staticmethod
    def _parse_float_anywhere(s: str):
        m = re.search(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)", s)
        return float(m.group(0)) if m else None

    def _read_latest_value(self, max_lines: int = 32):
        val = None
        for _ in range(max_lines):
            raw = self.ser.readline()
            if not raw: break
            try:
                line = raw.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue
            v = self._parse_float_anywhere(line)
            if v is not None:
                val = v
        return val

    def read_deg(self):
        v = self._read_latest_value()
        if v is not None:
            self.last = v
        return self.last

    def close(self):
        try: self.ser.close()
        except Exception: pass

class EncUnwrap:
    """把 0~360 的 encoder 角度展開成連續角，避免跨 360/0 時跳值。"""
    def __init__(self):
        self.inited=False; self.prev=0.0; self.acc=0.0
    def update(self, raw_deg: Optional[float]) -> float:
        if raw_deg is None:
            return self.acc if self.inited else 0.0
        if not self.inited:
            self.inited=True; self.prev=raw_deg; self.acc=raw_deg; return self.acc
        d = raw_deg - self.prev
        if d > 180.0:  d -= 360.0
        if d < -180.0: d += 360.0
        self.acc += d; self.prev = raw_deg
        return self.acc

# ================== DXL 小工具 ==================
def set_profile(ph: PortHandler, pk: PacketHandler, vel: int, acc: int) -> None:
    # 設定目標速度
    comm, err = pk.write4ByteTxRx(ph, DXL_ID, ADDR_PROFILE_VELOCITY, int(vel))
    if comm != COMM_SUCCESS or err != 0:
        print(f"[DXL] set VEL fail: comm={comm}, err={err}")
    time.sleep(0.01)

    # 設定加速度
    comm, err = pk.write4ByteTxRx(ph, DXL_ID, ADDR_PROFILE_ACCEL, int(acc))
    if comm != COMM_SUCCESS or err != 0:
        print(f"[DXL] set ACC fail: comm={comm}, err={err}")
    time.sleep(0.01)

def goal_position(ph: PortHandler, pk: PacketHandler, goal_tick: int) -> None:
    pk.write4ByteTxRx(ph, DXL_ID, ADDR_GOAL_POSITION, int(goal_tick) & 0xFFFFFFFF)

def read_present_position(ph: PortHandler, pk: PacketHandler) -> Tuple[int, float]:
    pos_tick, comm, err = pk.read4ByteTxRx(ph, DXL_ID, ADDR_PRESENT_POSITION)
    if comm != COMM_SUCCESS or err != 0:
        raise RuntimeError(f"read pos error: comm={comm}, err={err}")
    return pos_tick, ut.pos2deg(pos_tick)

def read_present_current_mA(ph: PortHandler, pk: PacketHandler) -> float:
    raw, comm, err = pk.read2ByteTxRx(ph, DXL_ID, ADDR_PRESENT_CURRENT)
    if comm != COMM_SUCCESS or err != 0:
        raise RuntimeError(f"read current error: comm={comm}, err={err}")
    if raw > 32767: raw -= 65536
    return float(raw)  # XL330：1 mA/LSB

def set_torque(ph: PortHandler, pk: PacketHandler, enable: int):
    pk.write1ByteTxRx(ph, DXL_ID, ADDR_TORQUE_ENABLE, 1 if enable else 0)
    time.sleep(0.02)

def init_dxl():
    ph = PortHandler(PORT_NAME)
    pk = PacketHandler(2.0)
    if not ph.openPort():  print("Failed to open port"); sys.exit(1)
    if not ph.setBaudRate(BAUDRATE): print("Failed to set baudrate"); sys.exit(1)
    pk.write1ByteTxRx(ph, DXL_ID, ADDR_OPERATING_MODE, 4); time.sleep(0.02)  # Extended Position
    set_torque(ph, pk, 1)
    set_profile(ph, pk, PROFILE_VEL, PROFILE_ACC)
    return ph, pk

def signed_tick_diff(a: int, b: int) -> int:
    d = (int(a) - int(b)) & 0xFFFFFFFF
    return d - 0x100000000 if d >= 0x80000000 else d

# ================== 校正 / 檔案 ==================
def load_calibration():
    if os.path.exists(CAL_FILE):
        try:
            with open(CAL_FILE, "r", encoding="utf-8") as f:
                CAL.update(json.load(f))
        except Exception as e:
            print(f"[LOAD] 讀取 {CAL_FILE} 失敗：{e}")

def save_calibration():
    try:
        with open(CAL_FILE, "w", encoding="utf-8") as f:
            json.dump(CAL, f, ensure_ascii=False, indent=2)
        print(f"[SAVE] 已儲存到 {CAL_FILE}")
    except Exception as e:
        print(f"[SAVE] 失敗：{e}")

def calibrate_once(mode: str, ph: PortHandler, pk: PacketHandler, enc: EncoderStream):
    assert mode in ("abd","add")
    # 關扭矩 → 手動 → 取值 → 存檔 → 再開扭矩
    set_torque(ph, pk, 0)
    print(f"[CAL] 請手動移到 {mode.upper()} 基準姿勢後按 Enter...")
    input()
    motor_tick, _ = read_present_position(ph, pk)
    enc_deg = enc.read_deg()
    if enc_deg is None:
        print("[CAL] 讀不到編碼器角度，校正中止。")
        set_torque(ph, pk, 1); return False
    if mode == "abd":
        CAL["TIGHT_ABD_M"] = int(motor_tick); CAL["TIGHT_ABD_L"] = float(enc_deg)
    else:
        CAL["TIGHT_ADD_M"] = int(motor_tick); CAL["TIGHT_ADD_L"] = float(enc_deg)
    print(f"[CAL] {mode.upper()} 基準：M={motor_tick} tick, L={enc_deg:.3f} deg")
    save_calibration()
    set_torque(ph, pk, 1)
    set_profile(ph, pk, PROFILE_VEL, PROFILE_ACC)
    return True

# ================== 幾何工具（補償角） ==================
def _deg_to_arc_mm(deg: float, radius_mm: float) -> float:
    return 2.0 * math.pi * radius_mm * (deg / 360.0)

def _arc_mm_to_deg(arc_mm: float, radius_mm: float) -> float:
    return (arc_mm / (2.0 * math.pi * radius_mm)) * 360.0

def _tick_delta_to_deg(curr_tick: int, base_tick: int) -> float:
    delta_u32  = (int(curr_tick) - int(base_tick)) & 0xFFFFFFFF
    delta_tick = delta_u32 - 0x100000000 if delta_u32 >= 0x80000000 else delta_u32
    return ut.pos2deg(delta_tick, extend=True)

def sgn(x: float) -> int:
    return 1 if x > 0 else (-1 if x < 0 else 0)

def compute_compensation_deg(mode: str, ph: PortHandler, pk: PacketHandler, enc: EncoderStream) -> Tuple[float, int]:
    assert mode in ("abd","add")
    base_M = CAL["TIGHT_ABD_M"] if mode=="abd" else CAL["TIGHT_ADD_M"]
    base_L = CAL["TIGHT_ABD_L"] if mode=="abd" else CAL["TIGHT_ADD_L"]
    R_L, R_M = CAL["RADIUS_L"], CAL["RADIUS_M"]
    if base_M is None or base_L is None:
        raise RuntimeError("尚未做 cal abd/add")
    motor_tick, _ = read_present_position(ph, pk)
    enc_deg = enc.read_deg()
    if enc_deg is None:
        raise RuntimeError("Encoder 沒讀到")
    delta_deg_m = _tick_delta_to_deg(motor_tick, base_M)
    delta_deg_e = enc_deg - base_L
    arc_m_mm = _deg_to_arc_mm(delta_deg_m, R_M)
    arc_l_mm = _deg_to_arc_mm(delta_deg_e, R_L)
    delta_L_mm = arc_m_mm + arc_l_mm
    motor_delta_deg = _arc_mm_to_deg(-delta_L_mm, R_M)
    return motor_delta_deg, motor_tick

# ================== A) 幾何補償：可設定是否「留一段不走」 ==================
def compensate_only(mode: str, ph: PortHandler, pk: PacketHandler, enc: EncoderStream,
                    leave_deg: float = LEAVE_COMP_DEG,
                    only_tighten: bool = False,
                    force_dir_sign: Optional[int] = None) -> float:
    """
    回傳實際走的馬達角（deg）。
    - only_tighten=True 時：僅允許沿收緊方向移動；反向補償量會被抑制為 0。
    - force_dir_sign 可強制指定收緊方向（+1=馬達正向, -1=馬達負向）。若不給，ABD=+1、ADD=-1。
    """
    comp_deg, _ = compute_compensation_deg(mode, ph, pk, enc)
    if abs(comp_deg) < 1e-6:
        return 0.0

    # 只收不放：把幾何補償投影到「收緊方向」
    if only_tighten:
        desired = force_dir_sign if force_dir_sign in (+1, -1) else (+1 if mode == "abd" else -1)
        comp_deg = math.copysign(max(0.0, desired * comp_deg), desired)  # 方向相反→0

    # 預留不走滿
    move_deg = math.copysign(max(0.0, abs(comp_deg) - abs(leave_deg)), comp_deg)
    if abs(move_deg) <= 0.0:
        return 0.0

    set_profile(ph, pk, VEL_TIGHT, ACC_TIGHT)
    tick0, _ = read_present_position(ph, pk)
    goal_tick = (tick0 + ut.deg2pos(move_deg)) & 0xFFFFFFFF
    goal_position(ph, pk, goal_tick)

    t0 = time.time()
    while True:
        curr_tick, _ = read_present_position(ph, pk)
        if abs(signed_tick_diff(goal_tick, curr_tick)) <= POS_EPS_TICK:
            break
        if time.time() - t0 > 1.5:
            break
        if abs(read_present_current_mA(ph, pk)) >= I_STOP_HARD_mA:
            set_profile(ph, pk, 60, 30)
            goal_position(ph, pk, curr_tick)
            break
        time.sleep(0.01)

    return move_deg

# ================== B) 只用於 t abd / t add 的張力-PID ==================
def tighten_pid_on_current(ph: PortHandler, pk: PacketHandler, dir_sign: int) -> bool:
    set_profile(ph, pk, PROFILE_VEL, PROFILE_ACC)  # 微調階段速度

    tick_base, _ = read_present_position(ph, pk)
    path_deg = 0.0
    last_goal_tick = tick_base

    integ = 0.0
    prev_err = None
    u_filt = 0.0
    t0 = time.time()

    last_i = abs(read_present_current_mA(ph, pk))
    spike_hold = 0
    DT_LOC = 0.01

    while True:
        if time.time() - t0 > T_PID_TIMEOUT_S:
            print("[WARN] 收線 PID 超時"); return False

        i_now = abs(read_present_current_mA(ph, pk))
        if i_now >= I_STOP_HARD_mA:
            print(f"[SAFE] |I|={i_now:.1f} mA 過大"); return False

        # 停車條件 A：上一筆跳增
        if (i_now - last_i) >= SPIKE_MARGIN_mA:
            spike_hold += 1
        else:
            spike_hold = 0
        if spike_hold >= EMA_HOLD_COUNT:
            print(f"[TIGHT] ΔI 跳增達門檻：{last_i:.1f}→{i_now:.1f} mA (Δ≈{i_now-last_i:.1f}≥{SPIKE_MARGIN_mA}) → 停")
            return True

        # 停車條件 B：超過目標太多（避免過拉）
        if i_now >= (I_TARGET_mA + OVERSHOOT_STOP_mA):
            print(f"[TIGHT] |I| 達目標附近：now≈{i_now:.1f} mA ≥ I*({I_TARGET_mA:.1f}+{OVERSHOOT_STOP_mA:.1f}) → 停")
            return True

        # PID（只收不放）
        err   = I_TARGET_mA - i_now
        deriv = 0.0 if prev_err is None else (err - prev_err) / DT_LOC
        prev_err = err
        integ = max(-T_INTEG_CLAMP, min(T_INTEG_CLAMP, integ + err * DT_LOC))

        u_deg = T_KP * err + T_KI * integ + T_KD * deriv
        u_deg = max(0.0, u_deg)

        u_filt = (1 - T_SMOOTH_BETA) * u_filt + T_SMOOTH_BETA * u_deg
        step_deg = min(u_filt, T_MICRO_STEP_DEG)

        if step_deg > 0.0:
            step_tick = ut.deg2pos(step_deg) * (1 if dir_sign > 0 else -1)
            path_deg += ut.pos2deg(step_tick, extend=True)
            goal_tick = (tick_base + ut.deg2pos(path_deg)) & 0xFFFFFFFF
            if goal_tick != last_goal_tick:
                goal_position(ph, pk, goal_tick)
                last_goal_tick = goal_tick

        last_i = i_now
        time.sleep(DT_LOC)

def tighten_only(mode: str, ph: PortHandler, pk: PacketHandler, enc: EncoderStream) -> bool:
    """先補償（可留一段），再用張力PID把線拉緊就停止。"""
    mode = mode.lower(); assert mode in ("abd","add")
    comp_deg, _ = compute_compensation_deg(mode, ph, pk, enc)
    comp_sign = 1 if comp_deg > 0 else (-1 if comp_deg < 0 else 0)
    fallback_sign = CAL.get("TIGHT_DIR", {}).get(mode, +1 if mode=="abd" else -1)
    dir_sign = comp_sign if abs(comp_deg) >= 0.4 else fallback_sign

    print(f"[COMP] {mode.upper()} 原補償 {comp_deg:+.2f}° → 部分/走滿（保留 {LEAVE_COMP_DEG}°）")
    compensate_only(mode, ph, pk, enc, leave_deg=LEAVE_COMP_DEG)

    ok = tighten_pid_on_current(ph, pk, dir_sign)
    # if ok:
    #     CAL.setdefault("TIGHT_DIR", {})[mode] = dir_sign
        # save_calibration()
    return ok

# ================== 角度 PID（以 encoder 為輸入） ==================
def move_encoder_to_target_pid(target_enc_deg: float,
                               ph: PortHandler, pk: PacketHandler,
                               enc: EncoderStream,
                               vel: int, acc: int) -> bool:
    """把 encoder 對到 target（±0.5°內且連續維持 150 ms）。"""
    set_profile(ph, pk, vel, acc)

    unwrap = EncUnwrap()
    en0 = enc.read_deg()
    if en0 is None:
        print("[ERR] 讀不到 encoder"); return False
    _ = unwrap.update(en0)

    tick_base, _ = read_present_position(ph, pk)
    path_mot_deg = 0.0
    last_goal_tick = tick_base

    TOL_CLOSE = 0.5
    HOLD_TIME = 0.15
    hold_t = 0.0

    TAU_D = 0.025
    ALPHA_D = DT / (TAU_D + DT)

    KI_NEAR = 0.02
    I_CLAMP_NEAR = 0.8
    PID_I_CLAMP = 2.0

    integ = 0.0
    prev_err = None
    prev_y = None
    d_y_prev = 0.0

    t0 = time.time()
    next_t = t0

    while True:
        if time.time() - t0 > PID_TIMEOUT_S:
            print("[WARN] PID 超時"); return False

        en_now = unwrap.update(enc.read_deg())
        err = target_enc_deg - en_now
        aerr = abs(err)

        if aerr <= TOL_CLOSE:
            hold_t += DT
            if hold_t >= HOLD_TIME:
                return True
        else:
            hold_t = 0.0

        dy = 0.0 if prev_y is None else (en_now - prev_y) / DT
        prev_y = en_now
        d_y_filt = d_y_prev + ALPHA_D * (dy - d_y_prev)
        d_y_prev = d_y_filt

        if aerr > 8.0:
            kp, kd, step_cap = 1.2, 0.05, 15.0
            nocross_factor = 0.70
        elif aerr > 3.0:
            kp, kd, step_cap = 0.9, 0.04, 9.0
            nocross_factor = 0.70
        else:
            kp, kd, step_cap = 0.6, 0.02, 2.0
            nocross_factor = 0.70

        u_pd_enc = kp * err - kd * d_y_filt
        would_saturate = (abs(K_MOTOR_PER_ENC * u_pd_enc) > (step_cap + 1e-6))
        crossing = (prev_err is not None and (err * prev_err) < 0)

        ki = KI_NEAR if aerr < 3.0 else 0.0
        i_clamp = I_CLAMP_NEAR if aerr < 3.0 else PID_I_CLAMP
        if (not would_saturate) and (not crossing):
            integ = max(-i_clamp, min(i_clamp, integ + err * DT))

        u_enc = kp * err + ki * integ - kd * d_y_filt
        u_motor = K_MOTOR_PER_ENC * u_enc

        step = u_motor
        if step > step_cap: step = step_cap
        if step < -step_cap: step = -step_cap

        max_no_cross = nocross_factor * aerr * abs(K_MOTOR_PER_ENC)
        if abs(step) > max_no_cross:
            step = math.copysign(max_no_cross, step)

        if step != 0.0:
            step_tick = ut.deg2pos(step)
            path_mot_deg += ut.pos2deg(step_tick, extend=True)
            goal_tick = (tick_base + ut.deg2pos(path_mot_deg)) & 0xFFFFFFFF
            if goal_tick != last_goal_tick:
                goal_position(ph, pk, goal_tick)
                last_goal_tick = goal_tick

        prev_err = err

        next_t += DT
        delay = next_t - time.time()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.time()

# ===== Unity one-shot entry (no hardware; just echo) =====
# def run_unity_once():
#     """
#     用參數 --angle <deg> 接收 Unity 的角度（可負），不做馬達動作，
#     直接輸出一行 `UNITY_RESULT <數值>` 後結束。
#     """
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--angle", type=float, required=True, help="relative angle (deg), can be negative")
#     parser.add_argument("--echo-mult", dest="echo_mult", type=float, default=1.0, help="result = angle * echo-mult")
#     parser.add_argument("--dry", action="store_true", default=True, help="stay in dry-run (no motors)")
#     args, _ = parser.parse_known_args()

#     # 乾跑：只回傳結果（不要動硬體）
#     result = args.angle * args.echo_mult
#     print(f"UNITY_RESULT {result:.3f}")
#     return 0

def run_unity_once():
    parser = argparse.ArgumentParser()
    parser.add_argument("--angle", type=float, required=True, help="relative angle (deg), can be negative")
    args, _ = parser.parse_known_args()

    # 初始化硬體
    ph, pk = init_dxl()
    enc = EncoderStream(ENCODER_PORT, ENCODER_BAUD)
    load_calibration()

    delta = args.angle
    unwrap_once = EncUnwrap()

    en_start = enc.read_deg()
    if en_start is None:
        print("UNITY_RESULT ERR_NO_ENCODER")
        set_torque(ph, pk, 0)
        ph.closePort()
        enc.close()
        return 1

    en_start_u = unwrap_once.update(en_start)
    target_enc = en_start_u + delta

    # 先判斷方向：delta<0 → ABD；delta>0 → ADD
    mode_tight = "abd" if delta < 0 else "add"
    print(f"[UNITY] MOVE {delta:+.2f}° (mode={mode_tight})")

    # 收線補償
    compensate_only(mode_tight, ph, pk, enc, leave_deg=LEAVE_COMP_DEG)

    # PID 控制到目標
    ok = move_encoder_to_target_pid(target_enc, ph, pk, enc, PROFILE_VEL, PROFILE_ACC)

    # 運轉結束後讀取 encoder
    time.sleep(0.08)
    en_end = enc.read_deg()

    if en_end is not None:
        delta_real = en_end - en_start
        if ok:
            print(f"{delta_real:.3f}")
        else:
            print(f"UNITY_RESULT FAIL_{delta_real:.3f}")
    else:
        print("UNITY_RESULT ERR_NO_END")

    # 收尾
    try:
        set_torque(ph, pk, 0)
    except Exception:
        pass
    try:
        ph.closePort()
    except Exception:
        pass
    enc.close()
    return 0


# ================== CLI 整合 ==================
def run_cli():
    ph, pk = init_dxl()
    enc = EncoderStream(ENCODER_PORT, ENCODER_BAUD)
    load_calibration()

    print("指令：'cal abd' / 'cal add' / 't abd' / 't add' / 整數(±度) / "
          "'speed <v>' / 'acc <a>' / 'tightvel <v>' / 'tightacc <a>' / 'leave <deg>' / 'q'")
    print(f"[INIT] PID vel/acc=({PROFILE_VEL},{PROFILE_ACC}); TIGHT vel/acc=({VEL_TIGHT},{ACC_TIGHT}); leave={LEAVE_COMP_DEG}°")

    try:
        while True:
            cmd = input("cmd> ").strip().lower()
            if cmd == "q": break

            if cmd in ("cal abd", "cal add"):
                mode = "abd" if cmd.endswith("abd") else "add"
                calibrate_once(mode, ph, pk, enc); continue

            if cmd in ("t abd", "t add"):
                mode = "abd" if cmd.endswith("abd") else "add"
                t0 = time.time()
                ok = tighten_only(mode, ph, pk, enc)
                print(f"[RESULT] tighten {mode.upper()} → {'OK' if ok else 'FAIL'} (elapsed {time.time()-t0:.2f}s)")
                continue

            # 調速度 / 參數
            if cmd.startswith("speed "):
                try:
                    v = int(cmd.split()[1]); globals()['PROFILE_VEL'] = v
                    set_profile(ph, pk, PROFILE_VEL, PROFILE_ACC)
                    print(f"[OK] PROFILE_VEL = {PROFILE_VEL}")
                except: print("用法：speed 800")
                continue

            if cmd.startswith("acc "):
                try:
                    a = int(cmd.split()[1]); globals()['PROFILE_ACC'] = a
                    set_profile(ph, pk, PROFILE_VEL, PROFILE_ACC)
                    print(f"[OK] PROFILE_ACC = {PROFILE_ACC}")
                except: print("用法：acc 100")
                continue

            if cmd.startswith("tightvel "):
                try:
                    v = int(cmd.split()[1]); globals()['VEL_TIGHT'] = v
                    set_profile(ph, pk, VEL_TIGHT, ACC_TIGHT)
                    print(f"[OK] VEL_TIGHT = {VEL_TIGHT}")
                except: print("用法：tightvel 500")
                continue

            if cmd.startswith("tightacc "):
                try:
                    a = int(cmd.split()[1]); globals()['ACC_TIGHT'] = a
                    set_profile(ph, pk, VEL_TIGHT, ACC_TIGHT)
                    print(f"[OK] ACC_TIGHT = {ACC_TIGHT}")
                except: print("用法：tightacc 180")
                continue

            if cmd.startswith("leave "):
                try:
                    d = float(cmd.split()[1]); globals()['LEAVE_COMP_DEG'] = d
                    print(f"[OK] LEAVE_COMP_DEG = {LEAVE_COMP_DEG}°")
                except: print("用法：leave 0.0")
                continue

            # 相對角度命令
            try:
                delta = float(cmd)
            except Exception:
                print("請輸入：整數(±度) / cal abd / cal add / t abd / t add / speed / acc / tightvel / tightacc / leave / q")
                continue

            # 1) 讀起點 + 設目標（連續角）
            unwrap_once = EncUnwrap()
            en_start = enc.read_deg()
            if en_start is None:
                print("[ERR] 讀不到 encoder"); continue
            en_start_u = unwrap_once.update(en_start)
            target_enc = en_start_u + delta

            # 2) 選側 & 先收線（幾何補償；預設走滿）
            mode_tight = "abd" if delta < 0 else "add"
            desired_sign = +1 if mode_tight == "abd" else -1  # ABD=馬達正向, ADD=馬達負向
            print(f"[MOVE] 先收線({mode_tight.upper()} 補償, leave={LEAVE_COMP_DEG}°) → 再對到 target={target_enc:.2f}°")
            compensate_only(mode_tight, ph, pk, enc, leave_deg=LEAVE_COMP_DEG)

            # 3) 角度 PID 對準（把收線造成的位移自然算進去）
            ok = move_encoder_to_target_pid(target_enc, ph, pk, enc, PROFILE_VEL, PROFILE_ACC)
            time.sleep(0.08)
            en_end = enc.read_deg()
            if en_end is not None:
                print(f"[ENC] before={en_start:.2f}°, after={en_end:.2f}°, Δ={en_end - en_start:+.3f}°（期望 Δ={delta:+.3f}°）")
            if not ok:
                print("[INFO] 未在超時內收斂，可調 KP/速度或放寬誤差帶")

    finally:
        try: set_torque(ph, pk, 0)
        except Exception: pass
        try: ph.closePort()
        except Exception: pass
        enc.close()

# ================== 進入點 ==================
if __name__ == "__main__":
    if any(arg.startswith("--angle") for arg in sys.argv):
        raise SystemExit(run_unity_once())
    else:
        run_cli()

