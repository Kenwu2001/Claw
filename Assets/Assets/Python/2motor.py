# -*- coding: utf-8 -*-
"""
實驗用精簡版控制（雙馬達 / 二次調速 / 報表 / 自動回位 / trial後可自動t收線）：
  指令：
    - cal abd / cal add / cal init / reset / t abd / t add
    - [d7, d9] 或 [d7, d9, flag]：
        d7→ID=7 收線+encoder-PID；d9→ID=9 直接相對角度
        例：[-5,0]     : ID=7 收線後對到 encoder -5°；ID=9 不動
            [-5,10]    : ID=7 如上；ID=9 直接 +10°（ID7<0 等補償後再啟動；ID7>0 同開始含二次調速）
            [-5,90,1]  : trial 完成且回位後，再自動做 t abd
            [-5,90,0]  : trial 完成且回位後，再自動做 t add

  啟動：
    - 連線、讀 calibration.json
    - 若已 cal init 且 AUTO_RESET=True，啟動即自動 reset
    流程：
    1. Python 收到 list → 執行收線 (若 ab_ad_angle!=0)
    2. 收線完成 → print("READY") → Unity 開始動畫
    3. Unity 動畫結束 → 再送 "ANIM_DONE"
    4. Python 控馬達完成後 reset，並回傳 "DONE {...}" JSON 給 Unity
"""

import os, sys, json, time, math, re, serial, threading, argparse
from typing import Optional, Tuple
from dynamixel_sdk import *  # PortHandler, PacketHandler
import utils as ut
import struct

# ========= 全域 I/O 鎖（解決 comm=-1000：port busy） =========
IO_LOCK = threading.Lock()

# ================== 連線 / 兩顆馬達 ID ==================
PORT_NAME = "COM4"
BAUDRATE  = 57600
# ================== 馬達群組 ==================
MODE1_IDS = [6,7,8,9]      # 收線 + encoder-PID
MODE2_IDS = [1,2,3,4,5]    # 直接轉角度
LEADER_ID = 6              # 收線模式選一顆當 leader（建議 ID=6）

# ================== DXL 控制表 ==================
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
ENC_INSTANCE = None

# ================== 檔案 ==================
CAL_FILE = "calibration.json"

# ================== 幾何比例 ==================
K_MOTOR_PER_ENC = -4.0   # motor +120° ≈ encoder -30° → 比例 -4（motor+ 會讓 encoder 減小）

# ================== 速度 / 超時 ==================
DT = 0.01                 # 100 Hz
PID_TIMEOUT_S = 5.0
PROFILE_VEL_DEFAULT = 800
PROFILE_ACC_DEFAULT = 100

VEL_TIGHT   = 1000         # 幾何補償 profile
ACC_TIGHT   = 300
I_STOP_HARD_mA = 300.0     # 過流斷
POS_EPS_TICK   = 20        # 到位容差
LEAVE_COMP_DEG = 0.0       # 預設走滿

# ====== v9 上下限與同步（預熱） ======
V9_VEL_MIN = 40
V9_VEL_MAX = 3500
V9_PREHEAT_FACTOR = 0.35  # ID7>0 同步時，ID9 以此比例的低速先跑

# 自動回位設定
AUTO_RESET_AFTER_T_ABD_ADD = False  # t abd / t add 完成後是否自動 reset（維持 False）
AUTO_RESET = True                   # trial（[d7,d9,(flag)]）完成後是否自動 reset

# ================== 張力-PID（僅 t abd/t add 用，ID=7） ==================
FORCE_TARGET_N      = 0.8
SPOOL_RADIUS_MM     = 2.5
KT_NM_PER_A         = 0.347
TENSION_EFF_GAIN    = 1.5
SPIKE_MARGIN_mA     = 3.0
OVERSHOOT_STOP_mA   = 2.0
T_KP, T_KI, T_KD    = 0.10, 0.015, 0.02
T_INTEG_CLAMP       = 300.0
T_MICRO_STEP_DEG    = 4.0
T_PID_TIMEOUT_S     = 4.0
T_SMOOTH_BETA       = 0.40
EMA_HOLD_COUNT      = 3

def forceN_to_mA(FN: float) -> float:
    return (FN * (SPOOL_RADIUS_MM/1000.0) / KT_NM_PER_A) * 1000.0 * TENSION_EFF_GAIN
I_TARGET_mA = forceN_to_mA(FORCE_TARGET_N)

# ================== 校正資料 ==================
CAL = {
    "TIGHT_ABD_L": None, "TIGHT_ABD_M": None,
    "TIGHT_ADD_L": None, "TIGHT_ADD_M": None,
    "RADIUS_L": 10.0, "RADIUS_M": 2.5,
    "TIGHT_DIR": {"abd": +1, "add": -1},
    # 實驗 trial 起始位姿
    "TRIAL_INIT_M": None,         # ID7 motor 絕對 tick
    "TRIAL_INIT_L_DEG": None,     # encoder deg（ID7）
    "TRIAL_INIT_M9": None         # ID9 motor 絕對 tick
}

# ================== Encoder 串流 & 展開 ==================
class EncoderStream:
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
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("[ENCODER] port closed")
        except Exception as e:
            print(f"[ENCODER] close fail：{e}")

class EncUnwrap:
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

# ================== DXL 小工具（全部上鎖、支援任意 ID） ==================
def set_profile(ph: PortHandler, pk: PacketHandler, dxl_id: int, vel: int, acc: int) -> None:
    with IO_LOCK:
        comm, err = pk.write4ByteTxRx(ph, dxl_id, ADDR_PROFILE_VELOCITY, int(vel))
    if comm != COMM_SUCCESS or err != 0:
        print(f"[DXL {dxl_id}] set VEL fail: comm={comm}, err={err}")
    time.sleep(0.005)
    with IO_LOCK:
        comm, err = pk.write4ByteTxRx(ph, dxl_id, ADDR_PROFILE_ACCEL, int(acc))
    if comm != COMM_SUCCESS or err != 0:
        print(f"[DXL {dxl_id}] set ACC fail: comm={comm}, err={err}")
    time.sleep(0.005)

def goal_position(ph: PortHandler, pk: PacketHandler, dxl_id: int, goal_tick: int) -> None:
    with IO_LOCK:
        pk.write4ByteTxRx(ph, dxl_id, ADDR_GOAL_POSITION, int(goal_tick) & 0xFFFFFFFF)

def read_present_position(ph: PortHandler, pk: PacketHandler, dxl_id: int) -> Tuple[int, float]:
    with IO_LOCK:
        pos_tick, comm, err = pk.read4ByteTxRx(ph, dxl_id, ADDR_PRESENT_POSITION)
    if comm != COMM_SUCCESS or err != 0:
        raise RuntimeError(f"[DXL {dxl_id}] read pos error: comm={comm}, err={err}")
    return pos_tick, ut.pos2deg(pos_tick)

def read_present_current_mA(ph: PortHandler, pk: PacketHandler, dxl_id: int) -> float:
    with IO_LOCK:
        raw, comm, err = pk.read2ByteTxRx(ph, dxl_id, ADDR_PRESENT_CURRENT)
    if comm != COMM_SUCCESS or err != 0:
        raise RuntimeError(f"[DXL {dxl_id}] read current error: comm={comm}, err={err}")
    if raw > 32767: raw -= 65536
    return float(raw)  # XL330：1 mA/LSB

def set_torque(ph: PortHandler, pk: PacketHandler, dxl_id: int, enable: int):
    with IO_LOCK:
        pk.write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 1 if enable else 0)
    time.sleep(0.01)

def init_dxl():
    global ENC_INSTANCE
    ph = PortHandler(PORT_NAME)
    pk = PacketHandler(2.0)
    if not ph.openPort():  print("Failed to open port"); sys.exit(1)
    if not ph.setBaudRate(BAUDRATE): print("Failed to set baudrate"); sys.exit(1)
    for dxl_id in (DXL_IDS_MODE1, DXL_IDS_MODE2):
        with IO_LOCK:
            pk.write1ByteTxRx(ph, dxl_id, ADDR_OPERATING_MODE, 4)  # Extended Position
        time.sleep(0.02)
        set_torque(ph, pk, dxl_id, 1)
        set_profile(ph, pk, dxl_id, PROFILE_VEL_DEFAULT, PROFILE_ACC_DEFAULT)
    
        # Encoder 只初始化一次
        if ENC_INSTANCE is None:
            try:
                ENC_INSTANCE = EncoderStream(ENCODER_PORT, ENCODER_BAUD)
            except Exception as e:
                print(f"[ERR] can't open encoder ({ENCODER_PORT}): {e}")
                ENC_INSTANCE = None

    return ph, pk, ENC_INSTANCE

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
    set_torque(ph, pk, DXL_ID_MODE1, 0)
    print(f"[CAL] 請手動移到 {mode.upper()} 基準姿勢後按 Enter...")
    input()
    motor_tick, _ = read_present_position(ph, pk, DXL_ID_MODE1)
    enc_deg = enc.read_deg()
    if enc_deg is None:
        print("[CAL] 讀不到編碼器角度，校正中止。")
        set_torque(ph, pk, DXL_ID_MODE1, 1); return False
    if mode == "abd":
        CAL["TIGHT_ABD_M"] = int(motor_tick); CAL["TIGHT_ABD_L"] = float(enc_deg)
    else:
        CAL["TIGHT_ADD_M"] = int(motor_tick); CAL["TIGHT_ADD_L"] = float(enc_deg)
    print(f"[CAL] {mode.upper()} 基準：M={motor_tick} tick, L={enc_deg:.3f} deg")
    save_calibration()
    set_torque(ph, pk, DXL_ID_MODE1, 1)
    set_profile(ph, pk, DXL_ID_MODE1, PROFILE_VEL_DEFAULT, PROFILE_ACC_DEFAULT)
    return True

def calibrate_trial_initial(ph: PortHandler, pk: PacketHandler, enc: EncoderStream) -> bool:
    set_torque(ph, pk, DXL_ID_MODE1, 0); set_torque(ph, pk, DXL_ID_MODE2, 0)
    print("[CAL INIT] 請手動移到該使用者的 trial 起始姿勢（兩顆馬達皆就位）後按 Enter...")
    input()
    m7_tick, _ = read_present_position(ph, pk, DXL_ID_MODE1)
    m9_tick, _ = read_present_position(ph, pk, DXL_ID_MODE2)
    enc_deg = enc.read_deg()
    CAL["TRIAL_INIT_M"]     = int(m7_tick)
    CAL["TRIAL_INIT_L_DEG"] = float(enc_deg) if enc_deg is not None else None
    CAL["TRIAL_INIT_M9"]    = int(m9_tick)
    print(f"[CAL INIT] 設定完成：ID7 motor={m7_tick} tick, encoder={enc_deg if enc_deg is not None else 'n/a'} deg；ID9 motor={m9_tick} tick")
    save_calibration()
    set_torque(ph, pk, DXL_ID_MODE1, 1); set_torque(ph, pk, DXL_ID_MODE2, 1)
    set_profile(ph, pk, DXL_ID_MODE1, PROFILE_VEL_DEFAULT, PROFILE_ACC_DEFAULT)
    set_profile(ph, pk, DXL_ID_MODE2, PROFILE_VEL_DEFAULT, PROFILE_ACC_DEFAULT)
    return True

# ================== 幾何 / 補償 ==================
def _deg_to_arc_mm(deg: float, radius_mm: float) -> float:
    return 2.0 * math.pi * radius_mm * (deg / 360.0)
def _arc_mm_to_deg(arc_mm: float, radius_mm: float) -> float:
    return (arc_mm / (2.0 * math.pi * radius_mm)) * 360.0
def _tick_delta_to_deg(curr_tick: int, base_tick: int) -> float:
    delta_u32  = (int(curr_tick) - int(base_tick)) & 0xFFFFFFFF
    delta_tick = delta_u32 - 0x100000000 if delta_u32 >= 0x80000000 else delta_u32
    return ut.pos2deg(delta_tick, extend=True)

def compute_compensation_deg(mode: str, ph: PortHandler, pk: PacketHandler, enc: EncoderStream) -> Tuple[float, int]:
    assert mode in ("abd","add")
    base_M = CAL["TIGHT_ABD_M"] if mode=="abd" else CAL["TIGHT_ADD_M"]
    base_L = CAL["TIGHT_ABD_L"] if mode=="abd" else CAL["TIGHT_ADD_L"]
    R_L, R_M = CAL["RADIUS_L"], CAL["RADIUS_M"]
    if base_M is None or base_L is None:
        raise RuntimeError("尚未做 cal abd/add")
    motor_tick, _ = read_present_position(ph, pk, DXL_ID_MODE1)
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

def compensate_only(mode: str, ph: PortHandler, pk: PacketHandler, enc: EncoderStream,
                    leave_deg: float = LEAVE_COMP_DEG, only_tighten: bool = True) -> float:
    comp_deg, _ = compute_compensation_deg(mode, ph, pk, enc)
    if abs(comp_deg) < 1e-6: return 0.0
    desired = (+1 if mode == "abd" else -1)
    if only_tighten:
        comp_deg = math.copysign(max(0.0, desired * comp_deg), desired)
    move_deg = math.copysign(max(0.0, abs(comp_deg) - abs(leave_deg)), comp_deg)
    if abs(move_deg) <= 0.0: return 0.0
    set_profile(ph, pk, DXL_ID_MODE1, VEL_TIGHT, ACC_TIGHT)
    tick0, _ = read_present_position(ph, pk, DXL_ID_MODE1)
    goal_tick = (tick0 + ut.deg2pos(move_deg)) & 0xFFFFFFFF
    goal_position(ph, pk, DXL_ID_MODE1, goal_tick)
    t0 = time.time()
    while True:
        curr_tick, _ = read_present_position(ph, pk, DXL_ID_MODE1)
        if abs(signed_tick_diff(goal_tick, curr_tick)) <= POS_EPS_TICK: break
        if time.time() - t0 > 1.5: break
        if abs(read_present_current_mA(ph, pk, DXL_ID_MODE1)) >= I_STOP_HARD_mA:
            set_profile(ph, pk, DXL_ID_MODE1, 60, 30); goal_position(ph, pk, DXL_ID_MODE1, curr_tick); break
        time.sleep(0.01)
    return move_deg

# ================== t abd / t add（ID=7 張力控制） ==================
def tighten_pid_on_current(ph: PortHandler, pk: PacketHandler, dir_sign: int) -> bool:
    set_profile(ph, pk, DXL_ID_MODE1, PROFILE_VEL_DEFAULT, PROFILE_ACC_DEFAULT)
    tick_base, _ = read_present_position(ph, pk, DXL_ID_MODE1)
    path_deg = 0.0; last_goal_tick = tick_base
    integ = 0.0; prev_err = None; u_filt = 0.0
    t0 = time.time()
    last_i = abs(read_present_current_mA(ph, pk, DXL_ID_MODE1))
    spike_hold = 0; DT_LOC = 0.01
    while True:
        if time.time() - t0 > T_PID_TIMEOUT_S:
            print("[WARN] 收線 PID 超時"); return False
        i_now = abs(read_present_current_mA(ph, pk, DXL_ID_MODE1))
        if i_now >= I_STOP_HARD_mA:
            print(f"[SAFE] |I|={i_now:.1f} mA 過大"); return False
        if (i_now - last_i) >= SPIKE_MARGIN_mA: spike_hold += 1
        else: spike_hold = 0
        if spike_hold >= EMA_HOLD_COUNT:
            print(f"[TIGHT] ΔI 跳增達門檻 → 停"); return True
        if i_now >= (I_TARGET_mA + OVERSHOOT_STOP_mA):
            print(f"[TIGHT] |I| 達目標附近 → 停"); return True
        err = I_TARGET_mA - i_now
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
                goal_position(ph, pk, DXL_ID_MODE1, goal_tick); last_goal_tick = goal_tick
        last_i = i_now
        time.sleep(0.01)

def tighten_only(mode: str, ph: PortHandler, pk: PacketHandler, enc: EncoderStream) -> bool:
    mode = mode.lower(); assert mode in ("abd","add")
    comp_deg, _ = compute_compensation_deg(mode, ph, pk, enc)
    comp_sign = 1 if comp_deg > 0 else (-1 if comp_deg < 0 else 0)
    fallback_sign = CAL.get("TIGHT_DIR", {}).get(mode, +1 if mode=="abd" else -1)
    dir_sign = comp_sign if abs(comp_deg) >= 0.4 else fallback_sign
    print(f".[COMP] {mode.upper()} 原補償 {comp_deg:+.2f}° → 部分/走滿（保留 {LEAVE_COMP_DEG}°）")
    compensate_only(mode, ph, pk, enc, leave_deg=LEAVE_COMP_DEG)
    return tighten_pid_on_current(ph, pk, dir_sign)

# ================== ID=7 Encoder-PID ==================
def move_encoder_to_target_pid(target_enc_deg: float,
                               ph: PortHandler, pk: PacketHandler,
                               enc: EncoderStream,
                               vel: int, acc: int) -> bool:
    set_profile(ph, pk, DXL_ID_MODE1, vel, acc)
    unwrap = EncUnwrap()
    en0 = enc.read_deg()
    if en0 is None:
        print("[ERR] 讀不到 encoder"); return False
    _ = unwrap.update(en0)
    tick_base, _ = read_present_position(ph, pk, DXL_ID_MODE1)
    path_mot_deg = 0.0; last_goal_tick = tick_base
    TOL_CLOSE = 0.5; HOLD_TIME = 0.15; hold_t = 0.0
    TAU_D = 0.025; ALPHA_D = DT / (TAU_D + DT)
    KI_NEAR = 0.02; I_CLAMP_NEAR = 0.8; PID_I_CLAMP = 2.0
    integ = 0.0; prev_err = None; prev_y = None; d_y_prev = 0.0
    t0 = time.time(); next_t = t0
    while True:
        if time.time() - t0 > PID_TIMEOUT_S:
            print("[WARN] PID 超時"); return False
        en_now = unwrap.update(enc.read_deg())
        err = target_enc_deg - en_now; aerr = abs(err)
        if aerr <= TOL_CLOSE:
            hold_t += DT
            if hold_t >= HOLD_TIME: return True
        else:
            hold_t = 0.0
        dy = 0.0 if prev_y is None else (en_now - prev_y) / DT
        prev_y = en_now
        d_y_filt = d_y_prev + ALPHA_D * (dy - d_y_prev); d_y_prev = d_y_filt
        if aerr > 8.0:  kp, kd, step_cap = 1.2, 0.05, 15.0
        elif aerr > 3.0: kp, kd, step_cap = 0.9, 0.04, 9.0
        else:            kp, kd, step_cap = 0.6, 0.02, 2.0
        nocross_factor = 0.70
        u_pd_enc = kp * err - kd * d_y_filt
        crossing = (prev_err is not None and (err * prev_err) < 0)
        ki = KI_NEAR if aerr < 3.0 else 0.0
        i_clamp = I_CLAMP_NEAR if aerr < 3.0 else PID_I_CLAMP
        would_saturate = (abs(K_MOTOR_PER_ENC * u_pd_enc) > (step_cap + 1e-6))
        if (not would_saturate) and (not crossing):
            integ = max(-i_clamp, min(i_clamp, integ + err * DT))
        u_enc = kp * err + ki * integ - kd * d_y_filt
        u_motor = K_MOTOR_PER_ENC * u_enc
        step = max(-step_cap, min(step_cap, u_motor))
        max_no_cross = nocross_factor * aerr * abs(K_MOTOR_PER_ENC)
        if abs(step) > max_no_cross:
            step = math.copysign(max_no_cross, step)
        if step != 0.0:
            step_tick = ut.deg2pos(step)
            path_mot_deg += ut.pos2deg(step_tick, extend=True)
            goal_tick = (tick_base + ut.deg2pos(path_mot_deg)) & 0xFFFFFFFF
            if goal_tick != last_goal_tick:
                goal_position(ph, pk, DXL_ID_MODE1, goal_tick); last_goal_tick = goal_tick
        prev_err = err
        next_t += DT; delay = next_t - time.time()
        if delay > 0: time.sleep(delay)
        else: next_t = time.time()

# ================== 馬達移動工具 ==================
def move_motor_relative_deg(ph: PortHandler, pk: PacketHandler, dxl_id: int,
                            delta_deg: float, vel: int, acc: int,
                            timeout_s: float = 6.0) -> bool:
    set_profile(ph, pk, dxl_id, vel, acc)
    tick0, _ = read_present_position(ph, pk, dxl_id)
    goal_tick = (tick0 + ut.deg2pos(delta_deg)) & 0xFFFFFFFF
    goal_position(ph, pk, dxl_id, goal_tick)
    t0 = time.time()
    while True:
        curr_tick, _ = read_present_position(ph, pk, dxl_id)
        if abs(signed_tick_diff(goal_tick, curr_tick)) <= POS_EPS_TICK: return True
        if time.time() - t0 > timeout_s:
            print(f"[DXL {dxl_id}] move timeout; stop at current")
            goal_position(ph, pk, dxl_id, curr_tick); return False
        if abs(read_present_current_mA(ph, pk, dxl_id)) >= I_STOP_HARD_mA:
            print(f"[DXL {dxl_id}] overcurrent; abort")
            goal_position(ph, pk, dxl_id, curr_tick); return False
        time.sleep(0.01)

def move_motor_to_abs_tick(ph: PortHandler, pk: PacketHandler, dxl_id: int,
                           target_tick: int, vel: int, acc: int,
                           timeout_s: float = 6.0) -> bool:
    set_profile(ph, pk, dxl_id, vel, acc)
    goal_position(ph, pk, dxl_id, int(target_tick) & 0xFFFFFFFF)
    t0 = time.time()
    while True:
        curr_tick, _ = read_present_position(ph, pk, dxl_id)
        if abs(signed_tick_diff(int(target_tick), curr_tick)) <= POS_EPS_TICK: return True
        if time.time() - t0 > timeout_s:
            print(f"[DXL {dxl_id}] abs move timeout; stop at current")
            goal_position(ph, pk, dxl_id, curr_tick); return False
        if abs(read_present_current_mA(ph, pk, dxl_id)) >= I_STOP_HARD_mA:
            print(f"[DXL {dxl_id}] overcurrent; abort")
            goal_position(ph, pk, dxl_id, curr_tick); return False
        time.sleep(0.01)

# ================== RESET（同步 motor 回位 → ID7 再 PID） ==================
def reset_to_trial_initial_both(ph: PortHandler, pk: PacketHandler, enc: EncoderStream) -> None:
    m7 = CAL.get("TRIAL_INIT_M", None)
    e7 = CAL.get("TRIAL_INIT_L_DEG", None)
    m9 = CAL.get("TRIAL_INIT_M9", None)
    print("[RESET] 同步返回 trial 初始 motor 位置（ID7 & ID9），之後 ID7 做 encoder-PID 微調")

    results = {"id7_motor": True, "id9_motor": True}

    def _go_id7_motor():
        if m7 is None:
            print("[RESET][ID7] 未設定 TRIAL_INIT_M，略過 motor 回位"); return
        ok = move_motor_to_abs_tick(ph, pk, DXL_ID_MODE1, m7, PROFILE_VEL_DEFAULT, PROFILE_ACC_DEFAULT, timeout_s=6.0)
        results["id7_motor"] = ok
        print(f"[RESET][ID7] motor→{'OK' if ok else 'FAIL'}")

    def _go_id9_motor():
        if m9 is None:
            print("[RESET][ID9] 未設定 TRIAL_INIT_M9，略過 motor 回位"); return
        ok = move_motor_to_abs_tick(ph, pk, DXL_ID_MODE2, m9, PROFILE_VEL_DEFAULT, PROFILE_ACC_DEFAULT, timeout_s=6.0)
        results["id9_motor"] = ok
        print(f"[RESET][ID9] motor→{'OK' if ok else 'FAIL'}")

    t7 = threading.Thread(target=_go_id7_motor, daemon=True)
    t9 = threading.Thread(target=_go_id9_motor, daemon=True)
    t7.start(); t9.start(); t7.join(); t9.join()

    if e7 is not None:
        ok_pid = move_encoder_to_target_pid(float(e7), ph, pk, enc, vel=PROFILE_VEL_DEFAULT, acc=PROFILE_ACC_DEFAULT)
        print(f"[RESET][ID7] PID encoder→{'OK' if ok_pid else 'FAIL'}")
    else:
        print("[RESET][ID7] 未設定 TRIAL_INIT_L_DEG，略過 encoder-PID 微調")

# ================== 雙模式指令執行（含二次調速、報表、事後回位與可選 t 動作） ==================
def execute_dual_mode(ph: PortHandler, pk: PacketHandler, enc: EncoderStream,
                      delta7_deg: float, delta9_deg: float, post_t: Optional[bool] = None):
    """
    delta7_deg：ID=7 encoder 角度變化（先補償，再 PID）
    delta9_deg：ID=9 相對馬達角
    post_t：None/True/False；若非 None，trial 自動回位後再做 t abd(True) 或 t add(False)
    """
    EPS = 1e-6
    unwrap = EncUnwrap()
    en7_start_raw = enc.read_deg()
    if en7_start_raw is None:
        print("[ERR] 讀不到 encoder"); return
    en7_start = unwrap.update(en7_start_raw)
    tick9_start, _ = read_present_position(ph, pk, DXL_ID_MODE2)

    only_id9 = (abs(delta7_deg) <= EPS) and (abs(delta9_deg) > EPS)
    only_id7 = (abs(delta7_deg) > EPS) and (abs(delta9_deg) <= EPS)

    # ---------- case A：只動 ID=9 ----------
    if only_id9:
        print(f"[MOVE] 只動 ID=9，相對角度 {delta9_deg:+.3f}°")
        ok9 = move_motor_relative_deg(ph, pk, DXL_ID_MODE2, delta9_deg, PROFILE_VEL_DEFAULT, PROFILE_ACC_DEFAULT, 6.0)
        tick9_end, _ = read_present_position(ph, pk, DXL_ID_MODE2)
        d_deg9 = ut.pos2deg(signed_tick_diff(tick9_end, tick9_start), extend=True)
        d_enc7 = unwrap.update(enc.read_deg()) - en7_start
        print(f"[REPORT] ID7 Δencoder={d_enc7:+.3f}°（預期 0°）; ID9 Δmotor={d_deg9:+.3f}° → {'OK' if ok9 else 'FAIL'}")
        if AUTO_RESET:
            time.sleep(1.0); reset_to_trial_initial_both(ph, pk, enc)
            if post_t is not None:
                mode = "abd" if post_t else "add"
                # print(f"[POST] trial 回位後自動執行 t {mode.upper()}")
                # tighten_only(mode, ph, pk, enc)
        return

    # ---------- case B：只動 ID=7 ----------
    if only_id7:
        mode_tight = "abd" if delta7_deg < 0 else "add"
        print(f"[MOVE] 先收線({mode_tight.upper()} 補償, leave={LEAVE_COMP_DEG}°)")
        compensate_only(mode_tight, ph, pk, enc, leave_deg=LEAVE_COMP_DEG, only_tighten=True)
        target_enc = en7_start + delta7_deg
        ok7 = move_encoder_to_target_pid(target_enc, ph, pk, enc, PROFILE_VEL_DEFAULT, PROFILE_ACC_DEFAULT)
        tick9_end, _ = read_present_position(ph, pk, DXL_ID_MODE2)
        d_deg9 = ut.pos2deg(signed_tick_diff(tick9_end, tick9_start), extend=True)
        d_enc7 = unwrap.update(enc.read_deg()) - en7_start
        print(f"[REPORT] ID7 Δencoder={d_enc7:+.3f}° → {'OK' if ok7 else 'FAIL'}; ID9 Δmotor={d_deg9:+.3f}°（預期 0°）")
        if AUTO_RESET:
            time.sleep(1.0); reset_to_trial_initial_both(ph, pk, enc)
            if post_t is not None:
                mode = "abd" if post_t else "add"
                # print(f"[POST] trial 回位後自動執行 t {mode.upper()}")
                # tighten_only(mode, ph, pk, enc)
        return

    # ---------- case C：兩顆都動 ----------
    en_now = unwrap.update(enc.read_deg())
    target_enc_full = en_now + delta7_deg

    if delta7_deg < -EPS:
        # 先收線，再同時開始
        mode_tight = "abd"
        print(f"[MOVE] 先收線({mode_tight.upper()} 補償, leave={LEAVE_COMP_DEG}°)")
        compensate_only(mode_tight, ph, pk, enc, leave_deg=LEAVE_COMP_DEG, only_tighten=True)
        en_after = unwrap.update(enc.read_deg())
        remain_enc_deg = abs(target_enc_full - en_after)
        approx_m7 = abs(K_MOTOR_PER_ENC * remain_enc_deg)
        approx_m9 = abs(delta9_deg)
        v7 = PROFILE_VEL_DEFAULT; a = PROFILE_ACC_DEFAULT
        v9 = PROFILE_VEL_DEFAULT if (approx_m7 < EPS or approx_m9 < EPS) else \
             int(max(V9_VEL_MIN, min(V9_VEL_MAX, int(v7 * (approx_m9 / max(EPS, approx_m7))))))
        result = {"m7": None, "m9": None}
        def _run_m7(): result["m7"] = move_encoder_to_target_pid(target_enc_full, ph, pk, enc, v7, a)
        def _run_m9(): result["m9"] = move_motor_relative_deg(ph, pk, DXL_ID_MODE2, delta9_deg, v9, a, 8.0)
        t7 = threading.Thread(target=_run_m7, daemon=True)
        t9 = threading.Thread(target=_run_m9, daemon=True)
        t7.start(); t9.start(); t7.join(); t9.join()
    else:
        # 同步開始（ID9 低速預熱），收線結束二次調速
        mode_tight = "add"
        tick9_goal = (read_present_position(ph, pk, DXL_ID_MODE2)[0] + ut.deg2pos(delta9_deg)) & 0xFFFFFFFF
        v9_preheat = int(max(V9_VEL_MIN, min(V9_VEL_MAX, int(PROFILE_VEL_DEFAULT * V9_PREHEAT_FACTOR))))
        id9_running = {"ok": None}
        def _run_id9_preheat():
            id9_running["ok"] = move_motor_relative_deg(ph, pk, DXL_ID_MODE2, delta9_deg, v9_preheat, PROFILE_ACC_DEFAULT, 10.0)
        t9 = threading.Thread(target=_run_id9_preheat, daemon=True); t9.start()
        print(f"[MOVE] 同步開始：ID7 收線({mode_tight.upper()});  ID9 低速預熱 v={v9_preheat}")
        compensate_only(mode_tight, ph, pk, enc, leave_deg=LEAVE_COMP_DEG, only_tighten=True)
        en_after = unwrap.update(enc.read_deg())
        remain_enc_deg = abs(target_enc_full - en_after)
        approx_m7 = abs(K_MOTOR_PER_ENC * remain_enc_deg)
        tick9_curr, _ = read_present_position(ph, pk, DXL_ID_MODE2)
        rem_tick9 = signed_tick_diff(tick9_goal, tick9_curr)
        approx_m9 = abs(ut.pos2deg(rem_tick9, extend=True))
        v7 = PROFILE_VEL_DEFAULT
        v9_new = PROFILE_VEL_DEFAULT if (approx_m7 < EPS or approx_m9 < EPS) else \
                 int(max(V9_VEL_MIN, min(V9_VEL_MAX, int(v7 * (approx_m9 / max(EPS, approx_m7))))))
        set_profile(ph, pk, DXL_ID_MODE2, v9_new, PROFILE_ACC_DEFAULT)
        print(f"[SYNC] 二次調速：v9 從 {v9_preheat} → {v9_new}；ID7 進 PID")
        result = {"m7": None}
        def _run_m7_pid(): result["m7"] = move_encoder_to_target_pid(target_enc_full, ph, pk, enc, v7, PROFILE_ACC_DEFAULT)
        t7 = threading.Thread(target=_run_m7_pid, daemon=True); t7.start()
        t7.join(); t9.join()

    # 報表 + 自動回位 + 可選 t 動作
    tick9_end, _ = read_present_position(ph, pk, DXL_ID_MODE2)
    d_deg9 = ut.pos2deg(signed_tick_diff(tick9_end, tick9_start), extend=True)
    d_enc7 = unwrap.update(enc.read_deg()) - en7_start
    try:
        ok7 = result.get("m7"); ok9 = result.get("m9", None)
    except:
        ok7 = result.get("m7") if 'result' in locals() else None
        ok9 = id9_running.get("ok") if 'id9_running' in locals() else None
    print(f"[REPORT] ID7 Δencoder={d_enc7:+.3f}° → {'OK' if ok7 else 'FAIL' if ok7 is not None else 'N/A'}; "
          f"ID9 Δmotor={d_deg9:+.3f}° → {'OK' if ok9 else 'FAIL' if ok9 is not None else 'N/A'}")
    if AUTO_RESET:
        time.sleep(1.0); reset_to_trial_initial_both(ph, pk, enc)
        if post_t is not None:
            mode = "abd" if post_t else "add"
            # print(f"[POST] trial 回位後自動執行 t {mode.upper()}")
            # tighten_only(mode, ph, pk, enc)

# ================== CLI ==================
def run_cli():
    ph, pk = init_dxl()
    enc = EncoderStream(ENCODER_PORT, ENCODER_BAUD)
    load_calibration()

    if any(CAL.get(k) is None for k in ("TRIAL_INIT_M", "TRIAL_INIT_M9")):
        print("[INIT] 未設定 trial 初始位置，請先 'cal init'")
    else:
        if AUTO_RESET:
            time.sleep(1.0)
            reset_to_trial_initial_both(ph, pk, enc)

    print("指令：'cal abd' / 'cal add' / 'cal init' / 'reset' / 't abd' / 't add' / [d7,d9] 或 [d7,d9,flag] / 'q'")
    try:
        while True:
            cmd = input("cmd> ").strip().lower()
            if cmd == "q": break

            if cmd in ("cal abd", "cal add"):
                mode = "abd" if cmd.endswith("abd") else "add"
                calibrate_once(mode, ph, pk, enc); continue

            if cmd == "cal init":
                calibrate_trial_initial(ph, pk, enc); continue

            if cmd == "reset":
                reset_to_trial_initial_both(ph, pk, enc); continue

            if cmd in ("t abd", "t add"):
                mode = "abd" if cmd.endswith("abd") else "add"
                t0 = time.time()
                ok = tighten_only(mode, ph, pk, enc)
                print(f"[RESULT] tighten {mode.upper()} → {'OK' if ok else 'FAIL'} (elapsed {time.time()-t0:.2f}s)")
                if AUTO_RESET_AFTER_T_ABD_ADD:
                    time.sleep(1.0); reset_to_trial_initial_both(ph, pk, enc)
                continue

            # 解析 [d7,d9] 或 [d7,d9,flag]
            if cmd.startswith("[") and cmd.endswith("]"):
                try:
                    body = cmd[1:-1].strip()
                    parts = [p.strip() for p in body.split(",")]
                    if len(parts) not in (2,3):
                        raise ValueError("元素數量需為2或3")
                    delta7 = float(parts[0]); delta9 = float(parts[1])
                    post_t = None
                    if len(parts) == 3:
                        p3 = parts[2].lower()
                        if p3 in ("true","t","1","yes","y"): post_t = True
                        elif p3 in ("false","f","0","no","n"): post_t = False
                        else:
                            raise ValueError("第三個元素需為布林（true/false）")
                except Exception as e:
                    print(f"格式錯誤：{e}；請輸入例如：[-5,10] 或 [-5,90,true]")
                    continue
                execute_dual_mode(ph, pk, enc, delta7, delta9, post_t)
                continue

            print("請輸入：'cal abd' / 'cal add' / 'cal init' / 'reset' / 't abd' / 't add' / [d7,d9] 或 [d7,d9,flag] / 'q'")

    finally:
        try: set_torque(ph, pk, DXL_ID_MODE1, 0); set_torque(ph, pk, DXL_ID_MODE2, 0)
        except Exception: pass
        try: ph.closePort()
        except Exception: pass
        if ENC_INSTANCE is not None:ENC_INSTANCE.close()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--event", type=str, default=None, help="Unity 傳事件 (ANIM_DONE 等)")
    parser.add_argument("--bin", type=str, help="Unity 傳來的 6 bytes 二進位 (struct.pack 'hhh' 後的 hex string)")
    args = parser.parse_args()

    load_calibration()

    try:
        # 事件模式：若是 ANIM_DONE，就執行 reset → 輸出 DONE JSON
        if args.event == "ANIM_DONE":
            ph, pk, enc = init_dxl()
            reset_to_trial_initial_both(ph, pk, enc)
            _, enc7 = read_present_position(ph, pk, 7)
            _, mot9 = read_present_position(ph, pk, 9)
            print("DONE " + json.dumps({"id7_angle": enc7, "id9_angle": mot9}), flush=True)
            return

        # trial 指令模式（二進位輸入）
        if args.bin:
            try:
                # Unity 用 struct.pack("hhh", d7, d9, flag) 傳 → Python 這裡解
                data = bytes.fromhex(args.bin)  # 例如 "fbff5a0001" = [-5,90,1]
                ab_ad_angle, ex_angle, flag = struct.unpack("hhh", data)
                # 🔍 Debug print
                print(f"[DEBUG] 收到 HEX={args.bin}, unpack=({ab_ad_angle}, {ex_angle}, {flag})", flush=True)
            except Exception as e:
                print(f"[ERR] bin parse error: {e}", flush=True)
                return

            ph, pk, enc = init_dxl()

            # 若需要收線
            if ab_ad_angle != 0:
                mode = "abd" if ab_ad_angle < 0 else "add"
                compensate_only(mode, ph, pk, enc)
                print("READY", flush=True)  # 通知 Unity 可以開始動畫
            else:
                print("READY", flush=True)

            # 同時控制馬達
            post_t = True if flag == 1 else (False if flag == 0 else None)
            execute_dual_mode(ph, pk, enc, ab_ad_angle, ex_angle, post_t)

    finally:
        try: ph.closePort()
        except Exception: pass
        if ENC_INSTANCE is not None: ENC_INSTANCE.close()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        main()
    else:
        run_cli()
