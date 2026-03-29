"""
Free mode: keep each motor's HX711 value near a target (default 0 N).

- Reads HX values from SharedState (updated by hx711_reader thread).
- Runs a simple "error -> velocity" loop per motor, similar to dynamixel_match_control,
  but the target is FREE_TARGET_N and measured force comes from FREE_HX_INDEX mapping.
"""

import time, math
from dynamixel_sdk import PortHandler, PacketHandler

from shared_state import SharedState
from motor_calibration import load_limits, MotorLimit
from config import (
    DXL_BAUD, DXL_DEVICE, DXL_PROTOCOL,
    FREE_MODE_IDS, FREE_HX_INDEX,
    FREE_TARGET_N, FREE_K_POS, FREE_K_NEG, FREE_KD, FREE_V_MAX_MPS, FREE_ERR_DEADBAND,
    R_EFF_M, VEL_UNIT_RPM_PER_LSB, DIRECTION_SIGN,
    CURRENT_LIMIT_MA, DT_LOOP,
    MOTOR_LIMITS_FILE, ENFORCE_SOFT_LIMITS, SOFT_LIMIT_MARGIN_TICKS,
    PRESENT_CURRENT_BACKOFF_MA, BACKOFF_VEL_LSB, BACKOFF_DURATION_S,
)

# ===== Dynamixel addresses (X series) =====
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_CURRENT_LIMIT    = 38
ADDR_GOAL_VELOCITY    = 104
ADDR_PRESENT_CURRENT  = 126
ADDR_PRESENT_POSITION = 132

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def rad_s_to_vel_lsb(omega_rad_s: float) -> int:
    rpm = omega_rad_s * 60.0 / (2.0 * math.pi)
    lsb = rpm / VEL_UNIT_RPM_PER_LSB
    return int(max(-2**31, min(2**31-1, round(lsb))))

def _setup_motor(pkt: PacketHandler, port: PortHandler, dxl_id: int):
    pkt.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)
    pkt.write1ByteTxRx(port, dxl_id, ADDR_OPERATING_MODE, 1)  # velocity mode
    pkt.write2ByteTxRx(port, dxl_id, ADDR_CURRENT_LIMIT, int(CURRENT_LIMIT_MA))
    pkt.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)

def _set_goal_velocity(pkt: PacketHandler, port: PortHandler, dxl_id: int, vel_lsb: int):
    pkt.write4ByteTxRx(port, dxl_id, ADDR_GOAL_VELOCITY, int(vel_lsb))

def _read_present_position(pkt: PacketHandler, port: PortHandler, dxl_id: int) -> int:
    pos, dxl_comm_result, dxl_error = pkt.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    if pos >= 2**31:
        pos -= 2**32
    if dxl_comm_result != 0 or dxl_error != 0:
        return 0
    return int(pos)

def _read_present_current(pkt: PacketHandler, port: PortHandler, dxl_id: int) -> int:
    cur, dxl_comm_result, dxl_error = pkt.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_CURRENT)
    if cur >= 2**15:
        cur -= 2**16
    if dxl_comm_result != 0 or dxl_error != 0:
        return 0
    return int(cur)

def _apply_soft_limit(vel_lsb: int, pos_tick: int, lim: MotorLimit, margin: int) -> int:
    if vel_lsb < 0 and pos_tick <= (lim.min_tick + margin):
        return 0
    if vel_lsb > 0 and pos_tick >= (lim.max_tick - margin):
        return 0
    return vel_lsb

class FreeForceController:
    """Non-blocking FREE mode controller (call step() periodically)."""

    def __init__(self, state: SharedState):
        self.state = state
        self.port = PortHandler(DXL_DEVICE)
        if not self.port.openPort():
            raise RuntimeError(f"Failed to open Dynamixel port: {DXL_DEVICE}")
        if not self.port.setBaudRate(DXL_BAUD):
            raise RuntimeError(f"Failed to set Dynamixel baud: {DXL_BAUD}")
        self.pkt = PacketHandler(DXL_PROTOCOL)

        self.limits = load_limits(MOTOR_LIMITS_FILE)

        self.ids = list(FREE_MODE_IDS)
        if not self.ids:
            raise RuntimeError("FREE_MODE_IDS is empty in config.py")

        for mid in self.ids:
            _setup_motor(self.pkt, self.port, mid)

        self.backoff_until = {mid: 0.0 for mid in self.ids}
        self.prev_err = {mid: 0.0 for mid in self.ids}
        self.last_step_t = time.time()

        print("\n[FREE] Velocity Mode force-hold")
        print("[FREE] Target: FREE_TARGET_N (default 0 N). Mode switch: 'p' -> POS, 'm' -> FREE (no-op), 'q' -> quit")
        # Initialize runtime tunables once (so they persist across mode switches)
        with self.state.lock:
            if not self.state.free_params_initialized:
                self.state.free_target_n = float(FREE_TARGET_N)
                self.state.free_kp_pos = float(FREE_K_POS)
                self.state.free_kp_neg = float(FREE_K_NEG)
                self.state.free_kd = float(FREE_KD)
                self.state.free_params_initialized = True

        print("[FREE] Commands: t<number> set target N (e.g., t0.5). kp<number> set Kp (e.g., kp1.2). kd<number> set Kd (e.g., kd0.3).")

    def step(self):
        if self.state.stop:
            return
        with self.state.lock:
            hx_vals = list(self.state.F_meas3)

        for mid in self.ids:
            now = time.time()

            # safety backoff on over-current
            cur_ma = _read_present_current(self.pkt, self.port, mid)
            if cur_ma > PRESENT_CURRENT_BACKOFF_MA and now >= self.backoff_until[mid]:
                self.backoff_until[mid] = now + float(BACKOFF_DURATION_S)

            if now < self.backoff_until[mid]:
                _set_goal_velocity(self.pkt, self.port, mid, int(BACKOFF_VEL_LSB))
                continue
            elif self.backoff_until[mid] != 0.0 and now >= self.backoff_until[mid]:
                self.backoff_until[mid] = 0.0
                _set_goal_velocity(self.pkt, self.port, mid, 0)
                continue

            h_idx = FREE_HX_INDEX.get(mid, None)
            F_meas = float(hx_vals[h_idx]) if (h_idx is not None and h_idx < len(hx_vals)) else 0.0

            with self.state.lock:
                target = float(self.state.free_target_n)
                kp_pos = float(self.state.free_kp_pos)
                kp_neg = float(self.state.free_kp_neg)
                kd = float(self.state.free_kd)

            err = target - F_meas

            # dt for derivative term
            now_t = time.time()
            dt = max(1e-6, now_t - self.last_step_t)

            if abs(err) < float(FREE_ERR_DEADBAND):
                v_mps = 0.0
                self.prev_err[mid] = err
            else:
                k = kp_pos if err >= 0 else kp_neg
                derr = (err - self.prev_err[mid]) / dt
                self.prev_err[mid] = err
                v_cmd = (k * err) + (kd * derr)
                v_mps = clamp(v_cmd, -float(FREE_V_MAX_MPS), float(FREE_V_MAX_MPS))

            omega = (v_mps / float(R_EFF_M)) * float(DIRECTION_SIGN)
            vel_lsb = rad_s_to_vel_lsb(omega)

            if ENFORCE_SOFT_LIMITS and mid in self.limits:
                pos_tick = _read_present_position(self.pkt, self.port, mid)
                vel_lsb = _apply_soft_limit(vel_lsb, pos_tick, self.limits[mid], int(SOFT_LIMIT_MARGIN_TICKS))

            _set_goal_velocity(self.pkt, self.port, mid, vel_lsb)

        self.last_step_t = time.time()

    def handle_line(self, line: str):
        """Handle interactive commands while in FREE mode."""
        s = line.strip().lower().replace(' ', '')
        if not s:
            return None
        if s in ('q','quit','exit'):
            return 'quit'
        if s in ('h','help','?'):
            with self.state.lock:
                print(f"[FREE] target={self.state.free_target_n}N kp_pos={self.state.free_kp_pos} kp_neg={self.state.free_kp_neg} kd={self.state.free_kd}")
                print("[FREE] Commands: t<number>, kp<number>, kd<number>, (optional) kp+<number>/kp-<number> to set pos/neg separately.")
            return None

        def _parse_num(prefix: str):
            try:
                return float(s[len(prefix):])
            except Exception:
                return None

        # target
        if s.startswith('t') and len(s) > 1:
            val = _parse_num('t')
            if val is None:
                print('[FREE] Bad t command. Example: t0.5')
                return None
            with self.state.lock:
                self.state.free_target_n = float(val)
            print(f"[FREE] target -> {val} N")
            return None

        # kp (both)
        if s.startswith('kp') and len(s) > 2 and not (s.startswith('kp+') or s.startswith('kp-')):
            val = _parse_num('kp')
            if val is None:
                print('[FREE] Bad kp command. Example: kp1.2')
                return None
            with self.state.lock:
                self.state.free_kp_pos = float(val)
                self.state.free_kp_neg = float(val)
            print(f"[FREE] kp(pos/neg) -> {val}")
            return None

        # kp+ / kp-
        if s.startswith('kp+') and len(s) > 3:
            val = _parse_num('kp+')
            if val is None:
                print('[FREE] Bad kp+ command. Example: kp+0.8')
                return None
            with self.state.lock:
                self.state.free_kp_pos = float(val)
            print(f"[FREE] kp_pos -> {val}")
            return None
        if s.startswith('kp-') and len(s) > 3:
            val = _parse_num('kp-')
            if val is None:
                print('[FREE] Bad kp- command. Example: kp-0.8')
                return None
            with self.state.lock:
                self.state.free_kp_neg = float(val)
            print(f"[FREE] kp_neg -> {val}")
            return None

        # kd
        if s.startswith('kd') and len(s) > 2:
            val = _parse_num('kd')
            if val is None:
                print('[FREE] Bad kd command. Example: kd0.3')
                return None
            with self.state.lock:
                self.state.free_kd = float(val)
            print(f"[FREE] kd -> {val}")
            return None

        print('[FREE] Unknown command. Try: help')
        return None

    def stop_motors(self):
        for mid in self.ids:
            try:
                _set_goal_velocity(self.pkt, self.port, mid, 0)
            except Exception:
                pass

    def close(self):
        for mid in self.ids:
            try:
                _set_goal_velocity(self.pkt, self.port, mid, 0)
                self.pkt.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, 0)
            except Exception:
                pass
        try:
            self.port.closePort()
        except Exception:
            pass


def dynamixel_free_force_control(state: SharedState):
    """Legacy blocking version (kept for compatibility)."""
    ctrl = FreeForceController(state)
    try:
        while not state.stop:
            ctrl.step()
            time.sleep(float(DT_LOOP))
    finally:
        ctrl.close()
