
"""
Stiffness mode: virtual spring/damper in (extended) position mode, driven by HX711 forces.

Core idea (per your reference script):
- Read force F (N) per motor from HX711 serial (SharedState.F_meas3, via hx711_reader thread)
- Convert force to displacement via x = -F / K  (spring), and optionally a damper term
- Convert displacement (meters) -> delta degrees -> delta ticks
- Goal position = p0 + delta_ticks
- HARD RULE: goal never exceeds initial p0 on one side (config: STIFFNESS_GOAL_MUST_BE_LEQ_P0)

This controller is non-blocking: call step() periodically.
"""

import time, math
from dynamixel_sdk import PortHandler, PacketHandler

from shared_state import SharedState
from motor_calibration import load_limits, MotorLimit
from config import (
    DXL_BAUD, DXL_DEVICE, DXL_PROTOCOL,
    STIFFNESS_MODE_IDS, STIFFNESS_HX_INDEX,
    STIFFNESS_K_N_PER_M, STIFFNESS_B_N_S_PER_M, STIFFNESS_ENABLE_DAMPER,
    STIFFNESS_FORCE_DEADBAND_N, STIFFNESS_FORCE_LPF_ALPHA,
    STIFFNESS_DAMP_LEAK_PER_S, STIFFNESS_DAMP_CLAMP_DEG,
    STIFFNESS_SPOOL_RADIUS_M,
    STIFFNESS_MAX_DEFLECT_DEG, STIFFNESS_CURRENT_LIMIT_MA,
    STIFFNESS_SIGN_BY_ID, STIFFNESS_GOAL_MUST_BE_LEQ_P0,
    STIFFNESS_PROFILE_ACCEL, STIFFNESS_PROFILE_VEL,
    MOTOR_LIMITS_FILE, ENFORCE_SOFT_LIMITS, SOFT_LIMIT_MARGIN_TICKS,
    DT_LOOP,
)

# ===== Dynamixel addresses (X series) =====
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_CURRENT_LIMIT    = 38
ADDR_PROFILE_ACCEL    = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132

# Dynamixel ticks <-> deg (XL330 typical)
DEG_PER_TICK = 0.088
TICKS_PER_DEG = 1.0 / DEG_PER_TICK

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def _setup_motor(pkt: PacketHandler, port: PortHandler, dxl_id: int):
    pkt.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)
    pkt.write1ByteTxRx(port, dxl_id, ADDR_OPERATING_MODE, 3)  # Position mode
    pkt.write2ByteTxRx(port, dxl_id, ADDR_CURRENT_LIMIT, int(STIFFNESS_CURRENT_LIMIT_MA))
    pkt.write4ByteTxRx(port, dxl_id, ADDR_PROFILE_ACCEL, int(STIFFNESS_PROFILE_ACCEL))
    pkt.write4ByteTxRx(port, dxl_id, ADDR_PROFILE_VELOCITY, int(STIFFNESS_PROFILE_VEL))
    pkt.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)

def _read_present_position(pkt: PacketHandler, port: PortHandler, dxl_id: int) -> int:
    pos, dxl_comm_result, dxl_error = pkt.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    if pos >= 2**31:
        pos -= 2**32
    if dxl_comm_result != 0 or dxl_error != 0:
        return 0
    return int(pos)

def _write_goal_position(pkt: PacketHandler, port: PortHandler, dxl_id: int, goal_tick: int):
    pkt.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(goal_tick))

def _enforce_p0_rule(goal: int, p0: int, must_be_leq: bool) -> int:
    return min(goal, p0) if must_be_leq else max(goal, p0)

def _apply_soft_limits(goal: int, lim: MotorLimit, margin: int) -> int:
    lo = int(lim.min_tick) + int(margin)
    hi = int(lim.max_tick) - int(margin)
    return int(clamp(goal, lo, hi))

class StiffnessController:
    """Non-blocking stiffness controller (call step() periodically)."""

    def __init__(self, state: SharedState):
        self.state = state
        self.port = PortHandler(DXL_DEVICE)
        if not self.port.openPort():
            raise RuntimeError(f"Failed to open Dynamixel port: {DXL_DEVICE}")
        if not self.port.setBaudRate(DXL_BAUD):
            raise RuntimeError(f"Failed to set Dynamixel baud: {DXL_BAUD}")
        self.pkt = PacketHandler(DXL_PROTOCOL)

        self.limits = load_limits(MOTOR_LIMITS_FILE)

        self.ids = list(STIFFNESS_MODE_IDS)
        if not self.ids:
            raise RuntimeError("STIFFNESS_MODE_IDS is empty in config.py")

        for mid in self.ids:
            _setup_motor(self.pkt, self.port, mid)

        # Record equilibrium (initial position) per motor
        self.p0 = {mid: _read_present_position(self.pkt, self.port, mid) for mid in self.ids}

        # One-time init of runtime tunables (persist across mode switches)
        with self.state.lock:
            if not self.state.stiffness_params_initialized:
                self.state.stiffness_enable_damper = bool(STIFFNESS_ENABLE_DAMPER)
                self.state.stiffness_k_n_per_m = float(STIFFNESS_K_N_PER_M)
                self.state.stiffness_b_n_s_per_m = float(STIFFNESS_B_N_S_PER_M)
                self.state.stiffness_goal_must_be_leq_p0 = bool(STIFFNESS_GOAL_MUST_BE_LEQ_P0)
                self.state.stiffness_params_initialized = True

        # Force filters / damper state
        self.F_filt = {mid: 0.0 for mid in self.ids}
        self.x_damp_m = {mid: 0.0 for mid in self.ids}

        # geometry
        self.x_per_deg_m = 2.0 * math.pi * float(STIFFNESS_SPOOL_RADIUS_M) / 360.0

        self.last_t = time.time()

        print("\n[STIFF] Position Mode virtual spring")
        print(f"[STIFF] IDs={self.ids} p0={self.p0}")
        print("[STIFF] Key switch: p=POS, m=FREE, k=STIFF (no-op), q=quit")
        print("[STIFF] Commands: help, r(reset p0), damper0/1, k<val>, b<val>, rule0/1")

    def handle_line(self, line: str):
        s = line.strip().lower().replace(" ", "")
        if not s:
            return None
        if s in ("q", "quit", "exit"):
            return "quit"
        if s in ("h", "help", "?"):
            with self.state.lock:
                print(f"[STIFF] damper={self.state.stiffness_enable_damper} k={self.state.stiffness_k_n_per_m} b={self.state.stiffness_b_n_s_per_m} rule_leq_p0={self.state.stiffness_goal_must_be_leq_p0}")
            return None
        if s == "r":
            # reset equilibrium to current position
            self.p0 = {mid: _read_present_position(self.pkt, self.port, mid) for mid in self.ids}
            print(f"[STIFF] p0 reset: {self.p0}")
            return None

        def _parse(prefix: str):
            try:
                return float(s[len(prefix):])
            except Exception:
                return None

        if s.startswith("damper"):
            v = _parse("damper")
            if v is None:
                print("[STIFF] Bad damper command. Use damper0 or damper1")
                return None
            with self.state.lock:
                self.state.stiffness_enable_damper = bool(int(v))
            print(f"[STIFF] damper -> {bool(int(v))}")
            return None

        if s.startswith("k") and len(s) > 1:
            v = _parse("k")
            if v is None or v <= 0:
                print("[STIFF] Bad k command. Example: k400")
                return None
            with self.state.lock:
                self.state.stiffness_k_n_per_m = float(v)
            print(f"[STIFF] k -> {v}")
            return None

        if s.startswith("b") and len(s) > 1:
            v = _parse("b")
            if v is None or v <= 0:
                print("[STIFF] Bad b command. Example: b80")
                return None
            with self.state.lock:
                self.state.stiffness_b_n_s_per_m = float(v)
            print(f"[STIFF] b -> {v}")
            return None

        if s.startswith("rule"):
            v = _parse("rule")
            if v is None:
                print("[STIFF] Bad rule command. Use rule1 (goal<=p0) or rule0 (goal>=p0)")
                return None
            with self.state.lock:
                self.state.stiffness_goal_must_be_leq_p0 = bool(int(v))
            print(f"[STIFF] rule_leq_p0 -> {bool(int(v))}")
            return None

        print("[STIFF] Unknown command. Try: help")
        return None

    def step(self):
        if self.state.stop:
            return

        # Get HX values snapshot
        with self.state.lock:
            hx_vals = list(self.state.F_meas3)
            damper_on = bool(self.state.stiffness_enable_damper)
            k = max(1e-9, float(self.state.stiffness_k_n_per_m))
            b = max(1e-9, float(self.state.stiffness_b_n_s_per_m))
            must_leq = bool(self.state.stiffness_goal_must_be_leq_p0)

        # timing
        t_now = time.time()
        dt = max(1e-6, t_now - self.last_t)
        self.last_t = t_now

        for mid in self.ids:
            h_idx = STIFFNESS_HX_INDEX.get(mid, None)
            F_raw = float(hx_vals[h_idx]) if (h_idx is not None and h_idx < len(hx_vals)) else 0.0

            # deadband
            if abs(F_raw) < float(STIFFNESS_FORCE_DEADBAND_N):
                F_raw = 0.0

            # LPF
            self.F_filt[mid] = (1.0 - float(STIFFNESS_FORCE_LPF_ALPHA)) * self.F_filt[mid] + float(STIFFNESS_FORCE_LPF_ALPHA) * F_raw

            sgn = float(STIFFNESS_SIGN_BY_ID.get(mid, +1))

            # Spring: x = -F/k
            x_spring_m = sgn * (-self.F_filt[mid] / k)

            # Damper: integrate xdot = -F/b (same as your reference), with leak+clamp
            if damper_on:
                xdot = sgn * (-self.F_filt[mid] / b)
                self.x_damp_m[mid] += xdot * dt

                if abs(self.F_filt[mid]) < float(STIFFNESS_FORCE_DEADBAND_N):
                    self.x_damp_m[mid] *= math.exp(-float(STIFFNESS_DAMP_LEAK_PER_S) * dt)

                x_damp_deg = self.x_damp_m[mid] / max(1e-12, self.x_per_deg_m)
                x_damp_deg = clamp(x_damp_deg, -float(STIFFNESS_DAMP_CLAMP_DEG), float(STIFFNESS_DAMP_CLAMP_DEG))
                self.x_damp_m[mid] = x_damp_deg * self.x_per_deg_m
            else:
                self.x_damp_m[mid] = 0.0

            x_total_m = x_spring_m + self.x_damp_m[mid]

            delta_deg = x_total_m / max(1e-12, self.x_per_deg_m)
            delta_deg = clamp(delta_deg, -float(STIFFNESS_MAX_DEFLECT_DEG), float(STIFFNESS_MAX_DEFLECT_DEG))

            goal = int(self.p0[mid] + round(delta_deg * TICKS_PER_DEG))

            # HARD RULE: never exceed initial p0 on the specified side
            goal = _enforce_p0_rule(goal, int(self.p0[mid]), must_leq)

            # optional: soft limits
            if ENFORCE_SOFT_LIMITS and mid in self.limits:
                goal = _apply_soft_limits(goal, self.limits[mid], int(SOFT_LIMIT_MARGIN_TICKS))

            _write_goal_position(self.pkt, self.port, mid, goal)

    def stop_motors(self):
        # Move back to equilibrium p0 (optional) then hold position
        for mid in self.ids:
            try:
                _write_goal_position(self.pkt, self.port, mid, int(self.p0[mid]))
            except Exception:
                pass

    def close(self):
        for mid in self.ids:
            try:
                _write_goal_position(self.pkt, self.port, mid, int(self.p0[mid]))
                self.pkt.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, 0)
            except Exception:
                pass
        try:
            self.port.closePort()
        except Exception:
            pass


def dynamixel_stiffness_control(state: SharedState):
    """Legacy blocking entrypoint for stiffness mode."""
    ctrl = StiffnessController(state)
    try:
        while not state.stop:
            ctrl.step()
            time.sleep(float(DT_LOOP))
    finally:
        ctrl.close()
