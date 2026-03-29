"""
Dynamixel match control: per-motor loop using:
- PaXini target force (F_target) per motor
- HX711 measured force (F_meas) per motor

Update (2026-01):
- Each motor has its own measured force from a single HX711 serial port line "f1,f2,f3".
- Each motor has its own PaXini sensor stream (one port per PaXini).
- Mapping is controlled via config:
    ENABLED_IDS, DXL_PAXINI_INDEX, DXL_HX_INDEX
"""

import time, math
from collections import deque
import matplotlib.pyplot as plt
from dynamixel_sdk import PortHandler, PacketHandler

from shared_state import SharedState
from motor_calibration import load_limits, calibrate_limits_interactive, MotorLimit
from config import (
    DXL_BAUD, DXL_DEVICE, DXL_PROTOCOL,
    ENABLED_IDS, DXL_HX_INDEX,
    R_EFF_M, VEL_UNIT_RPM_PER_LSB,
    DIRECTION_SIGN,
    K_POS, K_NEG, V_MAX_MPS, CURRENT_LIMIT_MA, DT_LOOP, ERR_DEADBAND,
    TIME_WINDOW_S, PLOT_UPDATE_EVERY, MAX_POINTS_BUFFER, COLOR_FT, COLOR_FM,
    MOTOR_LIMITS_FILE, CALIBRATE_LIMITS_ON_START, ENFORCE_SOFT_LIMITS, SOFT_LIMIT_MARGIN_TICKS,
    PRESENT_CURRENT_BACKOFF_MA, BACKOFF_VEL_LSB, BACKOFF_DURATION_S,
)

# ===== Dynamixel addresses (X series) =====
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_CURRENT_LIMIT    = 38
ADDR_GOAL_VELOCITY    = 104
ADDR_HW_ERR           = 70
ADDR_PRESENT_CURRENT  = 126
ADDR_PRESENT_POSITION = 132

F_FIXED = 2

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def rad_s_to_vel_lsb(omega_rad_s: float) -> int:
    rpm = omega_rad_s * 60.0 / (2.0 * math.pi)
    lsb = rpm / VEL_UNIT_RPM_PER_LSB
    return int(max(-2**31, min(2**31-1, round(lsb))))

def _setup_motor(pkt: PacketHandler, port: PortHandler, dxl_id: int):
    # Velocity mode + current limit
    pkt.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)
    pkt.write1ByteTxRx(port, dxl_id, ADDR_OPERATING_MODE, 1)  # velocity mode
    pkt.write2ByteTxRx(port, dxl_id, ADDR_CURRENT_LIMIT, int(CURRENT_LIMIT_MA))
    pkt.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)

def _set_goal_velocity(pkt: PacketHandler, port: PortHandler, dxl_id: int, vel_lsb: int):
    # vel is 4 bytes (signed)
    pkt.write4ByteTxRx(port, dxl_id, ADDR_GOAL_VELOCITY, int(vel_lsb))

def _read_present_position(pkt: PacketHandler, port: PortHandler, dxl_id: int) -> int:
    pos, dxl_comm_result, dxl_error = pkt.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    if pos >= 2**31:
        pos -= 2**32
    if dxl_comm_result != 0 or dxl_error != 0:
        # don't hard-crash the control loop: return 0 and let soft-limit skip
        return 0
    return int(pos)

def _read_present_current(pkt: PacketHandler, port: PortHandler, dxl_id: int) -> int:
    cur, dxl_comm_result, dxl_error = pkt.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_CURRENT)
    # signed 16-bit
    if cur >= 2**15:
        cur -= 2**16
    if dxl_comm_result != 0 or dxl_error != 0:
        return 0
    return int(cur)

def _apply_soft_limit(vel_lsb: int, pos_tick: int, lim: MotorLimit, margin: int) -> int:
    """Prevent velocity from driving further outside [min,max]."""
    if vel_lsb < 0 and pos_tick <= (lim.min_tick + margin):
        return 0
    if vel_lsb > 0 and pos_tick >= (lim.max_tick - margin):
        return 0
    return vel_lsb

def dynamixel_match_control(state: SharedState):
    # -------------------------
    # Setup port + packet handler
    # -------------------------
    port = PortHandler(DXL_DEVICE)
    if not port.openPort():
        raise RuntimeError(f"Failed to open Dynamixel port: {DXL_DEVICE}")
    if not port.setBaudRate(DXL_BAUD):
        raise RuntimeError(f"Failed to set Dynamixel baud: {DXL_BAUD}")
    pkt = PacketHandler(DXL_PROTOCOL)

    # -------------------------
    # Load / calibrate soft-limits
    # -------------------------
    limits = load_limits(MOTOR_LIMITS_FILE)
    if CALIBRATE_LIMITS_ON_START:
        # Calibrate *enabled* motors only
        limits = calibrate_limits_interactive(pkt, port, ENABLED_IDS, MOTOR_LIMITS_FILE)

    # Setup motors
    for mid in ENABLED_IDS:
        _setup_motor(pkt, port, mid)

    # Backoff timers per motor (epoch seconds); if now < backoff_until[mid], we are in forced backoff.
    backoff_until = {mid: 0.0 for mid in ENABLED_IDS}

    # -------------------------
    # Plot buffers (single plot: aggregate)
    # -------------------------
    t0 = time.time()
    buf_t = deque(maxlen=MAX_POINTS_BUFFER)
    buf_ft = deque(maxlen=MAX_POINTS_BUFFER)
    buf_fm = deque(maxlen=MAX_POINTS_BUFFER)

    plt.ion()
    fig, ax = plt.subplots()
    line_ft, = ax.plot([], [], label="F_target (avg)")
    line_fm, = ax.plot([], [], label="F_meas (avg)")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("Force")
    ax.legend()
    ax.set_title("Dynamixel match-control (avg over enabled motors)")

    n_iter = 0
    try:
        while not state.stop:
            # Snapshot shared state
            with state.lock:
                Fz3 = list(state.Fz3)
                Fm3 = list(state.F_meas3)

            # Per-motor control
            ft_vals = []
            fm_vals = []
            for mid in ENABLED_IDS:
                now = time.time()

                # (1) Over-current safety backoff
                cur_ma = _read_present_current(pkt, port, mid)
                if cur_ma > PRESENT_CURRENT_BACKOFF_MA and now >= backoff_until[mid]:
                    backoff_until[mid] = now + float(BACKOFF_DURATION_S)

                if now < backoff_until[mid]:
                    _set_goal_velocity(pkt, port, mid, int(BACKOFF_VEL_LSB))
                    continue
                elif backoff_until[mid] != 0.0 and now >= backoff_until[mid]:
                    # one-shot stop after backoff window
                    backoff_until[mid] = 0.0
                    _set_goal_velocity(pkt, port, mid, 0)
                    continue

                h_idx = DXL_HX_INDEX.get(mid, 0)

                F_target = F_FIXED
                if F_target is None:
                    # no target yet -> hold still
                    _set_goal_velocity(pkt, port, mid, 0)
                    continue

                F_meas = float(Fm3[h_idx]) if h_idx < len(Fm3) else 0.0

                err = F_target - F_meas
                ft_vals.append(F_target)
                fm_vals.append(F_meas)

                if abs(err) < ERR_DEADBAND:
                    v_mps = 0.0
                else:
                    k = K_POS if err >= 0 else K_NEG
                    v_mps = clamp(k * err, -V_MAX_MPS, V_MAX_MPS)

                omega = (v_mps / R_EFF_M) * float(DIRECTION_SIGN)
                vel_lsb = rad_s_to_vel_lsb(omega)

                # (2) Position soft-limits in velocity mode
                if ENFORCE_SOFT_LIMITS and mid in limits:
                    pos_tick = _read_present_position(pkt, port, mid)
                    vel_lsb = _apply_soft_limit(vel_lsb, pos_tick, limits[mid], int(SOFT_LIMIT_MARGIN_TICKS))

                _set_goal_velocity(pkt, port, mid, vel_lsb)

            # Plot (avg)
            if ft_vals:
                ft_avg = sum(ft_vals) / len(ft_vals)
                fm_avg = sum(fm_vals) / len(fm_vals) if fm_vals else 0.0
                t = time.time() - t0
                buf_t.append(t); buf_ft.append(ft_avg); buf_fm.append(fm_avg)

            n_iter += 1
            if n_iter % PLOT_UPDATE_EVERY == 0 and buf_t:
                # window
                t_min = max(0.0, buf_t[-1] - TIME_WINDOW_S)
                # convert to lists for slicing
                tt = list(buf_t); ft = list(buf_ft); fm = list(buf_fm)
                # select indices within window
                start = 0
                for i in range(len(tt)):
                    if tt[i] >= t_min:
                        start = i
                        break
                tt = tt[start:]; ft = ft[start:]; fm = fm[start:]

                line_ft.set_data(tt, ft)
                line_fm.set_data(tt, fm)
                ax.relim(); ax.autoscale_view()
                fig.canvas.draw(); fig.canvas.flush_events()

            time.sleep(DT_LOOP)

    finally:
        # Stop motors
        for mid in ENABLED_IDS:
            try:
                _set_goal_velocity(pkt, port, mid, 0)
                pkt.write1ByteTxRx(port, mid, ADDR_TORQUE_ENABLE, 0)
            except Exception:
                pass
        try:
            port.closePort()
        except Exception:
            pass
