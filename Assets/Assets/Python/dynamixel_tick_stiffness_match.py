#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Velocity-mode stiffness matching using encoder ticks as the virtual-force target.

Compared with the paxini-based match controller:
- F_target is NOT read from PaXini.
- F_target is computed from motor displacement:
    x_now_m <- present_tick - p0_tick
    F_target <- k * x_now_m   (with per-motor sign policy)
- F_meas still comes from HX711.

Usage:
    python dynamixel_tick_stiffness_match.py
    python dynamixel_tick_stiffness_match.py es
    python dynamixel_tick_stiffness_match.py em
    python dynamixel_tick_stiffness_match.py eh
"""

import math
import sys
import time
import threading
from collections import deque

from dynamixel_sdk import PortHandler, PacketHandler

from shared_state import SharedState
from hx711_reader import hx711_reader
from config import (
    HX_PORT,
    HX_BAUD,
    HX_TIMEOUT,
    DXL_BAUD,
    DXL_DEVICE,
    DXL_PROTOCOL,
    STIFFNESS_COMMAND_SPECS,
    STIFFNESS_PRESETS,
    STIFFNESS_MODE_IDS,
    STIFFNESS_SINGLE_MOTOR_ID,
    DXL_HX_INDEX,
    GROUP_B_IDS,
    GROUP_E_IDS,
    CONTROL_SIGN_BY_ID,
    STIFFNESS_SIGN_BY_ID,
    STIFFNESS_SPOOL_RADIUS_M,
    STIFFNESS_SPOOL_RADIUS_M_BY_GROUP,
    STIFFNESS_CURRENT_LIMIT_MA,
    STIFFNESS_FORCE_DEADBAND_N,
    STIFFNESS_E_GROUP_MIN_FM_FOR_TARGET_FLOOR_N,
    STIFFNESS_E_GROUP_TARGET_FLOOR_N,
    DT_LOOP,
)

try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_OK = True
except Exception:
    MATPLOTLIB_OK = False


ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_CURRENT_LIMIT = 38
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132

VELOCITY_MODE = 1
VEL_UNIT_RPM_PER_LSB = 0.229
DEG_PER_TICK = 0.088

DEFAULT_TRIAL_CODE = "es"
DEFAULT_VEL_MAX_MPS_BY_GROUP = {
    "b": 0.03,
    "e": 0.05,
}
DEFAULT_ERR_DEADBAND_N = STIFFNESS_FORCE_DEADBAND_N
DEFAULT_PLOT_SECONDS = 20.0


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def get_group_name_for_id(dxl_id):
    if dxl_id in GROUP_B_IDS:
        return "b"
    if dxl_id in GROUP_E_IDS:
        return "e"
    return "b"


def get_spool_radius_m_for_id(dxl_id):
    g = get_group_name_for_id(dxl_id)
    return float(STIFFNESS_SPOOL_RADIUS_M_BY_GROUP.get(g, STIFFNESS_SPOOL_RADIUS_M))


def get_tick_to_m_for_id(dxl_id):
    x_per_deg_m = 2.0 * math.pi * get_spool_radius_m_for_id(dxl_id) / 360.0
    return DEG_PER_TICK * x_per_deg_m


def rad_s_to_vel_lsb(omega_rad_s):
    rpm = omega_rad_s * 60.0 / (2.0 * math.pi)
    lsb = rpm / VEL_UNIT_RPM_PER_LSB
    return int(max(-2**31, min(2**31 - 1, round(lsb))))


def linear_mps_to_vel_lsb(dxl_id, v_mps):
    omega = v_mps / max(1e-12, get_spool_radius_m_for_id(dxl_id))
    vel_lsb = rad_s_to_vel_lsb(omega)
    vel_lsb *= int(CONTROL_SIGN_BY_ID.get(dxl_id, 1))
    return int(vel_lsb)


def setup_motor(pkt, port, dxl_id):
    pkt.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)
    pkt.write1ByteTxRx(port, dxl_id, ADDR_OPERATING_MODE, VELOCITY_MODE)
    pkt.write2ByteTxRx(port, dxl_id, ADDR_CURRENT_LIMIT, int(STIFFNESS_CURRENT_LIMIT_MA))
    pkt.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)


def set_goal_velocity(pkt, port, dxl_id, vel_lsb):
    pkt.write4ByteTxRx(port, dxl_id, ADDR_GOAL_VELOCITY, int(vel_lsb))


def read_present_position(pkt, port, dxl_id):
    pos, dxl_comm_result, dxl_error = pkt.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    if pos >= 2**31:
        pos -= 2**32
    if dxl_comm_result != 0 or dxl_error != 0:
        raise RuntimeError(f"read present pos failed for ID {dxl_id}")
    return int(pos)


def read_present_current(pkt, port, dxl_id):
    cur, dxl_comm_result, dxl_error = pkt.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_CURRENT)
    if cur >= 2**15:
        cur -= 2**16
    if dxl_comm_result != 0 or dxl_error != 0:
        return 0
    return int(cur)


def build_active_ids_for_trial(trial_code):
    spec = STIFFNESS_COMMAND_SPECS.get(trial_code)
    if not spec:
        raise KeyError(f"Unknown trial code: {trial_code}")

    ids = []
    if "b" in spec:
        ids.extend(GROUP_B_IDS)
    if "e" in spec:
        ids.extend(GROUP_E_IDS)

    ids = [int(i) for i in dict.fromkeys(ids) if int(i) in STIFFNESS_MODE_IDS]

    if int(STIFFNESS_SINGLE_MOTOR_ID):
        only_id = int(STIFFNESS_SINGLE_MOTOR_ID)
        ids = [i for i in ids if i == only_id]

    if not ids:
        raise RuntimeError("No active motor ids for this trial code")

    return ids


def build_pid_params_for_trial(trial_code, ids):
    spec = STIFFNESS_COMMAND_SPECS[trial_code]
    params = {}

    def fill_group(group_name, preset_name):
        preset = STIFFNESS_PRESETS[preset_name]
        group_ids = GROUP_B_IDS if group_name == "b" else GROUP_E_IDS
        for dxl_id in group_ids:
            if dxl_id not in ids:
                continue
            params[int(dxl_id)] = {
                "k": float(preset["k"]),
                "kp": float(preset["kp"]),
                "kd": float(preset["kd"]),
                "vel_max_mps": float(DEFAULT_VEL_MAX_MPS_BY_GROUP.get(group_name, 0.05)),
            }

    if "b" in spec:
        fill_group("b", spec["b"])
    if "e" in spec:
        fill_group("e", spec["e"])

    return params


def maybe_apply_e_group_target_floor(dxl_id, f_meas, f_target):
    if dxl_id in GROUP_E_IDS and f_meas < STIFFNESS_E_GROUP_MIN_FM_FOR_TARGET_FLOOR_N:
        return float(STIFFNESS_E_GROUP_TARGET_FLOOR_N)
    return float(f_target)


def main():
    trial_code = sys.argv[1].strip().lower() if len(sys.argv) > 1 else DEFAULT_TRIAL_CODE
    ids = build_active_ids_for_trial(trial_code)
    params_by_id = build_pid_params_for_trial(trial_code, ids)

    print(f"[TICK_MATCH] trial={trial_code} ids={ids}")

    state = SharedState()
    hx_thread = threading.Thread(
        target=hx711_reader,
        args=(state, HX_PORT, HX_BAUD, HX_TIMEOUT),
        daemon=True,
    )
    hx_thread.start()

    port = PortHandler(DXL_DEVICE)
    if not port.openPort():
        raise RuntimeError(f"Failed to open Dynamixel port: {DXL_DEVICE}")
    if not port.setBaudRate(DXL_BAUD):
        raise RuntimeError(f"Failed to set Dynamixel baud: {DXL_BAUD}")
    pkt = PacketHandler(DXL_PROTOCOL)

    for mid in ids:
        setup_motor(pkt, port, mid)

    p0 = {mid: read_present_position(pkt, port, mid) for mid in ids}
    prev_err = {mid: 0.0 for mid in ids}
    last_t = time.time()

    buf_t = deque(maxlen=5000)
    buf_ft = deque(maxlen=5000)
    buf_fm = deque(maxlen=5000)

    if MATPLOTLIB_OK:
        plt.ion()
        fig, ax = plt.subplots()
        line_ft, = ax.plot([], [], label="F_target")
        line_fm, = ax.plot([], [], label="F_meas")
        ax.set_xlabel("t (s)")
        ax.set_ylabel("Force (N)")
        ax.legend()
        ax.set_title(f"Tick stiffness match: {trial_code}")
    else:
        fig = ax = line_ft = line_fm = None

    t0 = time.time()

    try:
        while True:
            with state.lock:
                fm_vals = list(state.F_meas3)

            now = time.time()
            dt = max(1e-6, now - last_t)
            last_t = now

            ft_loop = []
            fm_loop = []

            for mid in ids:
                pos_now = read_present_position(pkt, port, mid)
                delta_tick = pos_now - p0[mid]
                x_sign = -1 if mid in GROUP_E_IDS else 1
                x_now_m = x_sign * delta_tick * get_tick_to_m_for_id(mid)

                p = params_by_id[mid]
                sgn = float(STIFFNESS_SIGN_BY_ID.get(mid, -1))
                f_target = (-sgn) * p["k"] * x_now_m

                h_idx = int(DXL_HX_INDEX.get(mid, 0))
                f_meas = float(fm_vals[h_idx]) if h_idx < len(fm_vals) else 0.0
                f_target = maybe_apply_e_group_target_floor(mid, f_meas, f_target)

                err = f_target - f_meas
                derr = (err - prev_err[mid]) / dt
                prev_err[mid] = err

                if abs(err) < DEFAULT_ERR_DEADBAND_N:
                    v_mps = 0.0
                else:
                    v_mps = -(p["kp"] * err + p["kd"] * derr)
                    v_mps = clamp(v_mps, -p["vel_max_mps"], p["vel_max_mps"])

                vel_lsb = linear_mps_to_vel_lsb(mid, v_mps)
                set_goal_velocity(pkt, port, mid, vel_lsb)

                ft_loop.append(f_target)
                fm_loop.append(f_meas)

                print(
                    f"[TICK_MATCH{mid}] "
                    f"tick={pos_now} p0={p0[mid]} x={x_now_m:+.6f}m | "
                    f"Ft={f_target:+.3f} Fm={f_meas:+.3f} err={err:+.3f} | "
                    f"v={v_mps:+.5f}m/s vel_lsb={vel_lsb:+d}"
                )

            if ft_loop:
                buf_t.append(time.time() - t0)
                buf_ft.append(sum(ft_loop) / len(ft_loop))
                buf_fm.append(sum(fm_loop) / len(fm_loop))

            if MATPLOTLIB_OK and buf_t:
                t_min = max(0.0, buf_t[-1] - DEFAULT_PLOT_SECONDS)
                tt = list(buf_t)
                ft = list(buf_ft)
                fm = list(buf_fm)
                start = 0
                for idx, t_val in enumerate(tt):
                    if t_val >= t_min:
                        start = idx
                        break
                tt = tt[start:]
                ft = ft[start:]
                fm = fm[start:]
                line_ft.set_data(tt, ft)
                line_fm.set_data(tt, fm)
                ax.relim()
                ax.autoscale_view()
                fig.canvas.draw()
                fig.canvas.flush_events()

            time.sleep(DT_LOOP)

    except KeyboardInterrupt:
        pass
    finally:
        state.stop = True
        try:
            hx_thread.join(timeout=0.5)
        except Exception:
            pass

        for mid in ids:
            try:
                set_goal_velocity(pkt, port, mid, 0)
                pkt.write1ByteTxRx(port, mid, ADDR_TORQUE_ENABLE, 0)
            except Exception:
                pass
        try:
            port.closePort()
        except Exception:
            pass


if __name__ == "__main__":
    main()
