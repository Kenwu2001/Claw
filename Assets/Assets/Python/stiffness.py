#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import json
import time
import math
import threading
from collections import deque

import serial
from dynamixel_sdk import *

from config import (
    HX_PORT,
    HX_BAUD,
    HX_TIMEOUT,
    HX_MAX_VALUES,
    HX_ZERO_SAMPLES,
    HX_ZERO_SAMPLE_INTERVAL_S,
    HX_ZERO_TIMEOUT_S,
    DXL_BAUD,
    DXL_DEVICE,
    DXL_PROTOCOL,
    STIFFNESS_MODE_IDS,
    DXL_HX_INDEX,
    GROUP_B_IDS,
    GROUP_E_IDS,
    INTI_DEG_STIFF,
    STIFFNESS_FORCE_DEADBAND_N,
    STIFFNESS_FORCE_LPF_ALPHA,
    STIFFNESS_E_GROUP_MIN_FM_FOR_TARGET_FLOOR_N,
    STIFFNESS_E_GROUP_TARGET_FLOOR_N,
    STIFFNESS_SPOOL_RADIUS_M,
    STIFFNESS_SPOOL_RADIUS_M_BY_GROUP,
    STIFFNESS_CURRENT_LIMIT_MA,
    STIFFNESS_SIGN_BY_ID,
    CONTROL_SIGN_BY_ID,
    DT_LOOP,
    STIFFNESS_PRESETS,
    STIFFNESS_COMMAND_SPECS,
    STIFFNESS_DX_CMD_MAX_M_BY_GROUP,
    STIFFNESS_DX_CMD_PUSH_MAX_M_BY_GROUP,
    STIFFNESS_FALLING_PUSH_ERR_SCALE_MIN_BY_GROUP,
    STIFFNESS_FALLING_PUSH_MAX_BOOST_BY_GROUP,
    STIFFNESS_DX_CMD_RELEASE_MAX_M_BY_GROUP,
    STIFFNESS_RISING_RELEASE_SLOWDOWN_BY_GROUP,
    STIFFNESS_RISING_TARGET_POSITIVE_ERROR_CAP_N_BY_GROUP,
    STIFFNESS_SPEED_ERR_FULL_SCALE_N_BY_GROUP,
    STIFFNESS_FORCE_ERROR_DEADBAND_N_BY_GROUP,
    STIFFNESS_MATCH_ENTER_BASE_N_BY_GROUP,
    STIFFNESS_MATCH_ENTER_MIN_N_BY_GROUP,
    STIFFNESS_MATCH_ENTER_FT_SCALE_N_BY_GROUP,
    STIFFNESS_MATCH_USE_DYNAMIC_THRESHOLD,
    STIFFNESS_MATCH_EXIT_BASE_N_BY_GROUP,
    STIFFNESS_MATCH_EXIT_MIN_N_BY_GROUP,
    STIFFNESS_MATCH_EXIT_FT_SCALE_N_BY_GROUP,
    ERROR_STATE_MATCH_ENTER_ABS_N_BY_GROUP,
    ERROR_STATE_MATCH_EXIT_ABS_N_BY_GROUP,
    ERROR_STATE_MATCH_ENTER_DMAG_MAX_N_BY_GROUP,
    ERROR_STATE_MATCH_EXIT_DMAG_MIN_N_BY_GROUP,
    ERROR_STATE_MATCH_ENTER_CONFIRM,
    ERROR_STATE_MATCH_EXIT_CONFIRM,
    ERROR_STATE_MATCH_HOLD_SECONDS,
    ERROR_STATE_CONFIG_PRESET,
    ERROR_STATE_PRESETS,
    ENABLE_GAIN_SCHEDULING,
    GAIN_SCHEDULE_THRESH_N_BY_GROUP,
    GAIN_SCHEDULE_SCALE_BY_GROUP,
    STIFFNESS_PID_OUTPUT_MODE,
    STIFFNESS_PID_VEL_LPF_ALPHA,
    STIFFNESS_PID_VEL_MAX_MPS_BY_GROUP,
    STIFFNESS_E_GROUP_TARGET_FLOOR_VEL_LSB,
)

LIMIT_JSON_PATH = os.path.join(os.path.dirname(__file__), "motor_upper_limits.json")
MANUAL_LIMIT_IDS = [1, 2, 3, 4, 5]
manual_upper_limit_tick = {}

# ===== matplotlib for live debug plot =====
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_OK = True
except Exception:
    MATPLOTLIB_OK = False

# -----------------------------
# Dynamixel control table
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_CURRENT_LIMIT = 38
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCEL = 108
LEN_PRESENT_POSITION = 4
LEN_GOAL_POSITION = 4

# Extended Position Mode
EXTENDED_POSITION_MODE = 4
VELOCITY_MODE = 1

DXL_IDS = list(STIFFNESS_MODE_IDS)
BAUDRATE = DXL_BAUD
DEVICENAME = DXL_DEVICE
PROTOCOL = DXL_PROTOCOL

# -----------------------------
# force processing
FORCE_DEADBAND_N = STIFFNESS_FORCE_DEADBAND_N
FORCE_LPF_ALPHA = STIFFNESS_FORCE_LPF_ALPHA

# -----------------------------
# geometry / conversion
DEG_PER_TICK = 0.088
TICKS_PER_DEG = 1.0 / DEG_PER_TICK
VEL_UNIT_RPM_PER_LSB = 0.229

SPRING_SIGN_BY_ID = dict(STIFFNESS_SIGN_BY_ID)
CONTROL_SIGN = dict(CONTROL_SIGN_BY_ID)
CURRENT_LIMIT_MA = STIFFNESS_CURRENT_LIMIT_MA

# Ready-position conversion
B_READY_DEG_TO_CMD = 1.5689
E_READY_DEG_TO_CMD = 9.1618

# Baseline-relative safety bound
# 你指定：
# group B = baseline ± 50 * 1.5689 deg
# group E = baseline ± 60 * 9.1618 deg
GROUP_B_BOUND_DEG = 50.0 * B_READY_DEG_TO_CMD
GROUP_E_BOUND_DEG = 60.0 * E_READY_DEG_TO_CMD

GROUP_B_BOUND_TICK = int(round(GROUP_B_BOUND_DEG * TICKS_PER_DEG))
GROUP_E_BOUND_TICK = int(round(GROUP_E_BOUND_DEG * TICKS_PER_DEG))

# -----------------------------
# signal debug / trend-state detection
ENABLE_SIGNAL_DEBUG_PRINT = False
DEBUG_HX_ID = DXL_IDS[2] if DXL_IDS else 1
DEBUG_PRINT_INTERVAL_S = 0.05

ENABLE_DEBUG_PLOT = True
PLOT_HISTORY_SECONDS = 20
PLOT_UPDATE_INTERVAL_S = 0.05

ENABLE_UNITY_TELEMETRY = False

RISE_DIFF_THRESHOLD_N = 0.025
FALL_DIFF_THRESHOLD_N = 0.015
HOLD_DIFF_THRESHOLD_N = 0.010

RISE_CONFIRM_COUNT = 4
FALL_CONFIRM_COUNT = 2
HOLD_CONFIRM_COUNT = 3

# -----------------------------
lock = threading.Lock()
dxl_lock = threading.Lock()

running = True
render_enabled = False

boundary_ready = False
baseline_ready = False
trial_zero_ready = False

current_trial = None
active_ids = []

active_k_by_id = {}
active_b_by_id = {}
active_kp_by_id = {}
active_kd_by_id = {}
active_controller_by_id = {}
active_adm_m_by_id = {}
active_adm_b_by_id = {}
active_adm_k_by_id = {}
active_adm_vmax_by_id = {}
active_adm_xmax_by_id = {}
active_simple_push_gain_by_id = {}
active_simple_release_gain_by_id = {}
active_simple_push_max_by_id = {}
active_simple_release_max_by_id = {}
active_simple_err_full_scale_by_id = {}
active_profile_v_by_id = {}
active_profile_accel_by_id = {}

# 三種基準
boundary_pos = {}
baseline_pos = {}
trial_zero_pos = {}

latest_force_for_unity = 0.0
latest_disp_for_unity = 0.0
last_sent_force = None
last_sent_disp = None

F_raw = {i: 0.0 for i in DXL_IDS}
F_filt = {i: 0.0 for i in DXL_IDS}
hx_zero_offset_by_id = {i: 0.0 for i in DXL_IDS}
zeroing_in_progress = False
t_mea = 0.0

F_prev_filt = {i: 0.0 for i in DXL_IDS}
signal_state = {i: "HOLD" for i in DXL_IDS}
signal_candidate = {i: None for i in DXL_IDS}
signal_candidate_count = {i: 0 for i in DXL_IDS}
error_state_by_id = {i: "RELEASE" for i in DXL_IDS}
error_state_candidate_by_id = {i: None for i in DXL_IDS}
error_state_candidate_count_by_id = {i: 0 for i in DXL_IDS}
prev_force_error_mag_by_id = {i: 0.0 for i in DXL_IDS}
prev_force_error_dmag_by_id = {i: 0.0 for i in DXL_IDS}
error_state_match_hold_until_by_id = {i: 0.0 for i in DXL_IDS}
f_target_frozen_by_id = {i: False for i in DXL_IDS}

goal_tick_by_id = {i: 0 for i in DXL_IDS}
x_now_m_by_id = {i: 0.0 for i in DXL_IDS}
F_target_by_id = {i: 0.0 for i in DXL_IDS}
force_error_by_id = {i: 0.0 for i in DXL_IDS}
prev_force_error_by_id = {i: 0.0 for i in DXL_IDS}
prev_force_error_filt_by_id = {i: 0.0 for i in DXL_IDS}
prev_error_time_by_id = {i: time.time() for i in DXL_IDS}
prev_x_now_m_by_id = {i: 0.0 for i in DXL_IDS}
prev_x_time_by_id = {i: time.time() for i in DXL_IDS}
x_ref_m_by_id = {i: 0.0 for i in DXL_IDS}
v_ref_mps_by_id = {i: 0.0 for i in DXL_IDS}
vel_cmd_mps_by_id = {i: 0.0 for i in DXL_IDS}

error_state_runtime_preset = str(ERROR_STATE_CONFIG_PRESET)
error_state_match_enter_abs_n_by_group = dict(ERROR_STATE_MATCH_ENTER_ABS_N_BY_GROUP)
error_state_match_exit_abs_n_by_group = dict(ERROR_STATE_MATCH_EXIT_ABS_N_BY_GROUP)
error_state_match_enter_dmag_max_n_by_group = dict(ERROR_STATE_MATCH_ENTER_DMAG_MAX_N_BY_GROUP)
error_state_match_exit_dmag_min_n_by_group = dict(ERROR_STATE_MATCH_EXIT_DMAG_MIN_N_BY_GROUP)
error_state_match_enter_confirm = int(ERROR_STATE_MATCH_ENTER_CONFIRM)
error_state_match_exit_confirm = int(ERROR_STATE_MATCH_EXIT_CONFIRM)

DERR_LPF_ALPHA = 0.2

plot_t0 = time.time()
plot_time_buf = deque()
plot_fm_buf = deque()
plot_ft_buf = deque()
plot_ferr_buf = deque()
plot_state_buf = deque()
plot_error_state_buf = deque()
plot_f_target_mode_buf = deque()
plot_x_buf = deque()
last_debug_print_t = 0.0
last_error_state_debug_print_t = 0.0

pkt = None
port = None
group_sync_read_pos = None
group_sync_write_goal = None


# -----------------------------
def send_stdout(msg):
    sys.stdout.write(msg + "\n")
    sys.stdout.flush()


def debug(msg):
    sys.stderr.write(msg + "\n")
    sys.stderr.flush()


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

# -------- HX711 零点校正 --------
def zero_hx_sensors_for_trial(ids):
    """執行 HX711 零點校正：收集樣本計算平均值作為 offset"""
    global hx_zero_offset_by_id, zeroing_in_progress
    
    zeroing_in_progress = True
    debug(f"[ZERO] starting HX711 zeroing for IDs: {ids}")
    
    # 初始化樣本收集
    samples_by_id = {i: [] for i in ids}
    t_zero_start = time.time()
    
    # 收集樣本
    while len(samples_by_id[ids[0]]) < HX_ZERO_SAMPLES:
        t_now = time.time()
        if t_now - t_zero_start > HX_ZERO_TIMEOUT_S:
            debug(f"[ZERO] timeout! collected {len(samples_by_id[ids[0]])} samples")
            break
        
        with lock:
            for i in ids:
                # 取得未經 offset 的原始值
                raw_val = F_raw.get(i, 0.0)
                samples_by_id[i].append(raw_val)
        
        time.sleep(HX_ZERO_SAMPLE_INTERVAL_S)
    
    # 計算平均值作為 offset
    for i in ids:
        if samples_by_id[i]:
            avg = sum(samples_by_id[i]) / len(samples_by_id[i])
            with lock:
                hx_zero_offset_by_id[i] = avg
            debug(f"[ZERO] ID{i}: offset = {avg:.4f} N (from {len(samples_by_id[i])} samples)")
        else:
            with lock:
                hx_zero_offset_by_id[i] = 0.0
            debug(f"[ZERO] ID{i}: no samples collected, offset = 0.0")
    
    zeroing_in_progress = False
    debug(f"[ZERO] completed")

def load_manual_upper_limits():
    global manual_upper_limit_tick

    if not os.path.exists(LIMIT_JSON_PATH):
        manual_upper_limit_tick = {}
        debug(f"[LIMIT] json not found: {LIMIT_JSON_PATH}")
        return

    try:
        with open(LIMIT_JSON_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)

        manual_upper_limit_tick = {int(k): int(v) for k, v in data.items()}
        debug(f"[LIMIT] loaded: {manual_upper_limit_tick}")

    except Exception as e:
        manual_upper_limit_tick = {}
        debug(f"[LIMIT] load failed: {e}")


def save_manual_upper_limits():
    try:
        data = {str(k): int(v) for k, v in manual_upper_limit_tick.items()}
        with open(LIMIT_JSON_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        debug(f"[LIMIT] saved to {LIMIT_JSON_PATH}: {manual_upper_limit_tick}")
    except Exception as e:
        debug(f"[LIMIT] save failed: {e}")


# -----------------------------
def get_group_ids(group):
    if group == "b":
        return list(GROUP_B_IDS)
    if group == "e":
        return list(GROUP_E_IDS)
    if group == "be":
        return list(dict.fromkeys(GROUP_B_IDS + GROUP_E_IDS))
    return []


def get_cmd_deg_scale_for_id(dxl_id):
    if dxl_id in GROUP_B_IDS:
        return B_READY_DEG_TO_CMD
    if dxl_id in GROUP_E_IDS:
        return E_READY_DEG_TO_CMD
    return 1.0


def get_bound_tick_for_id(dxl_id):
    if dxl_id in GROUP_B_IDS:
        return GROUP_B_BOUND_TICK
    if dxl_id in GROUP_E_IDS:
        return GROUP_E_BOUND_TICK
    return GROUP_B_BOUND_TICK

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


def get_m_to_tick_for_id(dxl_id):
    return 1.0 / max(1e-12, get_tick_to_m_for_id(dxl_id))


def get_pid_vel_max_mps_for_id(dxl_id):
    g = get_group_name_for_id(dxl_id)
    return float(STIFFNESS_PID_VEL_MAX_MPS_BY_GROUP.get(g, 0.05))


def linear_mps_to_velocity_lsb(dxl_id, v_mps):
    omega_rad_s = float(v_mps) / max(1e-12, get_spool_radius_m_for_id(dxl_id))
    rpm = omega_rad_s * 60.0 / (2.0 * math.pi)
    vel_lsb = int(round(rpm / VEL_UNIT_RPM_PER_LSB))
    vel_lsb *= int(CONTROL_SIGN.get(dxl_id, 1))
    return int(max(-2**31, min(2**31 - 1, vel_lsb)))

def get_dx_cmd_max_m_for_id(dxl_id):
    g = get_group_name_for_id(dxl_id)
    return float(STIFFNESS_DX_CMD_MAX_M_BY_GROUP.get(g, 0.005))

def get_dx_cmd_push_max_m_for_id(dxl_id):
    g = get_group_name_for_id(dxl_id)
    return float(STIFFNESS_DX_CMD_PUSH_MAX_M_BY_GROUP.get(g, get_dx_cmd_max_m_for_id(dxl_id)))

def get_falling_push_err_scale_min_for_id(dxl_id):
    g = get_group_name_for_id(dxl_id)
    return float(STIFFNESS_FALLING_PUSH_ERR_SCALE_MIN_BY_GROUP.get(g, 1.0))

def get_falling_push_max_boost_for_id(dxl_id):
    g = get_group_name_for_id(dxl_id)
    return float(STIFFNESS_FALLING_PUSH_MAX_BOOST_BY_GROUP.get(g, 1.0))

def get_dx_cmd_release_max_m_for_id(dxl_id):
    g = get_group_name_for_id(dxl_id)
    return float(STIFFNESS_DX_CMD_RELEASE_MAX_M_BY_GROUP.get(g, get_dx_cmd_max_m_for_id(dxl_id)))

def get_rising_release_slowdown_for_id(dxl_id):
    g = get_group_name_for_id(dxl_id)
    return float(STIFFNESS_RISING_RELEASE_SLOWDOWN_BY_GROUP.get(g, 1.0))

def get_rising_target_positive_error_cap_n_for_id(dxl_id):
    g = get_group_name_for_id(dxl_id)
    return float(STIFFNESS_RISING_TARGET_POSITIVE_ERROR_CAP_N_BY_GROUP.get(g, 0.2))

def get_speed_err_full_scale_for_id(dxl_id):
    g = get_group_name_for_id(dxl_id)
    return float(STIFFNESS_SPEED_ERR_FULL_SCALE_N_BY_GROUP.get(g, 3.0))


def get_force_error_deadband_for_id(dxl_id):
    g = get_group_name_for_id(dxl_id)
    return float(STIFFNESS_FORCE_ERROR_DEADBAND_N_BY_GROUP.get(g, 0.05))


def get_match_enter_threshold_n_for_id(dxl_id, f_target_n):
    g = get_group_name_for_id(dxl_id)
    base_n = float(STIFFNESS_MATCH_ENTER_BASE_N_BY_GROUP.get(g, 0.2))
    if not STIFFNESS_MATCH_USE_DYNAMIC_THRESHOLD:
        return base_n
    min_n = float(STIFFNESS_MATCH_ENTER_MIN_N_BY_GROUP.get(g, 0.05))
    ft_scale_n = max(1e-6, float(STIFFNESS_MATCH_ENTER_FT_SCALE_N_BY_GROUP.get(g, 3.0)))
    return max(min_n, base_n / (1.0 + abs(float(f_target_n)) / ft_scale_n))


def get_match_exit_threshold_n_for_id(dxl_id, f_target_n):
    g = get_group_name_for_id(dxl_id)
    base_n = float(STIFFNESS_MATCH_EXIT_BASE_N_BY_GROUP.get(g, 0.5))
    if not STIFFNESS_MATCH_USE_DYNAMIC_THRESHOLD:
        return base_n
    min_n = float(STIFFNESS_MATCH_EXIT_MIN_N_BY_GROUP.get(g, 0.15))
    ft_scale_n = max(1e-6, float(STIFFNESS_MATCH_EXIT_FT_SCALE_N_BY_GROUP.get(g, 3.0)))
    return max(min_n, base_n / (1.0 + abs(float(f_target_n)) / ft_scale_n))


def get_error_state_match_enter_abs_n_for_id(dxl_id):
    g = get_group_name_for_id(dxl_id)
    return float(error_state_match_enter_abs_n_by_group.get(g, 0.45))


def get_error_state_match_exit_abs_n_for_id(dxl_id):
    g = get_group_name_for_id(dxl_id)
    return float(error_state_match_exit_abs_n_by_group.get(g, 0.70))


def get_error_state_match_enter_dmag_max_n_for_id(dxl_id):
    g = get_group_name_for_id(dxl_id)
    return float(error_state_match_enter_dmag_max_n_by_group.get(g, 0.010))


def get_error_state_match_exit_dmag_min_n_for_id(dxl_id):
    g = get_group_name_for_id(dxl_id)
    return float(error_state_match_exit_dmag_min_n_by_group.get(g, 0.020))


def get_gain_schedule_scale_for_id(dxl_id, force_error_abs_n):
    if not ENABLE_GAIN_SCHEDULING:
        return 1.0

    g = get_group_name_for_id(dxl_id)
    th = GAIN_SCHEDULE_THRESH_N_BY_GROUP.get(g, {})
    scale = GAIN_SCHEDULE_SCALE_BY_GROUP.get(g, {})

    th_low = float(th.get("th_low", 0.5))
    th_high = float(th.get("th_high", 2.0))

    low = float(scale.get("low", 1.0))
    mid = float(scale.get("mid", 1.0))
    high = float(scale.get("high", 1.0))

    if force_error_abs_n < th_low:
        return low
    if force_error_abs_n < th_high:
        return mid
    return high

def expand_command_to_motor_params(cmd):
    """
    把 trial command 展開成每顆 motor 的完整控制參數

    return:
        ids: list[int]
        k_by_id: dict[int, float]
        b_by_id: dict[int, float]
        kp_by_id: dict[int, float]
        kd_by_id: dict[int, float]
        controller_by_id: dict[int, str]
        adm_m_by_id: dict[int, float]
        adm_b_by_id: dict[int, float]
        adm_k_by_id: dict[int, float]
        adm_vmax_by_id: dict[int, float]
        adm_xmax_by_id: dict[int, float]
        simple_push_gain_by_id: dict[int, float]
        simple_release_gain_by_id: dict[int, float]
        simple_push_max_by_id: dict[int, float]
        simple_release_max_by_id: dict[int, float]
        simple_err_full_scale_by_id: dict[int, float]
        profile_v_by_id: dict[int, int]
        profile_accel_by_id: dict[int, int]
    """
    spec = STIFFNESS_COMMAND_SPECS.get(cmd)
    if not spec:
        return [], {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}

    ids = []
    k_by_id = {}
    b_by_id = {}
    kp_by_id = {}
    kd_by_id = {}
    controller_by_id = {}
    adm_m_by_id = {}
    adm_b_by_id = {}
    adm_k_by_id = {}
    adm_vmax_by_id = {}
    adm_xmax_by_id = {}
    simple_push_gain_by_id = {}
    simple_release_gain_by_id = {}
    simple_push_max_by_id = {}
    simple_release_max_by_id = {}
    simple_err_full_scale_by_id = {}
    profile_v_by_id = {}
    profile_accel_by_id = {}

    def apply_group(group_name, preset_name):
        if preset_name not in STIFFNESS_PRESETS:
            raise KeyError(f"Preset '{preset_name}' not found in STIFFNESS_PRESETS")

        preset = STIFFNESS_PRESETS[preset_name]
        motor_ids = GROUP_B_IDS if group_name == "b" else GROUP_E_IDS

        for dxl_id in motor_ids:
            if dxl_id not in DXL_IDS:
                continue

            ids.append(dxl_id)
            k_by_id[dxl_id] = float(preset["k"])
            b_by_id[dxl_id] = float(preset["b"])
            kp_by_id[dxl_id] = float(preset["kp"])
            kd_by_id[dxl_id] = float(preset["kd"])
            controller_by_id[dxl_id] = str(preset.get("controller", "pid")).lower()
            adm_m_by_id[dxl_id] = float(preset.get("adm_m", 0.2))
            adm_b_by_id[dxl_id] = float(preset.get("adm_b", 20.0))
            adm_k_by_id[dxl_id] = float(preset.get("adm_k", 100.0))
            adm_vmax_by_id[dxl_id] = float(preset.get("adm_vmax", 0.05))
            adm_xmax_by_id[dxl_id] = float(preset.get("adm_xmax", 0.01))
            simple_push_gain_by_id[dxl_id] = float(preset.get("simple_push_gain", 0.001))
            simple_release_gain_by_id[dxl_id] = float(preset.get("simple_release_gain", 0.002))
            simple_push_max_by_id[dxl_id] = float(preset.get("simple_push_max", 0.003))
            simple_release_max_by_id[dxl_id] = float(preset.get("simple_release_max", 0.006))
            simple_err_full_scale_by_id[dxl_id] = float(preset.get("simple_err_full_scale", 3.0))
            profile_v_by_id[dxl_id] = int(preset["profile_v"])
            profile_accel_by_id[dxl_id] = int(preset["profile_accel"])

    if "b" in spec:
        apply_group("b", spec["b"])

    if "e" in spec:
        apply_group("e", spec["e"])

    ids = list(dict.fromkeys(ids))
    return (
        ids,
        k_by_id,
        b_by_id,
        kp_by_id,
        kd_by_id,
        controller_by_id,
        adm_m_by_id,
        adm_b_by_id,
        adm_k_by_id,
        adm_vmax_by_id,
        adm_xmax_by_id,
        simple_push_gain_by_id,
        simple_release_gain_by_id,
        simple_push_max_by_id,
        simple_release_max_by_id,
        simple_err_full_scale_by_id,
        profile_v_by_id,
        profile_accel_by_id,
    )


# -----------------------------
def compute_force_for_unity():
    global latest_force_for_unity

    with lock:
        if not render_enabled or not active_ids:
            latest_force_for_unity = 0.0
            return

        vals = [F_raw[i] for i in active_ids if i in F_raw]
        latest_force_for_unity = (sum(vals) / len(vals)) if vals else 0.0


def compute_disp_for_unity():
    global latest_disp_for_unity

    with lock:
        if not render_enabled or not active_ids or not trial_zero_pos:
            latest_disp_for_unity = 0.0
            return

        active_set = set(active_ids)
        b_set = set(GROUP_B_IDS)
        e_set = set(GROUP_E_IDS)

        if (active_set & b_set) and not (active_set & e_set):
            ids_to_avg = list(active_set & b_set)
        elif (active_set & e_set) and not (active_set & b_set):
            ids_to_avg = list(active_set & e_set)
        elif (active_set & b_set) and (active_set & e_set):
            ids_to_avg = list(active_set & e_set)
        else:
            latest_disp_for_unity = 0.0
            return

    vals = []
    for i in ids_to_avg:
        try:
            pos_now = read_present_position(i)
            pos_zero = trial_zero_pos.get(i, pos_now)
            delta_tick = pos_now - pos_zero
            delta_m = delta_tick * TICK_TO_M
            vals.append(delta_m)
        except Exception:
            continue

    with lock:
        latest_disp_for_unity = (sum(vals) / len(vals)) if vals else 0.0


# -----------------------------
def get_required_confirm_count(target_state):
    if target_state == "RISING":
        return RISE_CONFIRM_COUNT
    if target_state == "FALLING":
        return FALL_CONFIRM_COUNT
    return HOLD_CONFIRM_COUNT


def classify_state_by_df(dF):
    if dF >= RISE_DIFF_THRESHOLD_N:
        return "RISING"
    if dF <= -FALL_DIFF_THRESHOLD_N:
        return "FALLING"
    if abs(dF) <= HOLD_DIFF_THRESHOLD_N:
        return "HOLD"
    return None


def update_signal_state(dxl_id, f_filtered):
    prev = F_prev_filt[dxl_id]
    dF = f_filtered - prev

    candidate = classify_state_by_df(dF)

    if candidate is None:
        F_prev_filt[dxl_id] = f_filtered
        return signal_state[dxl_id], dF

    if candidate == signal_state[dxl_id]:
        signal_candidate[dxl_id] = None
        signal_candidate_count[dxl_id] = 0
    else:
        if candidate == signal_candidate[dxl_id]:
            signal_candidate_count[dxl_id] += 1
        else:
            signal_candidate[dxl_id] = candidate
            signal_candidate_count[dxl_id] = 1

        need_count = get_required_confirm_count(candidate)
        if signal_candidate_count[dxl_id] >= need_count:
            signal_state[dxl_id] = candidate
            signal_candidate[dxl_id] = None
            signal_candidate_count[dxl_id] = 0

    F_prev_filt[dxl_id] = f_filtered
    return signal_state[dxl_id], dF


def classify_error_state(dxl_id, force_error):
    current = error_state_by_id[dxl_id]
    err_mag = abs(force_error)
    prev_mag = prev_force_error_mag_by_id[dxl_id]
    dmag = err_mag - prev_mag

    enter_abs_n = get_error_state_match_enter_abs_n_for_id(dxl_id)
    exit_abs_n = get_error_state_match_exit_abs_n_for_id(dxl_id)
    enter_dmag_max_n = get_error_state_match_enter_dmag_max_n_for_id(dxl_id)
    exit_dmag_min_n = get_error_state_match_exit_dmag_min_n_for_id(dxl_id)

    candidate = current

    if current == "MATCH":
        if force_error > exit_abs_n and dmag >= exit_dmag_min_n:
            candidate = "PUSH"
        elif force_error < -exit_abs_n and dmag >= exit_dmag_min_n:
            candidate = "RELEASE"
        else:
            candidate = "MATCH"
    else:
        # Enter MATCH only when the magnitude is already small and its
        # growth has flattened or turned downward. This makes MATCH entry
        # stricter than exit.
        if err_mag <= enter_abs_n and dmag <= enter_dmag_max_n:
            candidate = "MATCH"
        elif force_error > 0.0:
            candidate = "PUSH"
        elif force_error < 0.0:
            candidate = "RELEASE"
        else:
            candidate = current

    if candidate == current:
        error_state_candidate_by_id[dxl_id] = None
        error_state_candidate_count_by_id[dxl_id] = 0
    else:
        if candidate == error_state_candidate_by_id[dxl_id]:
            error_state_candidate_count_by_id[dxl_id] += 1
        else:
            error_state_candidate_by_id[dxl_id] = candidate
            error_state_candidate_count_by_id[dxl_id] = 1

        need_count = (
            int(error_state_match_enter_confirm)
            if candidate == "MATCH"
            else int(error_state_match_exit_confirm)
        )

        if error_state_candidate_count_by_id[dxl_id] >= need_count:
            prev_state = error_state_by_id[dxl_id]
            error_state_by_id[dxl_id] = candidate
            error_state_candidate_by_id[dxl_id] = None
            error_state_candidate_count_by_id[dxl_id] = 0
            if prev_state != "MATCH" and candidate == "MATCH":
                error_state_match_hold_until_by_id[dxl_id] = time.time() + float(ERROR_STATE_MATCH_HOLD_SECONDS)

    prev_force_error_mag_by_id[dxl_id] = err_mag
    return error_state_by_id[dxl_id], dmag


def apply_error_state_preset(preset_name):
    global error_state_runtime_preset
    global error_state_match_enter_abs_n_by_group
    global error_state_match_exit_abs_n_by_group
    global error_state_match_enter_dmag_max_n_by_group
    global error_state_match_exit_dmag_min_n_by_group
    global error_state_match_enter_confirm
    global error_state_match_exit_confirm

    key = str(preset_name).strip().lower()
    preset = ERROR_STATE_PRESETS.get(key)
    if preset is None:
        raise ValueError(f"unknown_error_state_preset:{preset_name}")

    error_state_runtime_preset = key
    error_state_match_enter_abs_n_by_group = dict(preset["enter_abs"])
    error_state_match_exit_abs_n_by_group = dict(preset["exit_abs"])
    error_state_match_enter_dmag_max_n_by_group = dict(preset["enter_dmag_max"])
    error_state_match_exit_dmag_min_n_by_group = dict(preset["exit_dmag_min"])
    error_state_match_enter_confirm = int(preset["enter_confirm"])
    error_state_match_exit_confirm = int(preset["exit_confirm"])

    for i in DXL_IDS:
        error_state_candidate_by_id[i] = None
        error_state_candidate_count_by_id[i] = 0
        prev_force_error_mag_by_id[i] = abs(force_error_by_id.get(i, 0.0))


def handle_error_state_preset(cmd):
    raw = str(cmd or "").strip().lower()
    if raw in ("epc", "presetc", "errc"):
        preset = "conservative"
    elif raw in ("epm", "presetm", "errm"):
        preset = "medium"
    elif raw in ("eps", "presets", "errs"):
        preset = "sensitive"
    elif raw.startswith("ep "):
        preset = raw[3:].strip()
    elif raw.startswith("preset "):
        preset = raw[7:].strip()
    elif raw.startswith("errpreset "):
        preset = raw[10:].strip()
    else:
        preset = raw

    apply_error_state_preset(preset)
    debug(f"[ERR_STATE] preset -> {error_state_runtime_preset}")
    return "K"


def debug_error_state_status(
    dxl_id,
    t_now,
    f_target,
    f_measured,
    force_error,
    dmag,
    current_state,
    candidate_state,
    candidate_count,
    f_target_mode,
    vel_cmd_mps=None,
    vel_lsb=None,
    match_enter_th_n=None,
    match_exit_th_n=None,
):
    global last_error_state_debug_print_t

    if dxl_id != DEBUG_HX_ID:
        return
    if (t_now - last_error_state_debug_print_t) < DEBUG_PRINT_INTERVAL_S:
        return

    enter_dmag_max_n = get_error_state_match_enter_dmag_max_n_for_id(dxl_id)
    exit_dmag_min_n = get_error_state_match_exit_dmag_min_n_for_id(dxl_id)
    err_mag = abs(force_error)
    if match_enter_th_n is None:
        match_enter_th_n = get_match_enter_threshold_n_for_id(dxl_id, f_target)
    if match_exit_th_n is None:
        match_exit_th_n = get_match_exit_threshold_n_for_id(dxl_id, f_target)

    vel_part = ""
    if vel_cmd_mps is not None:
        vel_part = f" | vcmd={vel_cmd_mps:+.5f} m/s"
        if vel_lsb is not None:
            vel_part += f" | vlsb={int(vel_lsb)}"

    debug(
        f"[ERR{dxl_id}] "
        f"preset={error_state_runtime_preset} | "
        f"Fm={f_measured:+.3f} | Ft={f_target:+.3f} | Ferr={force_error:+.3f} | "
        f"|Ferr|={err_mag:.3f} | d|Ferr|={dmag:+.4f} | "
        f"state={current_state} | cand={candidate_state}({candidate_count}) | "
        f"mode={f_target_mode} | "
        f"{vel_part}"
        f" | "
        f"match_in<={match_enter_th_n:.3f} enter_dmag<={enter_dmag_max_n:.4f} | "
        f"match_out>={match_exit_th_n:.3f} exit_dmag>={exit_dmag_min_n:.4f} | "
        f"confirm_in={error_state_match_enter_confirm} confirm_out={error_state_match_exit_confirm}"
    )
    last_error_state_debug_print_t = t_now


def append_debug_sample(t_now, x_m, f_measured, f_target, force_error, state_str, error_state_str, f_target_mode_str):
    t_rel = t_now - plot_t0

    plot_time_buf.append(t_rel)
    plot_x_buf.append(x_m)
    plot_fm_buf.append(f_measured)
    plot_ft_buf.append(f_target)
    plot_ferr_buf.append(force_error)
    plot_state_buf.append(state_str)
    plot_error_state_buf.append(error_state_str)
    plot_f_target_mode_buf.append(f_target_mode_str)

    cutoff = t_rel - PLOT_HISTORY_SECONDS
    while plot_time_buf and plot_time_buf[0] < cutoff:
        plot_time_buf.popleft()
        plot_x_buf.popleft()
        plot_fm_buf.popleft()
        plot_ft_buf.popleft()
        plot_ferr_buf.popleft()
        plot_state_buf.popleft()
        plot_error_state_buf.popleft()
        plot_f_target_mode_buf.popleft()


# -----------------------------
def parse_hx_line(line):
    parts = [p.strip() for p in line.split(",") if p.strip() != ""]
    values = []
    for p in parts[:HX_MAX_VALUES]:
        values.append(float(p))
    return values


def hx711_thread_csv():
    global t_mea, last_debug_print_t

    ser = serial.Serial(HX_PORT, HX_BAUD, timeout=HX_TIMEOUT)
    debug(f"[HX] connected: {HX_PORT}")

    while running:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        try:
            vals = parse_hx_line(line)
            t_now = time.time()

            with lock:
                for dxl_id in DXL_IDS:
                    hx_idx = DXL_HX_INDEX.get(dxl_id, None)
                    raw_f = 0.0
                    if hx_idx is not None and 0 <= hx_idx < len(vals):
                        raw_f = float(vals[hx_idx])

                    F_raw[dxl_id] = raw_f

                    # 減去零點偏移（如果有設定的話）
                    f = raw_f - hx_zero_offset_by_id.get(dxl_id, 0.0)
                    if abs(f) < FORCE_DEADBAND_N:
                        f = 0.0

                    F_filt[dxl_id] = (1 - FORCE_LPF_ALPHA) * F_filt[dxl_id] + FORCE_LPF_ALPHA * f
                    state_now, dF = update_signal_state(dxl_id, F_filt[dxl_id])

                    if (
                        dxl_id == DEBUG_HX_ID
                        and ENABLE_SIGNAL_DEBUG_PRINT
                        and (t_now - last_debug_print_t) >= DEBUG_PRINT_INTERVAL_S
                    ):
                        debug(
                            f"[HX{dxl_id}] "
                            f"raw={F_raw[dxl_id]:+.3f} N | "
                            f"filt={F_filt[dxl_id]:+.3f} N | "
                            f"dF={dF:+.4f} N | "
                            f"state={state_now}"
                        )
                        last_debug_print_t = t_now

                t_mea = t_now

            compute_force_for_unity()

        except Exception as e:
            debug(f"[HX] parse error: {e} | line={line}")
            continue


# -----------------------------
def debug_plot_thread():
    if not ENABLE_DEBUG_PLOT:
        return

    if not MATPLOTLIB_OK:
        debug("[PLOT] matplotlib import failed, plot disabled")
        return

    if threading.current_thread() is not threading.main_thread():
        debug("[PLOT] live plot requires the main thread; plot disabled for this thread")
        return

    plt.ion()
    fig, ax1_force = plt.subplots(1, 1, figsize=(10, 5))

    while running:
        try:
            if not plt.fignum_exists(fig.number):
                break

            with lock:
                t_list = list(plot_time_buf)
                x_list = list(plot_x_buf)
                fm_list = list(plot_fm_buf)
                ft_list = list(plot_ft_buf)
                ferr_list = list(plot_ferr_buf)
                s_list = list(plot_state_buf)
                e_list = list(plot_error_state_buf)
                ft_mode_list = list(plot_f_target_mode_buf)

            ax1_force.clear()

            if len(t_list) >= 2:
                ax1_force.plot(t_list, fm_list, color="tab:blue", linewidth=2, label="F_m")
                ax1_force.plot(t_list, ft_list, color="tab:green", linewidth=2, label="F_t")
                ax1_force.plot(t_list, ferr_list, color="tab:red", linewidth=2, label="F_error")

                ax1_force.set_xlim(
                    max(0.0, t_list[-1] - PLOT_HISTORY_SECONDS),
                    max(PLOT_HISTORY_SECONDS, t_list[-1]),
                )

                # Annotate latest error_state on the time plot so we can see
                # whether the controller currently wants to push, release, or match.
                if e_list:
                    err_color = {
                        "PUSH": "tab:purple",
                        "RELEASE": "tab:brown",
                        "MATCH": "tab:gray",
                    }.get(e_list[-1], "black")
                    ax1_force.text(
                        0.02,
                        0.95,
                        f"error_state: {e_list[-1]}",
                        transform=ax1_force.transAxes,
                        ha="left",
                        va="top",
                        fontsize=10,
                        color=err_color,
                        bbox=dict(facecolor="white", edgecolor=err_color, alpha=0.8),
                    )

                if ft_mode_list:
                    ft_mode = ft_mode_list[-1]
                    ft_color = "black" if ft_mode == "LIVE" else "tab:olive"
                    ax1_force.text(
                        0.02,
                        0.77,
                        f"F_target: {ft_mode}",
                        transform=ax1_force.transAxes,
                        ha="left",
                        va="top",
                        fontsize=10,
                        color=ft_color,
                        bbox=dict(facecolor="white", edgecolor=ft_color, alpha=0.8),
                    )

                if s_list:
                    trend_color = {
                        "RISING": "tab:blue",
                        "FALLING": "tab:red",
                        "HOLD": "tab:orange",
                    }.get(s_list[-1], "black")
                    ax1_force.text(
                        0.02,
                        0.86,
                        f"signal_state: {s_list[-1]}",
                        transform=ax1_force.transAxes,
                        ha="left",
                        va="top",
                        fontsize=10,
                        color=trend_color,
                        bbox=dict(facecolor="white", edgecolor=trend_color, alpha=0.8),
                    )

            ax1_force.set_title(f"HX711 ID {DEBUG_HX_ID}: F_m, F_t, F_error vs Time")
            ax1_force.set_xlabel("Time (s)")
            ax1_force.set_ylabel("Force (N)")
            ax1_force.grid(True, alpha=0.3)

            force_handles = [
                plt.Line2D([0], [0], color="tab:blue", linewidth=2, label="F_m"),
                plt.Line2D([0], [0], color="tab:green", linewidth=2, label="F_t"),
                plt.Line2D([0], [0], color="tab:red", linewidth=2, label="F_error"),
                plt.Line2D([0], [0], color="tab:purple", marker="s", linestyle="None", label="error PUSH"),
                plt.Line2D([0], [0], color="tab:brown", marker="^", linestyle="None", label="error RELEASE"),
                plt.Line2D([0], [0], color="tab:gray", marker="o", linestyle="None", label="error MATCH"),
                plt.Line2D([0], [0], color="black", linewidth=2, label="F_target LIVE"),
                plt.Line2D([0], [0], color="tab:olive", linewidth=2, label="F_target FROZEN"),
            ]
            ax1_force.legend(force_handles, [h.get_label() for h in force_handles], loc="best")

            fig.tight_layout()
            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.pause(0.001)

        except Exception as e:
            debug(f"[PLOT] error: {e}")

        time.sleep(PLOT_UPDATE_INTERVAL_S)

    try:
        plt.close(fig)
    except Exception:
        pass


# -----------------------------
def read_present_position(dxl_id):
    with dxl_lock:
        pos, dxl_comm_result, _ = pkt.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)

    if dxl_comm_result != COMM_SUCCESS:
        raise RuntimeError(f"read pos fail ID{dxl_id}: {pkt.getTxRxResult(dxl_comm_result)}")

    # unsigned 32-bit -> signed 32-bit
    if pos >= 0x80000000:
        pos -= 0x100000000

    return int(pos)


def sync_read_present_positions(dxl_ids):
    pos_by_id = {}
    if not dxl_ids:
        return pos_by_id

    with dxl_lock:
        group_sync_read_pos.clearParam()

        for dxl_id in dxl_ids:
            if not group_sync_read_pos.addParam(int(dxl_id)):
                raise RuntimeError(f"sync read addParam fail ID{dxl_id}")

        dxl_comm_result = group_sync_read_pos.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(f"sync read pos fail: {pkt.getTxRxResult(dxl_comm_result)}")

        for dxl_id in dxl_ids:
            if not group_sync_read_pos.isAvailable(int(dxl_id), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                raise RuntimeError(f"sync read pos unavailable ID{dxl_id}")

            pos = group_sync_read_pos.getData(int(dxl_id), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if pos >= 0x80000000:
                pos -= 0x100000000
            pos_by_id[int(dxl_id)] = int(pos)

    return pos_by_id


def write_goal_position(dxl_id, goal):
    with dxl_lock:
        dxl_comm_result, _ = pkt.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(goal))

    if dxl_comm_result != COMM_SUCCESS:
        raise RuntimeError(f"write goal fail ID{dxl_id}: {pkt.getTxRxResult(dxl_comm_result)}")


def write_goal_velocity(dxl_id, vel_lsb):
    with dxl_lock:
        dxl_comm_result, _ = pkt.write4ByteTxRx(port, dxl_id, ADDR_GOAL_VELOCITY, int(vel_lsb) & 0xFFFFFFFF)

    if dxl_comm_result != COMM_SUCCESS:
        raise RuntimeError(f"write goal velocity fail ID{dxl_id}: {pkt.getTxRxResult(dxl_comm_result)}")


def sync_write_goal_positions(pos_map):
    if not pos_map:
        return

    with dxl_lock:
        group_sync_write_goal.clearParam()

        for dxl_id, goal in pos_map.items():
            goal_u32 = int(goal) & 0xFFFFFFFF
            param = [
                goal_u32 & 0xFF,
                (goal_u32 >> 8) & 0xFF,
                (goal_u32 >> 16) & 0xFF,
                (goal_u32 >> 24) & 0xFF,
            ]
            if not group_sync_write_goal.addParam(int(dxl_id), param):
                raise RuntimeError(f"sync write addParam fail ID{dxl_id}")

        dxl_comm_result = group_sync_write_goal.txPacket()
        group_sync_write_goal.clearParam()

    if dxl_comm_result != COMM_SUCCESS:
        raise RuntimeError(f"sync write goal fail: {pkt.getTxRxResult(dxl_comm_result)}")


def set_motor_profile(dxl_id, profile_v=None, profile_accel=None):
    with dxl_lock:
        if profile_accel is not None:
            dxl_comm_result, _ = pkt.write4ByteTxRx(
                port, dxl_id, ADDR_PROFILE_ACCEL, int(profile_accel)
            )
            if dxl_comm_result != COMM_SUCCESS:
                raise RuntimeError(
                    f"write profile_accel fail ID{dxl_id}: {pkt.getTxRxResult(dxl_comm_result)}"
                )

        if profile_v is not None:
            dxl_comm_result, _ = pkt.write4ByteTxRx(
                port, dxl_id, ADDR_PROFILE_VELOCITY, int(profile_v)
            )
            if dxl_comm_result != COMM_SUCCESS:
                raise RuntimeError(
                    f"write profile_v fail ID{dxl_id}: {pkt.getTxRxResult(dxl_comm_result)}"
                )


def init_dynamixel():
    operating_mode = VELOCITY_MODE if str(STIFFNESS_PID_OUTPUT_MODE).lower() == "velocity" else EXTENDED_POSITION_MODE
    for i in DXL_IDS:
        with dxl_lock:
            pkt.write1ByteTxRx(port, i, ADDR_TORQUE_ENABLE, 0)
            pkt.write1ByteTxRx(port, i, ADDR_OPERATING_MODE, operating_mode)
            pkt.write2ByteTxRx(port, i, ADDR_CURRENT_LIMIT, int(CURRENT_LIMIT_MA))
            pkt.write4ByteTxRx(port, i, ADDR_PROFILE_ACCEL, 20)
            pkt.write4ByteTxRx(port, i, ADDR_PROFILE_VELOCITY, 80)
            pkt.write4ByteTxRx(port, i, ADDR_GOAL_VELOCITY, 0)
            pkt.write1ByteTxRx(port, i, ADDR_TORQUE_ENABLE, 1)

def set_torque_enable(dxl_id, enable):
    with dxl_lock:
        dxl_comm_result, _ = pkt.write1ByteTxRx(
            port, dxl_id, ADDR_TORQUE_ENABLE, 1 if enable else 0
        )
    if dxl_comm_result != COMM_SUCCESS:
        raise RuntimeError(
            f"torque {'on' if enable else 'off'} fail ID{dxl_id}: {pkt.getTxRxResult(dxl_comm_result)}"
        )


def set_torque_enable_many(dxl_ids, enable):
    for dxl_id in dxl_ids:
        if dxl_id in DXL_IDS:
            set_torque_enable(dxl_id, enable)


def stop_velocity_many(dxl_ids):
    if str(STIFFNESS_PID_OUTPUT_MODE).lower() != "velocity":
        return
    for dxl_id in dxl_ids:
        if dxl_id in DXL_IDS:
            try:
                write_goal_velocity(dxl_id, 0)
            except Exception:
                pass


def calibrate_manual_upper_limits():
    """
    讓使用者手動把 ID1~5 轉到「最大允許位置」後按 Enter，
    讀取 present position 並存成 json。
    """
    global manual_upper_limit_tick

    debug("")
    debug("========== Manual Upper Limit Calibration ==========")
    debug("請依序手動把 ID1~5 轉到『最大允許位置』後按 Enter")
    debug("若某顆不想重設，直接輸入 s 再 Enter 可跳過")
    debug("===================================================")
    debug("")

    # 先關 torque，才方便手轉
    for dxl_id in MANUAL_LIMIT_IDS:
        if dxl_id in DXL_IDS:
            try:
                set_torque_enable(dxl_id, False)
            except Exception as e:
                debug(f"[LIMIT] torque off failed ID{dxl_id}: {e}")

    time.sleep(0.2)

    # 若已有舊檔先載入，跳過時可保留
    load_manual_upper_limits()

    for dxl_id in MANUAL_LIMIT_IDS:
        if dxl_id not in DXL_IDS:
            continue

        try:
            prev = manual_upper_limit_tick.get(dxl_id, None)
            if prev is not None:
                debug(f"[LIMIT] ID{dxl_id} current saved upper = {prev}")

            user_in = input(f"Move ID{dxl_id} to UPPER LIMIT, then press Enter (or type s to skip): ").strip()

            if user_in.lower() == "s":
                debug(f"[LIMIT] ID{dxl_id} skipped")
                continue

            pos = read_present_position(dxl_id)
            manual_upper_limit_tick[dxl_id] = int(pos)
            debug(f"[LIMIT] ID{dxl_id} recorded upper limit = {pos}")

        except Exception as e:
            debug(f"[LIMIT] ID{dxl_id} calibration failed: {e}")

    save_manual_upper_limits()
    debug("[LIMIT] calibration done")
    debug("")

def get_goal_bounds(boundary_val, dxl_id):
    baseline_val = baseline_pos.get(dxl_id, None)
    if boundary_ready and baseline_ready and baseline_val is not None:
        goal_min = min(int(boundary_val), int(baseline_val))
        goal_max = max(int(boundary_val), int(baseline_val))
    else:
        bound_tick = get_bound_tick_for_id(dxl_id)
        goal_min = boundary_val - bound_tick
        goal_max = boundary_val + bound_tick

    # 對 ID1~5 再加上手動校正的「絕對上限」
    if dxl_id in MANUAL_LIMIT_IDS:
        manual_max = manual_upper_limit_tick.get(dxl_id, None)
        if manual_max is not None:
            goal_max = min(goal_max, int(manual_max))

    return int(goal_min), int(goal_max)


def move_all_to_positions(pos_map, settle=0.7, profile_v_map=None, profile_accel_map=None):
    for mid, goal in pos_map.items():
        pv = None if profile_v_map is None else profile_v_map.get(mid)
        pa = None if profile_accel_map is None else profile_accel_map.get(mid)

        if pv is not None or pa is not None:
            set_motor_profile(mid, profile_v=pv, profile_accel=pa)
        else:
            with dxl_lock:
                pkt.write4ByteTxRx(port, mid, ADDR_PROFILE_VELOCITY, 180)

    sync_write_goal_positions(pos_map)

    time.sleep(settle)


def move_all_to_positions_velocity(
    pos_map,
    settle=0.3,
    timeout_s=4.0,
    tol_tick=12,
    min_vel_lsb=35,
    max_vel_lsb=120,
):
    if not pos_map:
        return

    ids = list(pos_map.keys())
    deadline = time.time() + float(timeout_s)

    try:
        while time.time() < deadline:
            pos_now_by_id = sync_read_present_positions(ids)
            all_reached = True

            for dxl_id, goal_tick in pos_map.items():
                pos_now = pos_now_by_id.get(dxl_id)
                if pos_now is None:
                    continue

                err_tick = int(goal_tick) - int(pos_now)
                if abs(err_tick) <= int(tol_tick):
                    vel_lsb = 0
                else:
                    all_reached = False
                    vel_mag = int(clamp(abs(err_tick) // 6, int(min_vel_lsb), int(max_vel_lsb)))
                    vel_lsb = vel_mag if err_tick > 0 else -vel_mag

                write_goal_velocity(dxl_id, vel_lsb)

            if all_reached:
                break

            time.sleep(max(DT_LOOP, 0.02))
    finally:
        stop_velocity_many(ids)
        time.sleep(settle)

# -----------------------------
def render_loop():
    global running

    t_prev = time.time()
    use_velocity_output = str(STIFFNESS_PID_OUTPUT_MODE).lower() == "velocity"

    while running:
        t_now = time.time()
        dt = t_now - t_prev
        if dt < DT_LOOP:
            time.sleep(DT_LOOP - dt)
            continue

        t_prev = t_now

        with lock:
            do_render = render_enabled
            ids = list(active_ids)
            trial = current_trial
            mea_ok = (t_now - t_mea) < 0.5
            boundary_local = dict(boundary_pos)
            zero_local = dict(trial_zero_pos)

            k_local = dict(active_k_by_id)
            kp_local = dict(active_kp_by_id)
            kd_local = dict(active_kd_by_id)
            controller_local = dict(active_controller_by_id)
            simple_push_gain_local = dict(active_simple_push_gain_by_id)
            simple_release_gain_local = dict(active_simple_release_gain_by_id)
            simple_push_max_local = dict(active_simple_push_max_by_id)
            simple_release_max_local = dict(active_simple_release_max_by_id)
            simple_err_full_scale_local = dict(active_simple_err_full_scale_by_id)

        if not do_render:
            continue

        if not trial:
            continue

        try:
            pos_now_tick_by_id = sync_read_present_positions(ids)
        except Exception as e:
            debug(str(e))
            continue

        pending_goal_map = {}
        pending_updates = []

        for i in ids:
            if i not in boundary_local or i not in zero_local:
                continue

            if i not in pos_now_tick_by_id:
                debug(f"sync read missing ID{i}")
                continue
            pos_now_tick = pos_now_tick_by_id[i]
            
            x_sign = -1 if i in GROUP_E_IDS else 1
            x_now_m = x_sign * (pos_now_tick - zero_local[i]) * get_tick_to_m_for_id(i)
            F_measured = F_filt[i] if mea_ok else 0.0

            k = k_local.get(i, 0.0)
            kp = kp_local.get(i, 0.0)
            kd = kd_local.get(i, 0.0)
            controller = controller_local.get(i, "pid")
            sgn = SPRING_SIGN_BY_ID.get(i, -1)
            prev_error_state = error_state_by_id[i]
            prev_frozen_target = F_target_by_id[i]
            prev_frozen_active = f_target_frozen_by_id[i]
            using_e_target_floor = False

            F_target_live = (-sgn) * k * x_now_m
            use_frozen_target = (
                controller != "simple_admittance"
                and prev_error_state == "MATCH"
                and prev_frozen_active
            )
            F_target = prev_frozen_target if use_frozen_target else F_target_live
            if i in GROUP_E_IDS and F_measured < STIFFNESS_E_GROUP_MIN_FM_FOR_TARGET_FLOOR_N:
                F_target = STIFFNESS_E_GROUP_TARGET_FLOOR_N
                using_e_target_floor = True
            f_target_mode = "FROZEN" if use_frozen_target else "LIVE"
            match_enter_th_n = get_match_enter_threshold_n_for_id(i, F_target)
            match_exit_th_n = get_match_exit_threshold_n_for_id(i, F_target)

            force_error_raw = F_target - F_measured
            force_error_deadband_n = get_force_error_deadband_for_id(i)
            force_error = 0.0 if abs(force_error_raw) < force_error_deadband_n else force_error_raw
            err_mag = abs(force_error)
            prev_err_mag = prev_force_error_mag_by_id[i]
            prev_dmag = prev_force_error_dmag_by_id[i]
            dmag = err_mag - prev_err_mag

            error_state = prev_error_state
            if controller == "simple_admittance":
                if force_error > 0.0:
                    error_state = "PUSH"
                elif force_error < 0.0:
                    error_state = "RELEASE"
                else:
                    error_state = prev_error_state
            else:
                if prev_error_state == "MATCH":
                    # Leave MATCH when the force relationship clearly separates again
                    # in either direction.
                    if force_error > match_exit_th_n:
                        error_state = "PUSH"
                        prev_frozen_active = False
                        F_target = F_target_live
                        f_target_mode = "LIVE"
                        match_enter_th_n = get_match_enter_threshold_n_for_id(i, F_target)
                        match_exit_th_n = get_match_exit_threshold_n_for_id(i, F_target)
                        force_error_raw = F_target - F_measured
                        force_error = 0.0 if abs(force_error_raw) < force_error_deadband_n else force_error_raw
                        err_mag = abs(force_error)
                        dmag = err_mag - prev_err_mag
                    elif force_error < -match_exit_th_n:
                        error_state = "RELEASE"
                        prev_frozen_active = False
                        F_target = F_target_live
                        f_target_mode = "LIVE"
                        match_enter_th_n = get_match_enter_threshold_n_for_id(i, F_target)
                        match_exit_th_n = get_match_exit_threshold_n_for_id(i, F_target)
                        force_error_raw = F_target - F_measured
                        force_error = 0.0 if abs(force_error_raw) < force_error_deadband_n else force_error_raw
                        err_mag = abs(force_error)
                        dmag = err_mag - prev_err_mag
                    else:
                        error_state = "MATCH"
                else:
                    if force_error > 0.0:
                        error_state = "PUSH"
                    elif force_error < 0.0:
                        error_state = "RELEASE"
                    else:
                        error_state = prev_error_state

            prev_err = prev_force_error_by_id[i]
            prev_err_t = prev_error_time_by_id[i]
            dt_err = max(1e-6, t_now - prev_err_t)
            d_err = (force_error - prev_err) / dt_err
            signal_now = signal_state.get(i, "HOLD")

            if controller == "simple_admittance":
                err_full_scale = max(1e-6, simple_err_full_scale_local.get(i, 3.0))
                err_scale = clamp(abs(force_error) / err_full_scale, 0.0, 1.0)
                if force_error > 0.0:
                    push_max = simple_push_max_local.get(i, 0.003) * err_scale
                    dx_cmd_m = -simple_push_gain_local.get(i, 0.001) * force_error
                    dx_cmd_m = clamp(dx_cmd_m, -push_max, push_max)
                elif force_error < 0.0:
                    release_max = simple_release_max_local.get(i, 0.006) * err_scale
                    dx_cmd_m = -simple_release_gain_local.get(i, 0.002) * force_error
                    dx_cmd_m = clamp(dx_cmd_m, -release_max, release_max)
                else:
                    dx_cmd_m = 0.0
            else:
                dx_cmd_m = -(kp * force_error + kd * d_err)
                err_scale = clamp(abs(force_error) / max(1e-6, get_speed_err_full_scale_for_id(i)), 0.0, 1.0)
                if force_error > 0.0:
                    if signal_now == "FALLING":
                        err_scale = max(err_scale, get_falling_push_err_scale_min_for_id(i))
                    dx_cmd_max_m = get_dx_cmd_push_max_m_for_id(i) * err_scale
                    if signal_now == "FALLING":
                        dx_cmd_max_m *= get_falling_push_max_boost_for_id(i)
                    dx_cmd_max_m = min(dx_cmd_max_m, get_dx_cmd_max_m_for_id(i))
                elif force_error < 0.0:
                    dx_cmd_max_m = get_dx_cmd_release_max_m_for_id(i) * err_scale
                else:
                    dx_cmd_max_m = 0.0
                dx_cmd_m = clamp(dx_cmd_m, -dx_cmd_max_m, dx_cmd_max_m)

                # PID match capture:
                # while retreating (RELEASE), if the loop reaches the narrow band
                # 0 <= Ft-Fm < 0.2 N, freeze Ft and stop motor motion.
                if prev_error_state == "RELEASE" and 0.0 <= force_error < match_enter_th_n:
                    error_state = "MATCH"
                    prev_frozen_active = True
                    f_target_mode = "FROZEN"
                    dx_cmd_m = 0.0
                    F_target = prev_frozen_target = F_target

            with lock:
                goal_prev = goal_tick_by_id[i]

            goal_min, goal_max = get_goal_bounds(boundary_local[i], i)

            if use_velocity_output:
                vel_cmd_mps = dx_cmd_m
                vel_cmd_mps = clamp(vel_cmd_mps, -get_pid_vel_max_mps_for_id(i), get_pid_vel_max_mps_for_id(i))
                vel_prev = vel_cmd_mps_by_id.get(i, 0.0)
                vel_cmd_mps = (
                    (1.0 - float(STIFFNESS_PID_VEL_LPF_ALPHA)) * vel_prev
                    + float(STIFFNESS_PID_VEL_LPF_ALPHA) * vel_cmd_mps
                )
                vel_lsb = linear_mps_to_velocity_lsb(i, vel_cmd_mps)
                if using_e_target_floor:
                    floor_lsb = abs(int(STIFFNESS_E_GROUP_TARGET_FLOOR_VEL_LSB))
                    if vel_lsb > 0:
                        vel_lsb = max(vel_lsb, floor_lsb)
                    elif vel_lsb < 0:
                        vel_lsb = min(vel_lsb, -floor_lsb)
                    else:
                        vel_lsb = floor_lsb

                if vel_lsb < 0 and pos_now_tick <= goal_min:
                    vel_lsb = 0
                    vel_cmd_mps = 0.0
                elif vel_lsb > 0 and pos_now_tick >= goal_max:
                    vel_lsb = 0
                    vel_cmd_mps = 0.0

                should_write = True
                goal_new = goal_prev
                d_tick = 0
                pending_goal_map[i] = vel_lsb
            else:
                ctrl_sign = CONTROL_SIGN.get(i, 1)
                d_tick = int(round(dx_cmd_m * get_m_to_tick_for_id(i)))
                d_tick *= ctrl_sign

                goal_new = goal_prev + d_tick
                goal_new = int(clamp(goal_new, goal_min, goal_max))
                should_write = (goal_new != goal_prev)
                if should_write:
                    pending_goal_map[i] = goal_new
                vel_cmd_mps = 0.0
                vel_lsb = 0

            debug_error_state_status(
                dxl_id=i,
                t_now=t_now,
                f_target=F_target,
                f_measured=F_measured,
                force_error=force_error,
                dmag=dmag,
                current_state=error_state,
                candidate_state=None,
                candidate_count=0,
                f_target_mode=f_target_mode,
                vel_cmd_mps=vel_cmd_mps,
                vel_lsb=vel_lsb,
                match_enter_th_n=match_enter_th_n,
                match_exit_th_n=match_exit_th_n,
            )

            pending_updates.append({
                "id": i,
                "goal_new": goal_new,
                "should_write": should_write,
                "x_now_m": x_now_m,
                "F_target": F_target,
                "force_error": force_error,
                "error_state": error_state,
                "frozen_active": prev_frozen_active,
                "err_mag": err_mag,
                "dmag": dmag,
                "d_err": d_err,
                "f_measured": F_measured,
                "f_target_mode": f_target_mode,
                "vel_cmd_mps": vel_cmd_mps,
                "vel_lsb": vel_lsb,
            })

        written_ids = set()
        if pending_goal_map:
            try:
                if use_velocity_output:
                    for dxl_id, vel_lsb in pending_goal_map.items():
                        write_goal_velocity(dxl_id, vel_lsb)
                else:
                    sync_write_goal_positions(pending_goal_map)
                written_ids = set(pending_goal_map.keys())
            except Exception as e:
                debug(str(e))

        with lock:
            for item in pending_updates:
                i = item["id"]
                if (not use_velocity_output) and item["should_write"] and i in written_ids:
                    goal_tick_by_id[i] = item["goal_new"]
                vel_cmd_mps_by_id[i] = item["vel_cmd_mps"] if i in written_ids else 0.0

                x_now_m_by_id[i] = item["x_now_m"]
                F_target_by_id[i] = item["F_target"]
                force_error_by_id[i] = item["force_error"]
                error_state_by_id[i] = item["error_state"]
                f_target_frozen_by_id[i] = item["frozen_active"]
                prev_force_error_mag_by_id[i] = item["err_mag"]
                prev_force_error_dmag_by_id[i] = item["dmag"]
                prev_force_error_by_id[i] = item["force_error"]
                prev_force_error_filt_by_id[i] = item["d_err"]
                prev_error_time_by_id[i] = t_now
                prev_x_now_m_by_id[i] = item["x_now_m"]
                prev_x_time_by_id[i] = t_now
                x_ref_m_by_id[i] = 0.0
                v_ref_mps_by_id[i] = 0.0

                if i == DEBUG_HX_ID:
                    append_debug_sample(
                        t_now=t_now,
                        x_m=item["x_now_m"],
                        f_measured=item["f_measured"],
                        f_target=item["F_target"],
                        force_error=item["force_error"],
                        state_str=signal_state[i],
                        error_state_str=error_state_by_id[i],
                        f_target_mode_str=item["f_target_mode"],
                    )

            # if i == DEBUG_HX_ID:
            #     print(
            #         f"{pos_now_tick}({trial_zero_pos.get(DEBUG_HX_ID, pos_now_tick)}), "
            #         f"x={x_now_m:.6f}, goal={goal_new}, Fm={F_measured:.4f}, Ft={F_target:.4f}"
            #     )

        compute_force_for_unity()
        compute_disp_for_unity()


# -----------------------------
def telemetry_sender_loop():
    global running, last_sent_force, last_sent_disp

    while running:
        if not ENABLE_UNITY_TELEMETRY:
            time.sleep(0.03)
            continue

        with lock:
            fval = round(float(latest_force_for_unity), 3)
            xval = round(float(latest_disp_for_unity), 5)

        if last_sent_force is None or fval != last_sent_force:
            send_stdout(f"F,{fval:.3f}")
            last_sent_force = fval

        if last_sent_disp is None or xval != last_sent_disp:
            send_stdout(f"X,{xval:.5f}")
            last_sent_disp = xval

        time.sleep(0.03)


# -----------------------------
def handle_B():
    global boundary_ready, render_enabled

    with lock:
        if not baseline_ready:
            debug("boundary1 not ready; press L first")
            return
        render_enabled = False

    try:
        set_torque_enable_many(DXL_IDS, False)
    except Exception as e:
        debug(f"torque off before boundary failed: {e}")
        return

    time.sleep(DT_LOOP * 2)
    try:
        pos = sync_read_present_positions(DXL_IDS)
    except Exception as e:
        debug(f"boundary read failed: {e}")
        return
    debug(f"BOUNDARY POS = {pos}")

    with lock:
        boundary_pos.clear()
        boundary_pos.update(pos)
        boundary_ready = True

    debug("boundary2 recorded; future stiffness trials will start from this position")


def handle_L():
    global baseline_ready, render_enabled

    with lock:
        render_enabled = False

    try:
        set_torque_enable_many(DXL_IDS, False)
    except Exception as e:
        debug(f"torque off before baseline failed: {e}")
        return

    time.sleep(DT_LOOP * 2)
    try:
        pos = sync_read_present_positions(DXL_IDS)
    except Exception as e:
        debug(f"baseline read failed: {e}")
        return
    debug(f"BASELINE POS = {pos}")

    with lock:
        baseline_pos.clear()
        baseline_pos.update(pos)
        baseline_ready = True

    debug("boundary1 recorded")


def handle_V():
    global render_enabled, current_trial, latest_force_for_unity, latest_disp_for_unity, trial_zero_ready
    use_velocity_output = str(STIFFNESS_PID_OUTPUT_MODE).lower() == "velocity"

    with lock:
        if not boundary_ready:
            debug("boundary not ready; press B first")
            return

        render_enabled = False
        current_trial = None

        active_ids.clear()
        active_k_by_id.clear()
        active_b_by_id.clear()
        active_kp_by_id.clear()
        active_kd_by_id.clear()
        active_controller_by_id.clear()
        active_adm_m_by_id.clear()
        active_adm_b_by_id.clear()
        active_adm_k_by_id.clear()
        active_adm_vmax_by_id.clear()
        active_adm_xmax_by_id.clear()
        active_simple_push_gain_by_id.clear()
        active_simple_release_gain_by_id.clear()
        active_simple_push_max_by_id.clear()
        active_simple_release_max_by_id.clear()
        active_simple_err_full_scale_by_id.clear()
        active_profile_v_by_id.clear()
        active_profile_accel_by_id.clear()

        target_pos = dict(boundary_pos)

    time.sleep(DT_LOOP * 2)

    if not use_velocity_output:
        try:
            move_all_to_positions(target_pos, settle=0.5)
        except Exception as e:
            debug(f"move to boundary failed: {e}")
            return
    else:
        stop_velocity_many(DXL_IDS)

    with lock:
        for i in DXL_IDS:
            if i in target_pos:
                goal_tick_by_id[i] = int(target_pos[i])

                prev_force_error_by_id[i] = 0.0
                prev_force_error_filt_by_id[i] = 0.0
                prev_error_time_by_id[i] = time.time()
                prev_x_now_m_by_id[i] = 0.0
                prev_x_time_by_id[i] = time.time()
                x_now_m_by_id[i] = 0.0
                x_ref_m_by_id[i] = 0.0
                v_ref_mps_by_id[i] = 0.0
                F_target_by_id[i] = 0.0
                force_error_by_id[i] = 0.0
                error_state_by_id[i] = "MATCH"
                f_target_frozen_by_id[i] = False
                error_state_candidate_by_id[i] = None
                error_state_candidate_count_by_id[i] = 0
                prev_force_error_mag_by_id[i] = 0.0
                prev_force_error_dmag_by_id[i] = 0.0
                error_state_match_hold_until_by_id[i] = 0.0

        latest_force_for_unity = 0.0
        latest_disp_for_unity = 0.0
        trial_zero_ready = False

    debug("moved back to boundary position")
    send_stdout("K")


def handle_N():
    global render_enabled
    use_velocity_output = str(STIFFNESS_PID_OUTPUT_MODE).lower() == "velocity"

    with lock:
        if not baseline_ready:
            debug("baseline not ready; press L first")
            return
        render_enabled = False

        pos = {}
        for i in DXL_IDS:
            if i not in baseline_pos:
                continue
            delta_tick = int(round(float(INTI_DEG_STIFF.get(i, 0.0)) * TICKS_PER_DEG))
            pos[i] = int(baseline_pos[i]) + delta_tick

        print("BOUNDARY2 TARGET POS =", pos)

    set_torque_enable_many(DXL_IDS, True)
    if use_velocity_output:
        try:
            move_all_to_positions_velocity(pos)
        except Exception as e:
            debug(f"move to initial stiffness pose failed: {e}")
            return
    else:
        move_all_to_positions(pos)

    with lock:
        for i in DXL_IDS:
            if i in pos:
                goal_tick_by_id[i] = int(pos[i])

    send_stdout("K")


def handle_trial(code):
    global render_enabled, current_trial, trial_zero_ready
    use_velocity_output = str(STIFFNESS_PID_OUTPUT_MODE).lower() == "velocity"

    if not boundary_ready:
        debug("boundary2 not ready; press B first")
        return
    if not baseline_ready:
        debug("boundary1 not ready; press L first")
        return

    try:
        (
            ids,
            k_by_id,
            b_by_id,
            kp_by_id,
            kd_by_id,
            controller_by_id,
            adm_m_by_id,
            adm_b_by_id,
            adm_k_by_id,
            adm_vmax_by_id,
            adm_xmax_by_id,
            simple_push_gain_by_id,
            simple_release_gain_by_id,
            simple_push_max_by_id,
            simple_release_max_by_id,
            simple_err_full_scale_by_id,
            profile_v_by_id,
            profile_accel_by_id,
        ) = expand_command_to_motor_params(code)
    except Exception as e:
        debug(f"expand_command_to_motor_params failed: {e}")
        return

    if not ids:
        debug(f"unknown trial command: {code}")
        return

    try:
        set_torque_enable_many(ids, True)
    except Exception as e:
        debug(f"torque on before trial failed: {e}")
        return

    for i in ids:
        try:
            set_motor_profile(
                i,
                profile_v=profile_v_by_id.get(i),
                profile_accel=profile_accel_by_id.get(i),
            )
        except Exception as e:
            debug(f"[PROFILE] ID{i} set failed: {e}")
            return

    if not use_velocity_output:
        try:
            start_pos = {i: int(boundary_pos[i]) for i in ids if i in boundary_pos}
            if len(start_pos) != len(ids):
                missing_ids = [i for i in ids if i not in start_pos]
                debug(f"boundary2 missing IDs for trial start: {missing_ids}")
                return
            move_all_to_positions(
                start_pos,
                settle=0.5,
                profile_v_map=profile_v_by_id,
                profile_accel_map=profile_accel_by_id,
            )
        except Exception as e:
            debug(f"move to boundary2 failed before trial: {e}")
            return
    else:
        stop_velocity_many(ids)

    try:
        zero_map = sync_read_present_positions(ids)
    except Exception as e:
        debug(f"trial zero read failed: {e}")
        return

    # 執行 HX711 零點校正
    zero_hx_sensors_for_trial(ids)

    with lock:
        current_trial = code

        active_ids.clear()
        active_ids.extend(ids)

        active_k_by_id.clear()
        active_k_by_id.update(k_by_id)

        active_b_by_id.clear()
        active_b_by_id.update(b_by_id)

        active_kp_by_id.clear()
        active_kp_by_id.update(kp_by_id)

        active_kd_by_id.clear()
        active_kd_by_id.update(kd_by_id)

        active_controller_by_id.clear()
        active_controller_by_id.update(controller_by_id)

        active_adm_m_by_id.clear()
        active_adm_m_by_id.update(adm_m_by_id)

        active_adm_b_by_id.clear()
        active_adm_b_by_id.update(adm_b_by_id)

        active_adm_k_by_id.clear()
        active_adm_k_by_id.update(adm_k_by_id)

        active_adm_vmax_by_id.clear()
        active_adm_vmax_by_id.update(adm_vmax_by_id)

        active_adm_xmax_by_id.clear()
        active_adm_xmax_by_id.update(adm_xmax_by_id)

        active_simple_push_gain_by_id.clear()
        active_simple_push_gain_by_id.update(simple_push_gain_by_id)

        active_simple_release_gain_by_id.clear()
        active_simple_release_gain_by_id.update(simple_release_gain_by_id)

        active_simple_push_max_by_id.clear()
        active_simple_push_max_by_id.update(simple_push_max_by_id)

        active_simple_release_max_by_id.clear()
        active_simple_release_max_by_id.update(simple_release_max_by_id)

        active_simple_err_full_scale_by_id.clear()
        active_simple_err_full_scale_by_id.update(simple_err_full_scale_by_id)

        active_profile_v_by_id.clear()
        active_profile_v_by_id.update(profile_v_by_id)

        active_profile_accel_by_id.clear()
        active_profile_accel_by_id.update(profile_accel_by_id)

        for i in ids:
            trial_zero_pos[i] = int(zero_map[i])
            goal_tick_by_id[i] = int(zero_map[i])

            prev_force_error_by_id[i] = 0.0
            prev_force_error_filt_by_id[i] = 0.0
            prev_error_time_by_id[i] = time.time()
            prev_x_now_m_by_id[i] = 0.0
            prev_x_time_by_id[i] = time.time()
            x_ref_m_by_id[i] = 0.0
            v_ref_mps_by_id[i] = 0.0
            error_state_by_id[i] = "MATCH"
            f_target_frozen_by_id[i] = False
            error_state_candidate_by_id[i] = None
            error_state_candidate_count_by_id[i] = 0
            prev_force_error_mag_by_id[i] = 0.0
            prev_force_error_dmag_by_id[i] = 0.0
            error_state_match_hold_until_by_id[i] = 0.0

        trial_zero_ready = True
        render_enabled = True

    compute_force_for_unity()
    compute_disp_for_unity()
    debug(f"trial {code} started from boundary2; HX711 zeroed; ids={ids}")


def handle_M():
    global render_enabled, current_trial, latest_force_for_unity, latest_disp_for_unity, trial_zero_ready
    use_velocity_output = str(STIFFNESS_PID_OUTPUT_MODE).lower() == "velocity"

    with lock:
        render_enabled = False
        current_trial = None

        active_ids.clear()
        active_k_by_id.clear()
        active_b_by_id.clear()
        active_kp_by_id.clear()
        active_kd_by_id.clear()
        active_controller_by_id.clear()
        active_adm_m_by_id.clear()
        active_adm_b_by_id.clear()
        active_adm_k_by_id.clear()
        active_adm_vmax_by_id.clear()
        active_adm_xmax_by_id.clear()
        active_simple_push_gain_by_id.clear()
        active_simple_release_gain_by_id.clear()
        active_simple_push_max_by_id.clear()
        active_simple_release_max_by_id.clear()
        active_simple_err_full_scale_by_id.clear()
        active_profile_v_by_id.clear()
        active_profile_accel_by_id.clear()

    time.sleep(DT_LOOP * 2)

    if use_velocity_output:
        stop_velocity_many(DXL_IDS)
    elif baseline_ready:
        set_torque_enable_many(DXL_IDS, True)
        move_all_to_positions(dict(baseline_pos), 0.5)
        with lock:
            for i in DXL_IDS:
                if i in baseline_pos:
                    goal_tick_by_id[i] = int(baseline_pos[i])

    with lock:
        latest_force_for_unity = 0.0
        latest_disp_for_unity = 0.0
        trial_zero_ready = False

    send_stdout("K")


def handle_Z():
    """手動零點校正（僅針對當前啟用的馬達或所有馬達）"""
    global hx_zero_offset_by_id

    with lock:
        if active_ids:
            # 如果有進行中的 trial，只校正這些馬達
            ids_to_zero = list(active_ids)
        else:
            # 否則校正所有馬達
            ids_to_zero = list(DXL_IDS)

    debug(f"[ZERO] Manual zeroing triggered for IDs: {ids_to_zero}")
    zero_hx_sensors_for_trial(ids_to_zero)
    send_stdout("K")


def handle_Q():
    global running
    running = False


# -----------------------------
def stdin_loop():
    global running

    while running:
        line = sys.stdin.readline()
        if line == "":
            break

        cmd = line.strip()

        if cmd in ("B", "b"):
            handle_B()
        elif cmd in ("L", "l"):
            handle_L()
        elif cmd in ("V", "v"):
            handle_V()
        elif cmd in ("N", "n"):
            handle_N()
        elif cmd in ("M", "m"):
            handle_M()
        elif cmd in ("Z", "z"):
            handle_Z()
        elif cmd.lower() in (
            "epc", "epm", "eps",
            "presetc", "presetm", "presets",
            "errc", "errm", "errs",
            "conservative", "medium", "sensitive",
        ) or cmd.lower().startswith(("ep ", "preset ", "errpreset ")):
            out = handle_error_state_preset(cmd)
            if out:
                send_stdout(out)
        elif cmd in ("Q", "q"):
            handle_Q()
        elif cmd in STIFFNESS_COMMAND_SPECS:
            handle_trial(cmd)
        else:
            debug("unknown cmd " + cmd)

    running = False


# -----------------------------
def main():
    global pkt, port, group_sync_read_pos, group_sync_write_goal, running

    port = PortHandler(DEVICENAME)
    pkt = PacketHandler(PROTOCOL)

    assert port.openPort(), "openPort failed"
    assert port.setBaudRate(BAUDRATE), "setBaudRate failed"
    group_sync_read_pos = GroupSyncRead(port, pkt, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    group_sync_write_goal = GroupSyncWrite(port, pkt, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

    # # ---------- 上限校正 ----------
    # if os.path.exists(LIMIT_JSON_PATH):
    #     load_manual_upper_limits()
    #     debug("[LIMIT] using existing manual upper limits")
    # else:
    #     calibrate_manual_upper_limits()

    init_dynamixel()

    threading.Thread(target=hx711_thread_csv, daemon=True).start()
    threading.Thread(target=render_loop, daemon=True).start()
    threading.Thread(target=telemetry_sender_loop, daemon=True).start()
    stdin_thread = None

    if ENABLE_DEBUG_PLOT and MATPLOTLIB_OK:
        stdin_thread = threading.Thread(target=stdin_loop, daemon=True)
        stdin_thread.start()
        debug_plot_thread()
        stdin_thread.join(timeout=1.0)
    else:
        stdin_loop()

    try:
        stop_velocity_many(DXL_IDS)
        for i in DXL_IDS:
            with dxl_lock:
                pkt.write1ByteTxRx(port, i, ADDR_TORQUE_ENABLE, 0)
    except Exception:
        pass

    try:
        port.closePort()
    except Exception:
        pass


if __name__ == "__main__":
    main()
