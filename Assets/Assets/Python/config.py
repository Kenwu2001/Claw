"""
Centralized configuration for ports and tunables.

Update (2026-01):
- PaXini: one sensor per serial port, but mixed generations/protocols:
    * 2x PaXini Gen2 streaming CSV lines (sensor_com16_reader.py style): Fz,Ft,theta_deg,sliding,slip_window_mm
    * 1x PaXini Gen1 using read_paxini.py (binary protocol): provides fx,fy,fz (we derive Fz,Ft,theta_deg; sliding/slip set to defaults)
- HX711: one serial port streaming 3 comma-separated values per line: f1,f2,f3
  (one measured force per motor loop)
"""

# =====================
# Serial ports
# =====================

# HX711 measured forces: single port streaming "f1,f2,f3" each line.
HX_PORT = "COM7"
HX_BAUD = 115200
HX_TIMEOUT = 0.05

# =====================
# Dynamixel
# =====================
DXL_BAUD     = 57600
DXL_DEVICE   = "COM5"
DXL_PROTOCOL = 2.0

# ---------------------
# Motor soft-limits / calibration
# ---------------------
# Per-motor (min_tick, max_tick) stored in JSON created by the calibration wizard.
# If you power-cycle/reboot the motor, XL330's Present Position reference can change;
# treat the saved limits as "session-to-session" hints unless you also apply homing.
MOTOR_LIMITS_FILE = "motor_limits.json"

# If True and limits exist, enforce software soft-limits during velocity control.
ENFORCE_SOFT_LIMITS = False

# Stop motion when within this margin (ticks) to a limit.
SOFT_LIMIT_MARGIN_TICKS = 20

abAdScale = 1.5689
exScale = 9.1618

# ENABLED_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9]
# ENABLED_IDS = [1, 2, 3, 4, 5]
ENABLED_IDS =[1, 6]


# Motor -> HX711 index mapping (0/1/2) from the "f1,f2,f3" line
DXL_HX_INDEX = {
    1: 0,
    2: 1,
    3: 6,
    4: 7,
    5: 8,
    6: 2,
    7: 3,
    8: 5,
    9: 4
}


# ====Initialization (relative to baseline), finger degree======
# === {motorID:degree}
INTI_DEG_SIZE = {
    1: 5*exScale,
    2: 20*exScale,
    3: 20*exScale,
    4: 20*exScale,
    5: 20*exScale,
    6: -20*abAdScale,
    7: -20*abAdScale,
    8: -20*abAdScale,
    9: -20*abAdScale
}

INTI_DEG_STIFF = {
    1: 5*exScale,
    2: 20*exScale,
    3: 20*exScale,
    4: 20*exScale,
    5: 20*exScale,
    6: -20*abAdScale,
    7: -20*abAdScale,
    8: -20*abAdScale,
    9: -20*abAdScale
}

INTI_DEG_FORCE = {
    1: 5*exScale,
    2: 5*exScale,
    3: 5*exScale,
    4: 5*exScale,
    5: 5*exScale,
    6: -10*abAdScale,
    7: -10*abAdScale,
    8: -10*abAdScale,
    9: -10*abAdScale
}

FIN_DEG_FORCE = {
    1: 25*exScale,
    2: 25*exScale,
    3: 25*exScale,
    4: 25*exScale,
    5: 25*exScale,
    6: -20*abAdScale,
    7: -20*abAdScale,
    8: -20*abAdScale,
    9: -20*abAdScale
}

SIZE_VB = 45
SIZE_VE = 100
APPL_VB = 500
APPL_VE = 1600

# =====================
# Admittance/match control gains
# =====================
DT_LOOP = 0.01


# Max number of comma-separated floats expected per HX711 line.
# The reader will accept fewer values per line (e.g. during testing).
HX_MAX_VALUES = 9

# =====================
# HX711 zeroing for stiffness trial start
# =====================
HX_ZERO_SAMPLES = 20          # 取幾筆平均當 baseline
HX_ZERO_SAMPLE_INTERVAL_S = 0.005
HX_ZERO_TIMEOUT_S = 1.0       # 最多等多久收齊樣本


# =====================
# Dynamixel: grouping for keyboard control (extended position mode)
# =====================
# You can freely edit these lists to match your wiring / test setup.
# GROUP_E_IDS = [1, 2, 3, 4, 5]
GROUP_E_IDS = [1]
GROUP_B_IDS = [6]
# GROUP_B_IDS = []

# Extended Position Mode (Operating Mode = 4)
DXL_TICKS_PER_REV = 4096  # XL330/X-series typical resolution
KEYBOARD_CURRENT_LIMIT_MA = 1500

# =====================
# Stiffness mode (virtual spring/damper in position mode)
# =====================
# Motors participating in stiffness mode
STIFFNESS_MODE_IDS = ENABLED_IDS

# PID output command mode:
# - "position": legacy behavior, write Goal Position increments per loop.
# - "velocity": keep PID/error logic but write Goal Velocity in velocity mode.
STIFFNESS_PID_OUTPUT_MODE = "velocity"

# Velocity-mode tunables (used when STIFFNESS_PID_OUTPUT_MODE == "velocity")
STIFFNESS_PID_VEL_LPF_ALPHA = 0.9
STIFFNESS_PID_VEL_MAX_MPS_BY_GROUP = {
    "b": 10.10,
    "e": 0.10,
}
STIFFNESS_E_GROUP_TARGET_FLOOR_VEL_LSB = 50

# 0: use all motors in STIFFNESS_MODE_IDS
# non-zero: stiffness mode only controls this motor, and keeps the others torque-off
STIFFNESS_SINGLE_MOTOR_ID = 0

STIFFNESS_FORCE_DEADBAND_N = 0.3
STIFFNESS_FORCE_LPF_ALPHA = 0.2
STIFFNESS_E_GROUP_MIN_FM_FOR_TARGET_FLOOR_N = 0.5
STIFFNESS_E_GROUP_TARGET_FLOOR_N = 0.3

# Geometry
STIFFNESS_SPOOL_RADIUS_M = 0.018
STIFFNESS_SPOOL_RADIUS_M_BY_GROUP = {
    "b": 0.018,
    "e": 0.005,
}

STIFFNESS_CURRENT_LIMIT_MA = 800

# Per-motor spring sign (+1 or -1)
STIFFNESS_SIGN_BY_ID = {
    1: -1,
    2: -1,
    3: -1,
    4: -1,
    5: -1,
    6: -1,
    7: -1,
    8: -1,
    9: -1
}

# goal_new = goal_prev + d_tick * (CONTROL_SIGN)
CONTROL_SIGN_BY_ID = {
    1: -1,
    2: -1,
    3: -1,
    4: -1,
    5: -1, 
    6: 1,
    7: 1,
    8: 1,
    9: 1,
}

# velocity stiffness
# CONTROL_SIGN_BY_ID = {
#     1: 1,
#     2: 1,
#     3: 1,
#     4: 1,
#     5: 1, 
#     6: 1,
#     7: 1,
#     8: 1,
#     9: 1,
# }

# =====================
# Stiffness mode: preset library + command mapping
# =====================
# 每個 preset 是一整包控制參數
# 改這裡就能改不同 group / 軟硬度條件
STIFFNESS_PRESETS = {
    # ----- B group -----
    "bs": {
        "k": 220.0,
        "b": 25.0,
        "kp": 0.0095,
        "kd": 0.00015,
        "profile_v": 45,
        "profile_accel": 6,
        # "k": 220.0,
        # "b": 30.0,
        # "kp": 0.0025,
        # "kd": 0.0002,
    },
    "bm": {
        "k": 700.0,
        "b": 40.0,
        "kp": 0.0075,
        "kd": 0.00025,
        "profile_v": 20,
        "profile_accel": 6,
        # "k": 520.0,
        # "b": 0.0,
        # "kp": 0.0035,
        # "kd": 0.0003,
    },
    "bh": {
        "k": 1500.0,
        "b": 60.0,
        "kp": 0.0110,
        "kd": 0.00040,
        "profile_v": 20,
        "profile_accel": 6,
        # "k": 1100.0,
        # "b": 0.0,
        # "kp": 0.0050,
        # "kd": 0.0004,
    },

    # ----- E group -----
    "es": {
        "k": 180.0,
        "b": 35.0,
        "kp": 0.0060,
        "kd": 0.00020,
        "profile_v": 90,
        "profile_accel": 8,
        # "kp": 0.010,
        # "kd": 0.0003,
    },
    "em": {
        "k":500.0,
        "b": 55.0,
        "kp": 0.005,
        "kd": 0.00035,
        "profile_v": 75,
        "profile_accel": 10,
        # "kp": 0.016,
        # "kd": 0.0005,
    },
    "eh": {
        "k": 1000.0,
        "b": 80.0,
        "kp": 0.0150,
        "kd": 0.00055,
        "profile_v": 60,
        "profile_accel": 12,
        # "kp": 0.024,
        # "kd": 0.0008,
    },

    # ----- B group admittance -----
    "ba": {
        "controller": "admittance",
        "k": 500.0,
        "b": 60.0,
        "kp": 0.003,
        "kd": 0.0,
        "adm_m": 0.20,
        "adm_b": 25.0,
        "adm_k": 120.0,
        "adm_vmax": 0.05,
        "adm_xmax": 0.010,
        "profile_v": 20,
        "profile_accel": 8,
    },

    # ----- E group admittance -----
    "ea": {
        "controller": "admittance",
        "k": 800.0,
        "b": 80.0,
        "kp": 0.003,
        "kd": 0.0,
        "adm_m": 0.25,
        "adm_b": 30.0,
        "adm_k": 150.0,
        "adm_vmax": 0.06,
        "adm_xmax": 0.015,
        "profile_v": 20,
        "profile_accel": 8,
    },

    # ----- Simple admittance -----
    # Minimal loop:
    #   1) read encoder displacement x
    #   2) compute virtual spring force Ft = k*x
    #   3) compare Ft with measured force Fm
    #   4) if Ft > Fm, motor moves inward slowly
    #   5) if Ft < Fm, motor releases faster
    "bsa": {
        "controller": "simple_admittance",
        "k": 450.0,
        "b": 0.0,
        "kp": 0.0,
        "kd": 0.0,
        "simple_err_full_scale": 3.0,
        "simple_push_gain": 0.0005,
        "simple_release_gain": 0.0025,
        "simple_push_max": 0.003,
        "simple_release_max": 0.008,
        "profile_v": 15,
        "profile_accel": 6,
    },
    "esa": {
        "controller": "simple_admittance",
        "k": 700.0,
        "b": 0.0,
        "kp": 0.0,
        "kd": 0.0,
        "simple_err_full_scale": 3.0,
        "simple_push_gain": 0.0010,
        "simple_release_gain": 0.0025,
        "simple_push_max": 0.004,
        "simple_release_max": 0.010,
        "profile_v": 20,
        "profile_accel": 8,
    },

    # ----- Admittance stable-first (conservative) -----
    # Focus: lower oscillation risk, slower but smoother response.
    "bas": {
        "controller": "admittance",
        "k": 450.0,
        "b": 80.0,
        "kp": 0.0020,
        "kd": 0.0,
        "adm_m": 0.28,
        "adm_b": 40.0,
        "adm_k": 100.0,
        "adm_vmax": 0.030,
        "adm_xmax": 0.007,
        "profile_v": 12,
        "profile_accel": 5,
    },
    "eas": {
        "controller": "admittance",
        "k": 650.0,
        "b": 110.0,
        "kp": 0.0020,
        "kd": 0.0,
        "adm_m": 0.35,
        "adm_b": 50.0,
        "adm_k": 130.0,
        "adm_vmax": 0.035,
        "adm_xmax": 0.010,
        "profile_v": 12,
        "profile_accel": 5,
    },

    # ----- Admittance response-first -----
    # Focus: faster convergence and stronger reaction, higher oscillation risk.
    "bar": {
        "controller": "admittance",
        "k": 650.0,
        "b": 50.0,
        "kp": 0.0040,
        "kd": 0.0002,
        "adm_m": 0.16,
        "adm_b": 18.0,
        "adm_k": 170.0,
        "adm_vmax": 0.070,
        "adm_xmax": 0.016,
        "profile_v": 30,
        "profile_accel": 12,
    },
    "ear": {
        "controller": "admittance",
        "k": 1000.0,
        "b": 70.0,
        "kp": 0.0045,
        "kd": 0.0003,
        "adm_m": 0.20,
        "adm_b": 22.0,
        "adm_k": 210.0,
        "adm_vmax": 0.080,
        "adm_xmax": 0.020,
        "profile_v": 35,
        "profile_accel": 12,
    },
}

# trial command -> group 使用哪個 preset
STIFFNESS_COMMAND_SPECS = {
    # single group
    "bs": {"b": "bs"},
    "bm": {"b": "bm"},
    "bh": {"b": "bh"},

    "es": {"e": "es"},
    "em": {"e": "em"},
    "eh": {"e": "eh"},

    # combined
    "bses": {"b": "bs", "e": "es"},
    "bmem": {"b": "bm", "e": "em"},
    "bheh": {"b": "bh", "e": "eh"},

    # admittance mode
    "ba": {"b": "ba"},
    "ea": {"e": "ea"},
    "baea": {"b": "ba", "e": "ea"},

    # simple admittance mode
    "bsa": {"b": "bsa"},
    "esa": {"e": "esa"},
    "bsaesa": {"b": "bsa", "e": "esa"},

    # admittance stable-first
    "bas": {"b": "bas"},
    "eas": {"e": "eas"},
    "base": {"b": "bas", "e": "eas"},

    # admittance response-first
    "bar": {"b": "bar"},
    "ear": {"e": "ear"},
    "bare": {"b": "bar", "e": "ear"},
}

# 控制器限制：依 group 分開
STIFFNESS_DX_CMD_MAX_M_BY_GROUP = {
    "b": 0.5,
    "e": 0.5,
}

STIFFNESS_DX_CMD_PUSH_MAX_M_BY_GROUP = {
    "b": 0.0035,
    "e": 0.01,
}

# When measured force is falling and Ft > Fm, push back more aggressively.
# `min_err_scale` raises the minimum speed scaling even for small positive error.
# `max_boost` multiplies the normal push max, then still clamps by DX_CMD_MAX.
STIFFNESS_FALLING_PUSH_ERR_SCALE_MIN_BY_GROUP = {
    "b": 1.0,
    "e": 1.0,
}

STIFFNESS_FALLING_PUSH_MAX_BOOST_BY_GROUP = {
    "b": 2.5,
    "e": 2.0,
}

STIFFNESS_DX_CMD_RELEASE_MAX_M_BY_GROUP = {
    "b": 0.05,
    "e": 0.05,
}

# When the measured force is still rising (user is pressing in) and the
# controller wants to RELEASE, slow that release down so the mechanism does
# not give way too easily.
STIFFNESS_RISING_RELEASE_SLOWDOWN_BY_GROUP = {
    "b": 0.25,
    "e": 0.35,
}

# During active pressing (`signal_state == RISING`), keep the virtual target
# from running too far above the measured force. Smaller values feel harder
# to press because Ft is prevented from greatly exceeding Fm.
STIFFNESS_RISING_TARGET_POSITIVE_ERROR_CAP_N_BY_GROUP = {
    "b": 0.20,
    "e": 0.25,
}

STIFFNESS_SPEED_ERR_FULL_SCALE_N_BY_GROUP = {
    "b": 1.0,
    "e": 3.0,
}

STIFFNESS_FORCE_ERROR_DEADBAND_N_BY_GROUP = {
    "b": 0.5,
    "e": 0.5,
}

# MATCH threshold scheduling:
# threshold = max(min_n, base_n / (1 + abs(Ft) / ft_scale_n))
# so when virtual target force Ft gets larger, the allowed force error shrinks.
STIFFNESS_MATCH_ENTER_BASE_N_BY_GROUP = {
    "b": 0.20,
    "e": 0.20,
}

STIFFNESS_MATCH_ENTER_MIN_N_BY_GROUP = {
    "b": 0.05,
    "e": 0.05,
}

STIFFNESS_MATCH_ENTER_FT_SCALE_N_BY_GROUP = {
    "b": 3.0,
    "e": 3.0,
}

# True: threshold = max(min_n, base_n / (1 + abs(Ft) / ft_scale_n))
# False: threshold = base_n
STIFFNESS_MATCH_USE_DYNAMIC_THRESHOLD = False

STIFFNESS_MATCH_EXIT_BASE_N_BY_GROUP = {
    "b": 0.30,
    "e": 0.50,
}

STIFFNESS_MATCH_EXIT_MIN_N_BY_GROUP = {
    "b": 0.15,
    "e": 0.15,
}

STIFFNESS_MATCH_EXIT_FT_SCALE_N_BY_GROUP = {
    "b": 3.0,
    "e": 3.0,
}

# =====================
# Error-state hysteresis
# =====================
# Startup preset: "conservative" | "medium" | "sensitive"
ERROR_STATE_CONFIG_PRESET = "medium"

# Runtime-switchable preset table
ERROR_STATE_PRESETS = {
    "conservative": {
        "enter_abs": {"b": 0.35, "e": 0.35},
        "exit_abs": {"b": 0.90, "e": 0.90},
        "enter_dmag_max": {"b": 0.006, "e": 0.006},
        "exit_dmag_min": {"b": 0.030, "e": 0.030},
        "enter_confirm": 5,
        "exit_confirm": 2,
    },
    "medium": {
        "enter_abs": {"b": 0.45, "e": 0.45},
        "exit_abs": {"b": 0.70, "e": 0.70},
        "enter_dmag_max": {"b": 0.010, "e": 0.010},
        "exit_dmag_min": {"b": 0.020, "e": 0.020},
        "enter_confirm": 4,
        "exit_confirm": 2,
    },
    "sensitive": {
        "enter_abs": {"b": 1.20, "e": 0.80},
        "exit_abs": {"b": 1.60, "e": 1.10},
        "enter_dmag_max": {"b": 0.60, "e": 0.40},
        "exit_dmag_min": {"b": 0.25, "e": 0.18},
        "enter_confirm": 2,
        "exit_confirm": 1,
    },
}

# error_state uses force_error = Ft - Fm with hysteresis:
# - entering MATCH requires small error and a shrinking / flattening error magnitude
# - leaving MATCH requires error magnitude to grow again
# entering MATCH is intentionally harder than leaving it
#
# Recommended presets:
# Conservative
#   ERROR_STATE_MATCH_ENTER_ABS_N_BY_GROUP = {"b": 0.35, "e": 0.35}
#   ERROR_STATE_MATCH_EXIT_ABS_N_BY_GROUP = {"b": 0.90, "e": 0.90}
#   ERROR_STATE_MATCH_ENTER_DMAG_MAX_N_BY_GROUP = {"b": 0.006, "e": 0.006}
#   ERROR_STATE_MATCH_EXIT_DMAG_MIN_N_BY_GROUP = {"b": 0.030, "e": 0.030}
#   ERROR_STATE_MATCH_ENTER_CONFIRM = 5
#   ERROR_STATE_MATCH_EXIT_CONFIRM = 2
#
# Medium
#   ERROR_STATE_MATCH_ENTER_ABS_N_BY_GROUP = {"b": 0.45, "e": 0.45}
#   ERROR_STATE_MATCH_EXIT_ABS_N_BY_GROUP = {"b": 0.70, "e": 0.70}
#   ERROR_STATE_MATCH_ENTER_DMAG_MAX_N_BY_GROUP = {"b": 0.010, "e": 0.010}
#   ERROR_STATE_MATCH_EXIT_DMAG_MIN_N_BY_GROUP = {"b": 0.020, "e": 0.020}
#   ERROR_STATE_MATCH_ENTER_CONFIRM = 4
#   ERROR_STATE_MATCH_EXIT_CONFIRM = 2
#
# Sensitive
#   ERROR_STATE_MATCH_ENTER_ABS_N_BY_GROUP = {"b": 0.60, "e": 0.60}
#   ERROR_STATE_MATCH_EXIT_ABS_N_BY_GROUP = {"b": 0.75, "e": 0.75}
#   ERROR_STATE_MATCH_ENTER_DMAG_MAX_N_BY_GROUP = {"b": 0.018, "e": 0.018}
#   ERROR_STATE_MATCH_EXIT_DMAG_MIN_N_BY_GROUP = {"b": 0.012, "e": 0.012}
#   ERROR_STATE_MATCH_ENTER_CONFIRM = 2
#   ERROR_STATE_MATCH_EXIT_CONFIRM = 1
ERROR_STATE_MATCH_ENTER_ABS_N_BY_GROUP = dict(
    ERROR_STATE_PRESETS[ERROR_STATE_CONFIG_PRESET]["enter_abs"]
)

ERROR_STATE_MATCH_EXIT_ABS_N_BY_GROUP = dict(
    ERROR_STATE_PRESETS[ERROR_STATE_CONFIG_PRESET]["exit_abs"]
)

# Per-loop change of |force_error|, unit: N/sample
ERROR_STATE_MATCH_ENTER_DMAG_MAX_N_BY_GROUP = dict(
    ERROR_STATE_PRESETS[ERROR_STATE_CONFIG_PRESET]["enter_dmag_max"]
)

ERROR_STATE_MATCH_EXIT_DMAG_MIN_N_BY_GROUP = dict(
    ERROR_STATE_PRESETS[ERROR_STATE_CONFIG_PRESET]["exit_dmag_min"]
)

ERROR_STATE_MATCH_ENTER_CONFIRM = int(
    ERROR_STATE_PRESETS[ERROR_STATE_CONFIG_PRESET]["enter_confirm"]
)
ERROR_STATE_MATCH_EXIT_CONFIRM = int(
    ERROR_STATE_PRESETS[ERROR_STATE_CONFIG_PRESET]["exit_confirm"]
)

# Brief pause right after entering MATCH, so the motor settles before
# continuing with frozen-F_target control.
ERROR_STATE_MATCH_HOLD_SECONDS = 0.10

# =====================
# Gain scheduling (shared by PID and admittance modes)
# =====================
# Scale gains according to |force_error|.
# region thresholds: |e| < th_low => low, th_low~th_high => mid, >= th_high => high
ENABLE_GAIN_SCHEDULING = True

GAIN_SCHEDULE_THRESH_N_BY_GROUP = {
    "b": {"th_low": 1.0, "th_high": 3.0},
    "e": {"th_low": 1.0, "th_high": 3.0},
}

GAIN_SCHEDULE_SCALE_BY_GROUP = {
    "b": {"low": 0.70, "mid": 1.00, "high": 1.35},
    "e": {"low": 0.70, "mid": 1.00, "high": 1.35},
}

# Optional quick presets for gain scheduling policy:
# stable-first suggestion: low=0.60, mid=0.90, high=1.10
# response-first suggestion: low=0.85, mid=1.10, high=1.50
