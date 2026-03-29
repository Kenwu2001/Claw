import csv
import os
import random
from itertools import permutations

# =========================================================
# SETTINGS
# =========================================================
N_USERS = 12

METHODS = ["inter-finger only", "extension only", "both"]
OBJECT_TYPES = ["virtual", "real"]
ANGLE_LEVELS = [10, 20, -10, -20]
N_REPETITIONS = 2

# output folder
OUTPUT_DIR = "trial_size"

# output filenames
ALL_SUBJECTS_CSV = "trial_table_size.csv"

# random seed
# 固定 seed 可重現相同結果；改成 None 則每次都不同
RANDOM_SEED = 42

# 其他欄位預設值（可自行改）
DEFAULT_LOWERBOUND = ""
DEFAULT_UPPERBOUND = ""
DEFAULT_DURATION = ""

if RANDOM_SEED is not None:
    random.seed(RANDOM_SEED)

# 建立輸出資料夾
os.makedirs(OUTPUT_DIR, exist_ok=True)

# =========================================================
# COUNTERBALANCE DESIGN
# =========================================================
# 3 methods -> 3! = 6 orders
method_orders = list(permutations(METHODS))

# object-order templates across the 3 method blocks
# Template 1: VR / RV / VR
# Template 2: RV / VR / RV
object_order_templates = [
    [
        ["virtual", "real"],
        ["real", "virtual"],
        ["virtual", "real"]
    ],
    [
        ["real", "virtual"],
        ["virtual", "real"],
        ["real", "virtual"]
    ]
]

# 6 method orders x 2 object templates = 12 subject plans
subject_plans = []
for mo in method_orders:
    for oot in object_order_templates:
        subject_plans.append({
            "method_order": list(mo),
            "object_order_template": oot
        })

# 打散受試者分配順序
random.shuffle(subject_plans)

# =========================================================
# HELPER FUNCTIONS
# =========================================================
def fill_angles_by_method(method, angle):
    """
    根據 method 決定 ab_ad_angle / ex_angle 的填法
    """
    if method == "inter-finger only":
        return angle, ""
    elif method == "extension only":
        return "", angle
    elif method == "both":
        return angle, angle
    else:
        raise ValueError(f"Unknown method: {method}")

def generate_subject_rows(user_id, plan):
    """
    依據單一受試者的 counterbalance plan 產生該受試者所有 trial rows
    """
    method_order = plan["method_order"]
    object_order_template = plan["object_order_template"]

    rows = []
    trial_no = 1
    global_block = 1

    for method_block_idx, method in enumerate(method_order):
        # 根據第幾個 method block，決定 object order
        object_order_this_block = object_order_template[method_block_idx]

        for object_block_idx, obj_type in enumerate(object_order_this_block, start=1):
            for repetition in range(1, N_REPETITIONS + 1):
                angle_list = ANGLE_LEVELS.copy()
                random.shuffle(angle_list)

                for angle in angle_list:
                    ab_ad_angle, ex_angle = fill_angles_by_method(method, angle)

                    row = {
                        "UserID": user_id,
                        "Trial_no": trial_no,
                        "ab_ad_angle": ab_ad_angle,
                        "ex_angle": ex_angle,
                        "Method": method,
                        "ObjectType": obj_type,
                        "Block": global_block,
                        "Size Change(%)": angle,
                        "lowerbound": DEFAULT_LOWERBOUND,
                        "upperbound": DEFAULT_UPPERBOUND,
                        "duration": DEFAULT_DURATION,
                        "MethodBlock": method_block_idx + 1,
                        "ObjectBlock": object_block_idx,
                        "Repetition": repetition,
                        "AngleChange": angle
                    }

                    rows.append(row)
                    trial_no += 1

            global_block += 1

    return rows

def write_csv(filepath, rows, fieldnames):
    with open(filepath, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

# =========================================================
# GENERATE ALL DATA
# =========================================================
fieldnames = [
    "UserID",
    "Trial_no",
    "ab_ad_angle",
    "ex_angle",
    "Method",
    "ObjectType",
    "Block",
    "Size Change(%)",
    "lowerbound",
    "upperbound",
    "duration",
    "MethodBlock",
    "ObjectBlock",
    "Repetition",
    "AngleChange"
]

all_rows = []

for user_id in range(1, N_USERS + 1):
    plan = subject_plans[user_id - 1]
    subject_rows = generate_subject_rows(user_id, plan)

    # 寫出 individual subject csv
    subject_filename = f"Trial_Size_{user_id}.csv"
    subject_filepath = os.path.join(OUTPUT_DIR, subject_filename)
    write_csv(subject_filepath, subject_rows, fieldnames)

    all_rows.extend(subject_rows)

# 寫出總表
all_subjects_filepath = os.path.join(OUTPUT_DIR, ALL_SUBJECTS_CSV)
write_csv(all_subjects_filepath, all_rows, fieldnames)

# =========================================================
# SUMMARY
# =========================================================
print("Done.")
print(f"Output folder: {OUTPUT_DIR}")
print(f"All-subject CSV: {all_subjects_filepath}")
print(f"Total rows: {len(all_rows)}")
print(f"Trials per subject: {len(all_rows) // N_USERS}")

print("\nGenerated individual subject files:")
for user_id in range(1, N_USERS + 1):
    print(f"  user_{user_id:02d}_trials.csv")

print("\n=== Subject assignment summary ===")
for user_id in range(1, N_USERS + 1):
    plan = subject_plans[user_id - 1]
    print(f"\nUser {user_id}")
    print("  Method order:", " -> ".join(plan["method_order"]))
    print("  Object template by method block:")
    for i, oo in enumerate(plan["object_order_template"], start=1):
        print(f"    MethodBlock {i}: {oo[0]} -> {oo[1]}")