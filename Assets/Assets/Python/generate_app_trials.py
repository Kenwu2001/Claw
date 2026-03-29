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
FORCE_LEVELS = [10, 20, 30]
N_REPETITIONS = 2

OUTPUT_DIR = "force_trials"
ALL_SUBJECTS_CSV = "force_trial_table_all_subjects.csv"

# 固定 seed 可重現；若不想固定可改成 None
RANDOM_SEED = 42

DEFAULT_BEST = ""
DEFAULT_LOWERBOUND = ""
DEFAULT_UPPERBOUND = ""
DEFAULT_DURATION = ""

if RANDOM_SEED is not None:
    random.seed(RANDOM_SEED)

os.makedirs(OUTPUT_DIR, exist_ok=True)

# =========================================================
# COUNTERBALANCE DESIGN
# =========================================================
# 3 methods -> 6 permutations
method_orders = list(permutations(METHODS))

# object-order templates across 3 method blocks
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

# 6 method orders x 2 templates = 12 subject plans
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
# HELPERS
# =========================================================
def fill_angles_by_method(method, force_level):
    """
    根據 method 決定 ab_ad_angle / ex_angle 填法
    """
    if method == "inter-finger only":
        return force_level, ""
    elif method == "extension only":
        return "", force_level
    elif method == "both":
        return force_level, force_level
    else:
        raise ValueError(f"Unknown method: {method}")

def write_csv(filepath, rows, fieldnames):
    with open(filepath, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

def generate_subject_rows(user_id, plan):
    rows = []
    trial_no = 1
    global_block = 1

    method_order = plan["method_order"]
    object_order_template = plan["object_order_template"]

    for method_block_idx, method in enumerate(method_order):
        object_order_this_block = object_order_template[method_block_idx]

        for object_block_idx, obj_type in enumerate(object_order_this_block, start=1):
            for repetition in range(1, N_REPETITIONS + 1):
                force_list = FORCE_LEVELS.copy()
                random.shuffle(force_list)

                for force_level in force_list:
                    ab_ad_angle, ex_angle = fill_angles_by_method(method, force_level)

                    row = {
                        "UserID": user_id,
                        "Trial_no": trial_no,
                        "ab_ad_angle": ab_ad_angle,
                        "ex_angle": ex_angle,
                        "Method": method,
                        "ObjectType": obj_type,
                        "Block": global_block,
                        "best": DEFAULT_BEST,
                        "lowerbound": DEFAULT_LOWERBOUND,
                        "upperbound": DEFAULT_UPPERBOUND,
                        "duration": DEFAULT_DURATION,

                        # 額外欄位，方便檢查/分析
                        "MethodBlock": method_block_idx + 1,
                        "ObjectBlock": object_block_idx,
                        "Repetition": repetition,
                        "ForceLevel": force_level
                    }

                    rows.append(row)
                    trial_no += 1

            # 一個 method × object 完成後 block+1
            global_block += 1

    return rows

# =========================================================
# GENERATE
# =========================================================
fieldnames = [
    "UserID",
    "Trial_no",
    "ab_ad_angle",
    "ex_angle",
    "Method",
    "ObjectType",
    "Block",
    "best",
    "lowerbound",
    "upperbound",
    "duration",
    "MethodBlock",
    "ObjectBlock",
    "Repetition",
    "ForceLevel"
]

all_rows = []

for user_id in range(1, N_USERS + 1):
    plan = subject_plans[user_id - 1]
    subject_rows = generate_subject_rows(user_id, plan)

    individual_name = f"user_{user_id}_force_trials.csv"
    individual_path = os.path.join(OUTPUT_DIR, individual_name)
    write_csv(individual_path, subject_rows, fieldnames)

    all_rows.extend(subject_rows)

all_path = os.path.join(OUTPUT_DIR, ALL_SUBJECTS_CSV)
write_csv(all_path, all_rows, fieldnames)

# =========================================================
# SUMMARY
# =========================================================
print("Done.")
print(f"Output folder: {OUTPUT_DIR}")
print(f"All-subject CSV: {all_path}")
print(f"Total rows: {len(all_rows)}")
print(f"Trials per subject: {len(all_rows) // N_USERS}")

print("\nGenerated individual files:")
for user_id in range(1, N_USERS + 1):
    print(f"  user_{user_id}_force_trials.csv")

print("\n=== Subject assignment summary ===")
for user_id in range(1, N_USERS + 1):
    plan = subject_plans[user_id - 1]
    print(f"\nUser {user_id}")
    print("  Method order:", " -> ".join(plan["method_order"]))
    print("  Object template by method block:")
    for i, oo in enumerate(plan["object_order_template"], start=1):
        print(f"    MethodBlock {i}: {oo[0]} -> {oo[1]}")