import csv
import os
import random

# =========================================================
# SETTINGS
# =========================================================
N_USERS = 12

METHOD_A = "inter-finger only"
METHOD_B = "extension only"
METHOD_LAST = "both"

OBJECT_TYPES = ["virtual", "real hard ball", "real soft ball"]
STIFFNESS_LEVELS = ["s", "m", "h"]
N_REPETITIONS = 2

OUTPUT_DIR = "trial_stiffness"
ALL_SUBJECTS_CSV = "trial_table_stiffness.csv"

DEFAULT_BEST = ""
DEFAULT_LOWERBOUND = ""
DEFAULT_UPPERBOUND = ""
DEFAULT_DURATION = ""

os.makedirs(OUTPUT_DIR, exist_ok=True)

# =========================================================
# METHOD ORDER (deterministic)
# =========================================================
def get_method_order_for_user(user_id):
    if user_id % 2 == 1:
        return [METHOD_A, METHOD_B, METHOD_LAST]
    else:
        return [METHOD_B, METHOD_A, METHOD_LAST]

# =========================================================
# OBJECT ORDER (6 permutations, deterministic)
# =========================================================
OBJECT_PERMUTATIONS = [
    ["virtual", "real hard ball", "real soft ball"],
    ["virtual", "real soft ball", "real hard ball"],
    ["real hard ball", "virtual", "real soft ball"],
    ["real hard ball", "real soft ball", "virtual"],
    ["real soft ball", "virtual", "real hard ball"],
    ["real soft ball", "real hard ball", "virtual"],
]

def get_object_order_template_for_user(user_id):
    start_idx = (user_id - 1) % len(OBJECT_PERMUTATIONS)
    template = []
    for offset in range(3):
        idx = (start_idx + offset) % len(OBJECT_PERMUTATIONS)
        template.append(OBJECT_PERMUTATIONS[idx])
    return template

# =========================================================
# ⭐ PSEUDO-RANDOM STIFFNESS ORDER
# =========================================================
def get_stiffness_order(user_id, method_block, object_block, repetition):
    """
    s/m/h 隨機排列，但 deterministic
    """
    seed = user_id * 1000 + method_block * 100 + object_block * 10 + repetition
    rng = random.Random(seed)
    k_list = STIFFNESS_LEVELS.copy()
    rng.shuffle(k_list)
    return k_list

# =========================================================
# CSV helpers
# =========================================================
def write_csv(filepath, rows, fieldnames):
    with open(filepath, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

# =========================================================
# GENERATE SUBJECT TRIALS
# =========================================================
def generate_subject_rows(user_id):
    rows = []
    trial_no = 1
    global_block = 1

    method_order = get_method_order_for_user(user_id)
    object_template = get_object_order_template_for_user(user_id)

    for method_block_idx, method in enumerate(method_order, start=1):
        object_order = object_template[method_block_idx - 1]

        for object_block_idx, obj in enumerate(object_order, start=1):

            for repetition in range(1, N_REPETITIONS + 1):

                k_list = get_stiffness_order(
                    user_id,
                    method_block_idx,
                    object_block_idx,
                    repetition
                )

                for k in k_list:

                    rows.append({
                        "UserID": user_id,
                        "Trial_no": trial_no,
                        "k": k,
                        "Method": method,
                        "ObjectType": obj,
                        "Repitition": repetition,
                        "Block": global_block,
                        "best": DEFAULT_BEST,
                        "lowerbound": DEFAULT_LOWERBOUND,
                        "upperbound": DEFAULT_UPPERBOUND,
                        "duration": DEFAULT_DURATION,
                        "MethodBlock": method_block_idx,
                        "ObjectBlock": object_block_idx
                    })

                    trial_no += 1

            global_block += 1

    return rows

# =========================================================
# MAIN GENERATION
# =========================================================
fieldnames = [
    "UserID",
    "Trial_no",
    "k",
    "Method",
    "ObjectType",
    "Repitition",
    "Block",
    "best",
    "lowerbound",
    "upperbound",
    "duration",
    "MethodBlock",
    "ObjectBlock"
]

all_rows = []

for user_id in range(1, N_USERS + 1):

    subject_rows = generate_subject_rows(user_id)

    individual_path = os.path.join(
        OUTPUT_DIR,
        f"user_{user_id:02d}_stiffness_trials.csv"
    )

    write_csv(individual_path, subject_rows, fieldnames)
    all_rows.extend(subject_rows)

all_path = os.path.join(OUTPUT_DIR, ALL_SUBJECTS_CSV)
write_csv(all_path, all_rows, fieldnames)

# =========================================================
# SUMMARY
# =========================================================
print("Done.")
print(f"Output folder: {OUTPUT_DIR}")
print(f"Total rows: {len(all_rows)}")
print(f"Trials per subject: {len(all_rows)//N_USERS}")

print("\nSubject assignment summary:")

for user_id in range(1, N_USERS + 1):

    method_order = get_method_order_for_user(user_id)
    object_template = get_object_order_template_for_user(user_id)

    print(f"\nUser {user_id}")
    print("  Method order:", " -> ".join(method_order))
    print("  Object order per method block:")

    for i, oo in enumerate(object_template, start=1):
        print(f"    Block {i}: {' -> '.join(oo)}")