"""motor_calibration.py

Interactive calibration for Dynamixel XL330 (and other X-series) **software limits**.

We ask the user to manually move each motor to its mechanical MIN and MAX, then
press Enter to record the corresponding Present Position ticks.

The resulting mapping is stored to JSON, e.g.:

    {"2": {"min": -1234, "max": 5678}, "3": {"min": ...}}

Notes
-----
* In Velocity Control Mode, built-in Min/Max Position Limit registers do not
  constrain motion, so we enforce the range in software.
* XL330's Present Position reference can be reset by some operations (notably
  switching into Position Control Mode + torque-on, and also after power-cycle).
  If you need a persistent absolute reference, consider adding a homing step.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Dict, Iterable, Tuple

from dynamixel_sdk import PortHandler, PacketHandler


# Control table addresses (X-series)
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132


@dataclass
class MotorLimit:
    min_tick: int
    max_tick: int

    def normalized(self) -> "MotorLimit":
        return MotorLimit(min(self.min_tick, self.max_tick), max(self.min_tick, self.max_tick))


def _read_present_position(pkt: PacketHandler, port: PortHandler, dxl_id: int) -> int:
    pos, dxl_comm_result, dxl_error = pkt.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    # SDK returns unsigned; interpret as signed 32-bit
    if pos >= 2**31:
        pos -= 2**32
    if dxl_comm_result != 0:
        raise RuntimeError(f"Read present position failed (id={dxl_id}): comm={dxl_comm_result}")
    if dxl_error != 0:
        # Not fatal for read in many cases, but better to surface during calibration.
        raise RuntimeError(f"Read present position failed (id={dxl_id}): error={dxl_error}")
    return int(pos)


def load_limits(path: str) -> Dict[int, MotorLimit]:
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    out: Dict[int, MotorLimit] = {}
    for k, v in raw.items():
        try:
            mid = int(k)
            out[mid] = MotorLimit(int(v["min"]), int(v["max"])).normalized()
        except Exception:
            continue
    return out


def save_limits(path: str, limits: Dict[int, MotorLimit]) -> None:
    raw = {str(mid): {"min": lim.min_tick, "max": lim.max_tick} for mid, lim in limits.items()}
    with open(path, "w", encoding="utf-8") as f:
        json.dump(raw, f, indent=2)


def calibrate_limits_interactive(
    pkt: PacketHandler,
    port: PortHandler,
    motor_ids: Iterable[int],
    save_path: str,
) -> Dict[int, MotorLimit]:
    """Run a terminal-based calibration wizard.

    Steps per motor:
    1) Torque OFF
    2) User manually moves motor to MIN mechanical stop -> press Enter -> record tick
    3) User manually moves motor to MAX mechanical stop -> press Enter -> record tick
    4) Keep torque OFF (runtime code will re-enable torque in its own init)
    """

    print("\n=== Dynamixel soft-limit calibration (interactive) ===")
    print("- This will turn TORQUE OFF so you can move each motor by hand.")
    print("- For each motor: move to MIN -> Enter, then MAX -> Enter.")
    print(f"- Will save to: {save_path}\n")

    limits: Dict[int, MotorLimit] = {}

    for mid in motor_ids:
        print(f"\n--- Motor ID {mid} ---")
        pkt.write1ByteTxRx(port, mid, ADDR_TORQUE_ENABLE, 0)

        input("Move motor to MIN mechanical limit, then press Enter...")
        pmin = _read_present_position(pkt, port, mid)
        print(f"  recorded MIN tick = {pmin}")

        input("Move motor to MAX mechanical limit, then press Enter...")
        pmax = _read_present_position(pkt, port, mid)
        print(f"  recorded MAX tick = {pmax}")

        limits[mid] = MotorLimit(pmin, pmax).normalized()
        print(f"  => normalized range: [{limits[mid].min_tick}, {limits[mid].max_tick}]")

    save_limits(save_path, limits)
    print("\n[OK] Saved motor limits.")
    return limits
