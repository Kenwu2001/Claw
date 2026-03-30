"""
Dynamixel keyboard control in Extended Position Mode.

Command grammar
---------------
Single-letter commands (exactly one letter):
    B / b  -> record baseline pos (all motors)
    U / u  -> record boundary pos (all motors)
    N / n  -> record initial pos (all motors)
    I / i  -> all motors move to initial degree
              - if initial absolute tick was already recorded by N/n, use it directly
              - otherwise compute from baseline + INTI_DEG_SIZE[mid], cache it, then move
    P / p  -> print baseline / boundary / initial tables

Motor motion commands:
    [motorID / B / E] [a/r] [angle]

Where:
    [motorID / B / E]
        - motorID: single motor, e.g. 2
        - B: GROUP_B_IDS
        - E: GROUP_E_IDS

    [a/r]
        - a = absolute angle relative to baseline (baseline = 0 deg)
        - r = relative angle relative to current present position

Examples:
    ba20     -> GROUP_B_IDS absolute 20 deg
    er-10    -> GROUP_E_IDS relative -10 deg
    2a-10    -> motor 2 absolute -10 deg

Spaces are allowed:
    "b a 20", "e r -10", "2 a -10"
"""

import re
import time
from typing import Dict, List, Optional, Tuple

from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite

from shared_state import SharedState
from motor_calibration import load_limits
from config import (
    DXL_BAUD,
    DXL_DEVICE,
    DXL_PROTOCOL,
    GROUP_E_IDS,
    GROUP_B_IDS,
    ENABLED_IDS,
    DXL_TICKS_PER_REV,
    KEYBOARD_CURRENT_LIMIT_MA,
    MOTOR_LIMITS_FILE,
    ENFORCE_SOFT_LIMITS,
    SOFT_LIMIT_MARGIN_TICKS,
    INTI_DEG_SIZE,
)

# ===== Dynamixel addresses (X series) =====
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_CURRENT_LIMIT = 38
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

DEFAULT_PROFILE_V = 500


def deg_to_ticks(deg: float) -> int:
    return int(round((deg / 360.0) * float(DXL_TICKS_PER_REV)))


def ticks_to_deg(ticks: int) -> float:
    return (float(ticks) / float(DXL_TICKS_PER_REV)) * 360.0


def _int32_to_little_endian_bytes(value: int) -> List[int]:
    """
    Convert signed int32 to 4 bytes (little endian) for Dynamixel sync write.
    """
    value &= 0xFFFFFFFF
    return [
        value & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF,
    ]


def _read_present_position(pkt: PacketHandler, port: PortHandler, dxl_id: int) -> int:
    pos, dxl_comm_result, dxl_error = pkt.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    if pos >= 2**31:
        pos -= 2**32
    if dxl_comm_result != 0 or dxl_error != 0:
        raise RuntimeError(f"read present pos failed for ID {dxl_id}")
    return int(pos)


def _write_goal_position(pkt: PacketHandler, port: PortHandler, dxl_id: int, goal: int):
    goal_u32 = int(goal) & 0xFFFFFFFF
    pkt.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, goal_u32)


def _write_profile_velocity(pkt: PacketHandler, port: PortHandler, dxl_id: int, goal: int):
    pkt.write4ByteTxRx(port, dxl_id, ADDR_PROFILE_VELOCITY, int(goal) & 0xFFFFFFFF)


def _setup_motor_extended_pos(pkt: PacketHandler, port: PortHandler, dxl_id: int):
    pkt.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)
    pkt.write1ByteTxRx(port, dxl_id, ADDR_OPERATING_MODE, 4)  # Extended Position Mode
    pkt.write2ByteTxRx(port, dxl_id, ADDR_CURRENT_LIMIT, int(KEYBOARD_CURRENT_LIMIT_MA))
    pkt.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)


def _clamp_to_soft_limits(dxl_id: int, goal_tick: int, limits, margin: int) -> int:
    if not ENFORCE_SOFT_LIMITS:
        return goal_tick
    lim = limits.get(dxl_id)
    if lim is None:
        return goal_tick
    lo = lim.min_tick + int(margin)
    hi = lim.max_tick - int(margin)
    if lo > hi:
        return goal_tick
    if goal_tick < lo:
        return lo
    if goal_tick > hi:
        return hi
    return goal_tick


def _parse_motion_cmd(line: str) -> Optional[Tuple[str, List[int], str, float]]:
    """
    Parse motion command:
        [motorID / B / E] [a/r] [angle]

    Returns:
        (kind, ids, mode, angle_deg)
        kind: 'group' or 'single'
        mode: 'abs' or 'rel'
    """
    s = re.sub(r"\s+", "", line.strip())
    if not s:
        return None

    if s.lower() in ("q", "quit", "exit"):
        return ("quit", [], "abs", 0.0)

    # admin commands should not be parsed here
    if len(s) == 1 and s.lower() in ("b", "u", "n", "i", "p", "h", "?"):
        return None

    # group: ba20 / er-10
    m = re.fullmatch(r"([bBeE])([aArR])([+-]?\d+(?:\.\d+)?)", s)
    if m:
        grp = m.group(1).lower()
        mode = "abs" if m.group(2).lower() == "a" else "rel"
        val = float(m.group(3))
        ids = list(GROUP_B_IDS) if grp == "b" else list(GROUP_E_IDS)
        return ("group", ids, mode, val)

    # single: 2a-10 / 8r15
    m = re.fullmatch(r"(\d+)([aArR])([+-]?\d+(?:\.\d+)?)", s)
    if m:
        mid = int(m.group(1))
        mode = "abs" if m.group(2).lower() == "a" else "rel"
        val = float(m.group(3))
        return ("single", [mid], mode, val)

    return None


class KeyboardPosController:
    """Non-blocking keyboard position controller."""

    def __init__(self, state: SharedState):
        self.state = state

        self.port = PortHandler(DXL_DEVICE)
        if not self.port.openPort():
            raise RuntimeError(f"Failed to open Dynamixel port: {DXL_DEVICE}")
        if not self.port.setBaudRate(DXL_BAUD):
            raise RuntimeError(f"Failed to set Dynamixel baud: {DXL_BAUD}")
        self.pkt = PacketHandler(DXL_PROTOCOL)

        self.limits = load_limits(MOTOR_LIMITS_FILE)

        self.all_ids = sorted(set(ENABLED_IDS))
        if not self.all_ids:
            raise RuntimeError("No motors configured for keyboard mode (ENABLED_IDS empty).")

        for mid in self.all_ids:
            _setup_motor_extended_pos(self.pkt, self.port, mid)

        with self.state.lock:
            self.baseline: Dict[int, int] = dict(getattr(self.state, "baseline_ticks", {}))
            self.boundary: Dict[int, int] = dict(getattr(self.state, "boundary_ticks", {}))
            self.initial_abs_ticks: Dict[int, int] = dict(getattr(self.state, "initial_ticks", {}))

        # copy from config, but can be modified by method
        self.initial_deg_by_id: Dict[int, float] = {
            int(mid): float(INTI_DEG_SIZE.get(mid, 0.0)) for mid in self.all_ids
        }

        self.profile_v = DEFAULT_PROFILE_V
        self.profile_v_by_id: Dict[int, int] = {
        mid: DEFAULT_PROFILE_V for mid in self.all_ids
        }

    # -------------------------
    # public methods
    # -------------------------
    def set_profile_velocity(self, value: int):
        """
        Set profile velocity used by subsequent motion commands.
        """
        value = int(value)
        if value < 0:
            raise ValueError("profile_velocity must be >= 0")
        self.profile_v = value
    
    def set_profile_velocity_for_ids(self, ids, value: int):
        value = int(value)
        if value < 0:
            raise ValueError("profile_velocity must be >= 0")
        for mid in ids:
            if mid in self.profile_v_by_id:
                self.profile_v_by_id[mid] = value

    def set_profile_velocity_for_id(self, mid: int, value: int):
        value = int(value)
        if value < 0:
            raise ValueError("profile_velocity must be >= 0")
        if mid in self.profile_v_by_id:
            self.profile_v_by_id[mid] = value

    def set_initial_deg(
        self,
        mapping: Dict[int, float],
        move_now: bool = False,
        recompute_existing: bool = True,
    ):
        """
        Update initial degree table (relative to baseline).

        Args:
            mapping:
                dict like {1: 10.0, 2: -5.0}
            move_now:
                if True, move updated motors now
            recompute_existing:
                if True, refresh cached absolute initial tick for updated motors
                using baseline + new degree.
        """
        updated_ids: List[int] = []

        for mid, deg in mapping.items():
            mid = int(mid)
            if mid not in self.all_ids:
                continue
            self.initial_deg_by_id[mid] = float(deg)
            updated_ids.append(mid)

        if recompute_existing and self._require_baseline():
            for mid in updated_ids:
                goal = self.baseline[mid] + deg_to_ticks(self.initial_deg_by_id[mid])
                goal = _clamp_to_soft_limits(mid, goal, self.limits, SOFT_LIMIT_MARGIN_TICKS)
                self.initial_abs_ticks[mid] = goal

            with self.state.lock:
                self.state.initial_ticks = dict(self.initial_abs_ticks)

        if move_now:
            if not self._require_baseline():
                return

            goals_by_id: Dict[int, int] = {}
            for mid in updated_ids:
                goal = self._get_initial_goal_tick(mid)
                if goal is None:
                    continue
                goals_by_id[mid] = goal

            self._move_motors_to_ticks_sync(goals_by_id)

    # -------------------------
    # recording
    # -------------------------
    def _read_all_present_positions(self) -> Dict[int, int]:
        out: Dict[int, int] = {}
        for mid in self.all_ids:
            out[mid] = _read_present_position(self.pkt, self.port, mid)
        return out

    def record_baseline(self):
        self.baseline = self._read_all_present_positions()
        with self.state.lock:
            self.state.baseline_ticks = dict(self.baseline)

    def record_boundary(self):
        self.boundary = self._read_all_present_positions()
        with self.state.lock:
            self.state.boundary_ticks = dict(self.boundary)

    def record_initial(self):
        self.initial_abs_ticks = self._read_all_present_positions()
        with self.state.lock:
            self.state.initial_ticks = dict(self.initial_abs_ticks)

    # -------------------------
    # helpers
    # -------------------------
    def _require_baseline(self) -> bool:
        return len(self.baseline) > 0

    def _get_initial_goal_tick(self, mid: int) -> Optional[int]:
        """
        Return cached initial absolute tick if present.
        Otherwise compute baseline + initial_deg_by_id[mid], cache it, return it.
        """
        if mid in self.initial_abs_ticks:
            return int(self.initial_abs_ticks[mid])

        if not self._require_baseline():
            return None

        rel_deg = float(self.initial_deg_by_id.get(mid, 0.0))
        goal = int(self.baseline[mid] + deg_to_ticks(rel_deg))
        goal = _clamp_to_soft_limits(mid, goal, self.limits, SOFT_LIMIT_MARGIN_TICKS)

        self.initial_abs_ticks[mid] = goal
        with self.state.lock:
            self.state.initial_ticks = dict(self.initial_abs_ticks)

        return goal

    def _move_motor_to_tick(self, mid: int, goal_tick: int):
        goal_tick = _clamp_to_soft_limits(mid, int(goal_tick), self.limits, SOFT_LIMIT_MARGIN_TICKS)
        # print(f"[DEBUG] ID {mid} goal_tick={goal_tick}")
        v = self.profile_v_by_id.get(mid, self.profile_v)
        _write_profile_velocity(self.pkt, self.port, mid, v)
        _write_goal_position(self.pkt, self.port, mid, goal_tick)

    def _move_motors_to_ticks_sync(self, goals_by_id: Dict[int, int]):
        """
        Move multiple motors with one sync write on GOAL_POSITION.
        This makes group commands start much more simultaneously.
        """
        if not goals_by_id:
            return

        # set profile velocity first
        for mid in goals_by_id.keys():
            v = self.profile_v_by_id.get(mid, self.profile_v)
            # print(f"[DEBUG] ID {mid} profile_v={v}")
            _write_profile_velocity(self.pkt, self.port, mid, v)

        group = GroupSyncWrite(self.port, self.pkt, ADDR_GOAL_POSITION, 4)

        try:
            for mid, goal_tick in goals_by_id.items():
                goal_tick = _clamp_to_soft_limits(mid, int(goal_tick), self.limits, SOFT_LIMIT_MARGIN_TICKS)
                param = _int32_to_little_endian_bytes(goal_tick)
                ok = group.addParam(mid, param)
                if not ok:
                    raise RuntimeError(f"GroupSyncWrite addParam failed for ID {mid}")

            dxl_comm_result = group.txPacket()
            if dxl_comm_result != 0:
                raise RuntimeError(
                    f"GroupSyncWrite txPacket failed: {self.pkt.getTxRxResult(dxl_comm_result)}"
                )
        finally:
            group.clearParam()

    def move_all_to_initial(self):
        if not self._require_baseline():
            return

        goals_by_id: Dict[int, int] = {}
        for mid in self.all_ids:
            goal = self._get_initial_goal_tick(mid)
            if goal is None:
                continue
            goals_by_id[mid] = goal

        self._move_motors_to_ticks_sync(goals_by_id)

    def move_all_to_baseline(self):
        if not self._require_baseline():
            return

        goals_by_id: Dict[int, int] = {}
        for mid in self.all_ids:
            if mid in self.baseline:
                goals_by_id[mid] = int(self.baseline[mid])

        self._move_motors_to_ticks_sync(goals_by_id)

    def reboot_all_motors(self, reboot_delay_s: float = 0.3):
        """
        Reboot all enabled motors, then re-apply runtime setup.

        XL330 present position can shift after reboot, so cached position
        references are cleared on purpose.
        """
        reboot_delay_s = max(0.0, float(reboot_delay_s))

        for mid in self.all_ids:
            dxl_comm_result, dxl_error = self.pkt.reboot(self.port, mid)
            if dxl_comm_result != 0 or dxl_error != 0:
                raise RuntimeError(f"reboot failed for ID {mid}")
            if reboot_delay_s > 0.0:
                time.sleep(reboot_delay_s)

        for mid in self.all_ids:
            _setup_motor_extended_pos(self.pkt, self.port, mid)

        self.baseline = {}
        self.boundary = {}
        self.initial_abs_ticks = {}
        with self.state.lock:
            self.state.baseline_ticks = {}
            self.state.boundary_ticks = {}
            self.state.initial_ticks = {}

    def print_tables(self):
        print("\n[POS] ===== Motor Tables =====")
        print(" ID | baseline_tick | boundary_tick | initial_tick | init_deg_cfg | init_deg_from_baseline")
        print("----+---------------+---------------+--------------+--------------+------------------------")
        for mid in self.all_ids:
            b = self.baseline.get(mid, None)
            u = self.boundary.get(mid, None)
            n = self.initial_abs_ticks.get(mid, None)
            init_deg = float(self.initial_deg_by_id.get(mid, 0.0))

            b_str = str(b) if b is not None else "-"
            u_str = str(u) if u is not None else "-"
            n_str = str(n) if n is not None else "-"

            if b is not None and n is not None:
                rel_deg = ticks_to_deg(n - b)
                rel_deg_str = f"{rel_deg:.2f}"
            else:
                rel_deg_str = "-"

            print(f"{mid:>3} | {b_str:>13} | {u_str:>13} | {n_str:>12} | {init_deg:>12.2f} | {rel_deg_str:>22}")
        print(f"[POS] profile_velocity = {self.profile_v}")
        print("[POS] ========================\n")

    # -------------------------
    # command handler
    # -------------------------
    def handle_line(self, line: str):
        """Handle one input line. Returns 'quit' or None."""
        raw = line.rstrip("\n")
        s = raw.strip()

        if not s:
            return None

        cmd1 = re.sub(r"\s+", "", s).lower()

        if cmd1 in ("h", "help", "?"):
            return None

        if cmd1 == "b":
            self.record_baseline()
            return None

        if cmd1 == "u":
            self.record_boundary()
            return None

        if cmd1 == "n":
            self.record_initial()
            return None

        if cmd1 == "i":
            self.move_all_to_initial()
            return None

        if cmd1 == "p":
            self.print_tables()
            return None

        parsed = _parse_motion_cmd(s)
        if parsed is None:
            return None

        kind, ids, mode, val_deg = parsed

        if kind == "quit":
            return "quit"

        goals_by_id: Dict[int, int] = {}

        for mid in ids:
            if mid not in self.all_ids:
                continue

            if mode == "abs":
                if not self._require_baseline():
                    continue
                # goal = self.baseline[mid] + deg_to_ticks(val_deg)
                delta_tick = deg_to_ticks(val_deg)
                goal = self.baseline[mid] + delta_tick
                # print(f"[DEBUG] ID {mid} ABS: baseline={self.baseline[mid]}, val_deg={val_deg}, delta_tick={delta_tick}, goal={goal}")
            else:
                cur = _read_present_position(self.pkt, self.port, mid)
                # goal = cur + deg_to_ticks(val_deg)
                delta_tick = deg_to_ticks(val_deg)
                goal = cur + delta_tick
                # print(f"[DEBUG] ID {mid} REL: cur={cur}, val_deg={val_deg}, delta_tick={delta_tick}, goal={goal}")

            goals_by_id[mid] = goal

        if kind == "group":
            self._move_motors_to_ticks_sync(goals_by_id)
        else:
            for mid, goal in goals_by_id.items():
                self._move_motor_to_tick(mid, goal)

        return None

    def close(self):
        for mid in self.all_ids:
            try:
                self.pkt.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, 0)
            except Exception:
                pass
        try:
            self.port.closePort()
        except Exception:
            pass


def dynamixel_keyboard_pos_control(state: SharedState):
    """Legacy blocking version (kept for compatibility)."""
    ctrl = KeyboardPosController(state)
    try:
        while not state.stop:
            line = input("> ")
            act = ctrl.handle_line(line)
            if act == "quit":
                break
            time.sleep(0.02)
    finally:
        ctrl.close()
