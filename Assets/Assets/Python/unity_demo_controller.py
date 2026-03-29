import re
import time
from typing import Dict

from dynamixel_keyboard_pos_control import (
    KeyboardPosController,
    deg_to_ticks,
    _read_present_position,
)
from config import (
    GROUP_B_IDS,
    GROUP_E_IDS,
    SIZE_VB,
    SIZE_VE,
    APPL_VB,
    APPL_VE,
    INTI_DEG_FORCE,
    FIN_DEG_FORCE,
)


class UnityDemoController:
    """
    Unity -> Python
      B           : record baseline
      N           : go to initial (baseline-based absolute)
      VSIZE       : use SIZE_VB for B-group, SIZE_VE for E-group
      VAPPL       : use APPL_VB for B-group, APPL_VE for E-group

      Move command examples:
        b20
        b-20
        e20
        e-20
        b20e20
        b-20e-20

      Semantics:
        - Unity sends trial-space angles
        - Python first applies calibration:
            B: x -> 1.5689 * x
            E: x -> 9.1618 * x
        - Then Python applies sign policy:
            b+ -> negative relative move
            b- -> positive relative move
            e+ -> positive relative move
            e- -> negative relative move

      K           : ack only (legacy compatibility)
      Q           : quit

    Python -> Unity
      K           : ok / step finished
      B           : bye
      E ...       : error
    """

    SETTLE_SECONDS = 0.05

    # calibration multipliers requested by user
    B_CAL = 1.5689
    E_CAL = 9.1618

    _MOVE_RE = re.compile(
        r"^(?:(b)([+-]?\d+(?:\.\d+)?))?(?:(e)([+-]?\d+(?:\.\d+)?))?$",
        re.IGNORECASE
    )

    def __init__(self, state):
        self.state = state
        self.motor = KeyboardPosController(state)

        # force mode initial pose table
        self.motor.set_initial_deg(INTI_DEG_FORCE, move_now=False, recompute_existing=True)
        self.fin_deg_force = {int(k): float(v) for k, v in FIN_DEG_FORCE.items()}

        self.busy = False
        self.baseline_ready = len(self.motor.baseline) > 0
        self.at_initial = False

        # legacy flag, kept for compatibility
        self.auto_reverse_on_anim_done = False

        # last actually applied relative move (post calibration + sign conversion)
        self.b_last_rel_deg = 0.0
        self.e_last_rel_deg = 0.0

        # default velocity mode -> APPL
        self.velocity_mode = "APPL"
        self._set_velocity_mode("APPL")
        

    def close(self):
        self.motor.close()

    def _ensure_not_busy(self):
        if self.busy:
            raise RuntimeError("busy")

    def _clear_last_rel(self):
        self.b_last_rel_deg = 0.0
        self.e_last_rel_deg = 0.0

    def _set_group_profile_velocity(self, ids, value):
        self.motor.set_profile_velocity_for_ids(ids, int(value))

    def _set_velocity_mode(self, mode: str):
        mode = mode.upper()

        if mode == "SIZE":
            self._set_group_profile_velocity(GROUP_B_IDS, SIZE_VB)
            self._set_group_profile_velocity(GROUP_E_IDS, SIZE_VE)

        elif mode == "APPL":
            self._set_group_profile_velocity(GROUP_B_IDS, APPL_VB)
            self._set_group_profile_velocity(GROUP_E_IDS, APPL_VE)

        else:
            raise ValueError(f"unknown_velocity_mode:{mode}")

        self.velocity_mode = mode

    def _build_sync_goals_from_relative(self, b_rel_deg=None, e_rel_deg=None) -> Dict[int, int]:
        goals_by_id: Dict[int, int] = {}

        if b_rel_deg is not None:
            for mid in GROUP_B_IDS:
                cur = _read_present_position(self.motor.pkt, self.motor.port, mid)
                goals_by_id[mid] = cur + deg_to_ticks(float(b_rel_deg))

        if e_rel_deg is not None:
            for mid in GROUP_E_IDS:
                cur = _read_present_position(self.motor.pkt, self.motor.port, mid)
                goals_by_id[mid] = cur + deg_to_ticks(float(e_rel_deg))

        return goals_by_id

    def _convert_b_command(self, b_raw) -> float:
        """
        Unity B command is in trial-space degrees.
        Steps:
          1) apply calibration: x -> 1.5689 * x
          2) apply sign rule:
               b+20 -> -calibrated
               b-20 -> +calibrated
        """
        raw = float(b_raw)
        calibrated = abs(raw) * self.B_CAL

        if str(b_raw).startswith("-"):
            return +calibrated
        else:
            return -calibrated

    def _convert_e_command(self, e_raw) -> float:
        """
        Unity E command is in trial-space degrees.
        Steps:
          1) apply calibration: x -> 9.1618 * x
          2) apply sign rule:
               e+20 -> +calibrated
               e-20 -> -calibrated
        """
        raw = float(e_raw)
        calibrated = abs(raw) * self.E_CAL

        if str(e_raw).startswith("-"):
            return -calibrated
        else:
            return +calibrated

    def _apply_trial_relative(self, b_mag=None, e_mag=None):
        """
        Accept a move relative to CURRENT pose.
        This is important because Unity may do:
          1) full open
          2) immediately move back to 50% pose
        so second move must start from the current pose, not from initial.
        """
        b_rel = None
        e_rel = None

        if b_mag is not None:
            b_rel = self._convert_b_command(b_mag)

        if e_mag is not None:
            e_rel = self._convert_e_command(e_mag)

        goals_by_id = self._build_sync_goals_from_relative(b_rel, e_rel)
        if not goals_by_id:
            raise RuntimeError("empty_trial_command")

        self.motor._move_motors_to_ticks_sync(goals_by_id)

        if b_rel is not None:
            self.b_last_rel_deg = b_rel
        if e_rel is not None:
            self.e_last_rel_deg = e_rel

        self.at_initial = False

    def _reverse_last_relative(self):
        b_rev = None
        e_rev = None

        if self.b_last_rel_deg != 0:
            b_rev = -self.b_last_rel_deg
        if self.e_last_rel_deg != 0:
            e_rev = -self.e_last_rel_deg

        if b_rev is None and e_rev is None:
            self.at_initial = True
            return

        goals_by_id = self._build_sync_goals_from_relative(b_rev, e_rev)
        self.motor._move_motors_to_ticks_sync(goals_by_id)

        self._clear_last_rel()
        self.at_initial = True

    def handle_record_baseline(self):
        self._ensure_not_busy()
        try:
            self.motor.record_baseline()
            self.baseline_ready = True
            self.at_initial = False
            self._clear_last_rel()
            return "K"
        except Exception as e:
            return f"E {e}"

    def handle_go_initial(self):
        self._ensure_not_busy()
        if not self.baseline_ready:
            return "E baseline_not_recorded"

        self.busy = True
        try:
            self.motor.move_all_to_initial()
            # time.sleep(self.SETTLE_SECONDS)
            time.sleep(1.0)  # keep your original pause
            self.at_initial = True
            self._clear_last_rel()
            return "K"
        except Exception as e:
            return f"E {e}"
        finally:
            self.busy = False

    def handle_go_finish(self):
        self._ensure_not_busy()
        if not self.baseline_ready:
            return "E baseline_not_recorded"

        self.busy = True
        try:
            goals_by_id = {}
            for mid in self.motor.all_ids:
                fin_deg = float(self.fin_deg_force.get(mid, 0.0))
                goal = self.motor.baseline[mid] + deg_to_ticks(fin_deg)
                goals_by_id[mid] = goal

            time.sleep(0.2)  # small pause before moving to final pose
            self.motor._move_motors_to_ticks_sync(goals_by_id)

            time.sleep(0.5)
            self.at_initial = False
            self._clear_last_rel()
            return "K"
        except Exception as e:
            return f"E {e}"
        finally:
            self.busy = False

    def handle_set_velocity_mode(self, mode: str):
        self._ensure_not_busy()
        try:
            self._set_velocity_mode(mode)
            return "K"
        except Exception as e:
            return f"E {e}"

    def handle_set_auto_reverse(self, enabled: bool):
        self._ensure_not_busy()
        try:
            self.auto_reverse_on_anim_done = bool(enabled)
            return "K"
        except Exception as e:
            return f"E {e}"

    def handle_reboot_all(self):
        self._ensure_not_busy()

        self.busy = True
        try:
            self.motor.reboot_all_motors()
            self.baseline_ready = False
            self.at_initial = False
            self._clear_last_rel()
            return "K"
        except Exception as e:
            return f"E {e}"
        finally:
            self.busy = False

    def handle_trial_move(self, b_mag=None, e_mag=None):
        self._ensure_not_busy()
        if not self.baseline_ready:
            return "E baseline_not_recorded"

        # IMPORTANT:
        # no longer requiring at_initial == True
        # because Unity may send:
        #   move to 100%
        #   then move back to 50%
        self.busy = True
        try:
            self._apply_trial_relative(b_mag=b_mag, e_mag=e_mag)
            time.sleep(self.SETTLE_SECONDS)
            return "K"
        except Exception as e:
            return f"E {e}"
        finally:
            self.busy = False

    def handle_anim_done(self):
        """
        When animation done:
        - if auto reverse enabled, move back by reversing last relative move
        - otherwise just ACK
        """
        self._ensure_not_busy()
        if not self.baseline_ready:
            return "E baseline_not_recorded"

        self.busy = True
        try:
            if self.auto_reverse_on_anim_done:
                # self._reverse_last_relative()
                self.motor.move_all_to_initial()
                self.at_initial = True
                self._clear_last_rel()
                time.sleep(self.SETTLE_SECONDS)
                # time.sleep(1.5)
            return "K"
        except Exception as e:
            return f"E {e}"
        finally:
            self.busy = False

    def handle_line(self, line: str):
        cmd = (line or "").strip()
        if not cmd:
            return None

        cmd_upper = cmd.upper()

        if cmd_upper == "B":
            return self.handle_record_baseline()

        if cmd_upper in ("N", "I"):
            return self.handle_go_initial()
            
        if cmd_upper == "F":
            return self.handle_go_finish()

        if cmd_upper == "VSIZE":
            return self.handle_set_velocity_mode("SIZE")

        if cmd_upper == "VAPPL":
            return self.handle_set_velocity_mode("APPL")

        if cmd_upper == "AUTOREV1":
            return self.handle_set_auto_reverse(True)

        if cmd_upper == "AUTOREV0":
            return self.handle_set_auto_reverse(False)

        if cmd_upper in ("REBOOT", "RB"):
            return self.handle_reboot_all()

        if cmd_upper == "K":
            return self.handle_anim_done()

        if cmd_upper == "Q":
            return "B"

        m = self._MOVE_RE.fullmatch(cmd)
        if m:
            b_mag = m.group(2) if m.group(1) else None
            e_mag = m.group(4) if m.group(3) else None

            if b_mag is None and e_mag is None:
                return f"E bad_command:{cmd}"

            return self.handle_trial_move(b_mag=b_mag, e_mag=e_mag)
    

        return f"E unknown_command:{cmd_upper}"
