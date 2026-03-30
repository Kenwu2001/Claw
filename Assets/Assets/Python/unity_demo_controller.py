import re
import time
import sys
import threading
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
    UNITY_DEMO_PAYLOAD_PRESETS,
)


class UnityDemoController:
    """
    Unity -> Python
      B           : record baseline
      SB          : move all enabled motors back to recorded baseline
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
        s6+20
        s6+20s7-20
        sv6=80
        # b/e combined in one command are executed together as one sync move

      Payload command examples:
        b20e40,v80,t5,b-20
        v60,sv6=80,s6+20,t1.5,e-5
        PL:b_cycle_demo

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
    _SINGLE_MOVE_RE = re.compile(r"^s(\d+)([+-]?\d+(?:\.\d+)?)$", re.IGNORECASE)
    _SINGLE_MOVE_TOKEN_RE = re.compile(r"s(\d+)([+-]?\d+(?:\.\d+)?)", re.IGNORECASE)
    _MULTI_SINGLE_MOVE_RE = re.compile(r"^(?:s\d+[+-]?\d+(?:\.\d+)?)+$", re.IGNORECASE)
    _SINGLE_VEL_RE = re.compile(r"^sv(\d+)=([+-]?\d+(?:\.\d+)?)$", re.IGNORECASE)
    _PAYLOAD_CONTROL_RE = re.compile(r"^(vb|ve|v|t)([+-]?\d+(?:\.\d+)?)$", re.IGNORECASE)

    SEQUENCE_DELAY_SECONDS = 0.5
    PRESET_HOTKEYS = {
        "g": "vollsmall",
        "m": "move",
        "j": "backmove",
        "y": "basbig",
        "u": "bomb",
        "o": "roll",
    }
    LOOPING_PRESET_HOTKEYS = {
        "c": "heart",
        "a": "breath",
    }
    SEQUENCE_COMMANDS = {
        "1": [("10", "10"), ("-10", None), (None, "-20")],
        "2": [("20", "20"), ("-10", "-10"), ("-10", "-20")],
        "3": [("30", "30"), ("-20", "-20"), (None, "-20")],
    }

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
        self._loop_stop_event = threading.Event()
        self._loop_thread = None
        self._loop_preset_name = None
        

    def close(self):
        self.stop_looping_preset()
        self.motor.close()

    def build_help_text(self) -> str:
        return (
            "[UnityDemo] Hotkeys / Commands:\n"
            "  B: Record Baseline\n"
            "  SB: Return all motors to recorded baseline\n"
            "  I / N: Go Initial\n"
            "  F / f: Go Finish\n"
            "  O: Reboot Motors\n"
            "  c: loop PL:heart\n"
            "  a: loop PL:breath\n"
            "  k: stop loop preset\n"
            "  g: PL:vollsmall\n"
            "  m: PL:move\n"
            "  j: PL:backmove\n"
            "  y: PL:basbig\n"
            "  u: PL:bomb\n"
            "  o: PL:roll\n"
            "  PL:<name>: run payload preset from config.py\n"
        )

    def print_help(self):
        sys.stderr.write(self.build_help_text())
        sys.stderr.flush()

    def _ensure_not_busy(self):
        if self.busy:
            raise RuntimeError("busy")

    def _clear_last_rel(self):
        self.b_last_rel_deg = 0.0
        self.e_last_rel_deg = 0.0

    def _loop_worker(self, preset_name: str):
        while not self._loop_stop_event.is_set():
            result = self.handle_payload_preset(preset_name)
            if result != "K":
                sys.stderr.write(f"[UnityDemo] loop preset `{preset_name}` stopped: {result}\n")
                sys.stderr.flush()
                break

        self._loop_stop_event.set()
        self._loop_preset_name = None
        self._loop_thread = None

    def start_looping_preset(self, preset_name: str):
        if self._loop_thread is not None and self._loop_thread.is_alive():
            if self._loop_preset_name == preset_name:
                return "K"
            self.stop_looping_preset()

        self._loop_stop_event.clear()
        self._loop_preset_name = preset_name
        self._loop_thread = threading.Thread(
            target=self._loop_worker,
            args=(preset_name,),
            daemon=True,
        )
        self._loop_thread.start()
        return "K"

    def stop_looping_preset(self):
        t = self._loop_thread
        if t is None:
            self._loop_preset_name = None
            self._loop_stop_event.set()
            return "K"

        self._loop_stop_event.set()
        if t.is_alive() and threading.current_thread() is not t:
            t.join(timeout=2.0)

        self._loop_thread = None
        self._loop_preset_name = None
        return "K"

    def _set_group_profile_velocity(self, ids, value):
        valid_ids = [int(mid) for mid in ids if int(mid) in self.motor.all_ids]
        if not valid_ids:
            return
        self.motor.set_profile_velocity_for_ids(valid_ids, int(value))

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

    def _set_payload_velocity(self, token: str, value: float):
        vel = int(round(float(value)))
        if vel < 0:
            raise ValueError(f"invalid_profile_velocity:{value}")

        token = token.lower()
        if token == "v":
            self._set_group_profile_velocity(GROUP_B_IDS, vel)
            self._set_group_profile_velocity(GROUP_E_IDS, vel)
        elif token == "vb":
            self._set_group_profile_velocity(GROUP_B_IDS, vel)
        elif token == "ve":
            self._set_group_profile_velocity(GROUP_E_IDS, vel)
        else:
            raise ValueError(f"unknown_velocity_token:{token}")

    def _set_single_profile_velocity(self, motor_id: int, value: float):
        mid = int(motor_id)
        vel = int(round(float(value)))
        if vel < 0:
            raise ValueError(f"invalid_profile_velocity:{value}")
        if mid not in self.motor.all_ids:
            return
        self.motor.set_profile_velocity_for_id(mid, vel)

    def _build_sync_goals_from_relative(self, b_rel_deg=None, e_rel_deg=None) -> Dict[int, int]:
        goals_by_id: Dict[int, int] = {}

        if b_rel_deg is not None:
            for mid in GROUP_B_IDS:
                if mid not in self.motor.all_ids:
                    continue
                cur = _read_present_position(self.motor.pkt, self.motor.port, mid)
                goals_by_id[mid] = cur + deg_to_ticks(float(b_rel_deg))

        if e_rel_deg is not None:
            for mid in GROUP_E_IDS:
                if mid not in self.motor.all_ids:
                    continue
                cur = _read_present_position(self.motor.pkt, self.motor.port, mid)
                goals_by_id[mid] = cur + deg_to_ticks(float(e_rel_deg))

        return goals_by_id

    def _build_single_goal_from_relative(self, motor_id: int, rel_deg: float) -> Dict[int, int]:
        mid = int(motor_id)
        if mid not in self.motor.all_ids:
            return {}

        cur = _read_present_position(self.motor.pkt, self.motor.port, mid)
        return {mid: cur + deg_to_ticks(float(rel_deg))}

    def _build_multi_single_goals_from_relative(self, moves) -> Dict[int, int]:
        goals_by_id: Dict[int, int] = {}
        for motor_id, rel_deg in moves:
            mid = int(motor_id)
            if mid not in self.motor.all_ids:
                continue
            if mid in goals_by_id:
                raise RuntimeError(f"duplicate_motor_id_in_step:{mid}")

            cur = _read_present_position(self.motor.pkt, self.motor.port, mid)
            goals_by_id[mid] = cur + deg_to_ticks(float(rel_deg))

        if not goals_by_id:
            raise RuntimeError("empty_single_move_step")
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

    def _apply_single_relative(self, motor_id: int, rel_deg: float):
        goals_by_id = self._build_single_goal_from_relative(motor_id, rel_deg)
        if not goals_by_id:
            return
        self.motor._move_motors_to_ticks_sync(goals_by_id)
        self.at_initial = False

    def _apply_multi_single_relative(self, moves):
        goals_by_id = self._build_multi_single_goals_from_relative(moves)
        if not goals_by_id:
            return
        self.motor._move_motors_to_ticks_sync(goals_by_id)
        self.at_initial = False

    def _parse_payload(self, payload: str):
        text = (payload or "").strip()
        if not text:
            raise ValueError("empty_payload")

        steps = []
        for raw_step in text.split(","):
            step = raw_step.strip()
            if not step:
                raise ValueError(f"bad_payload:{text}")

            move_match = self._MOVE_RE.fullmatch(step)
            if move_match:
                b_mag = move_match.group(2) if move_match.group(1) else None
                e_mag = move_match.group(4) if move_match.group(3) else None
                if b_mag is None and e_mag is None:
                    raise ValueError(f"bad_payload_step:{step}")

                steps.append(("move", b_mag, e_mag))
                continue

            single_match = self._SINGLE_MOVE_RE.fullmatch(step)
            if single_match:
                steps.append(("single_move", int(single_match.group(1)), float(single_match.group(2))))
                continue

            if self._MULTI_SINGLE_MOVE_RE.fullmatch(step):
                moves = [
                    (int(m.group(1)), float(m.group(2)))
                    for m in self._SINGLE_MOVE_TOKEN_RE.finditer(step)
                ]
                steps.append(("multi_single_move", moves))
                continue

            single_vel_match = self._SINGLE_VEL_RE.fullmatch(step)
            if single_vel_match:
                steps.append(("single_vel", int(single_vel_match.group(1)), float(single_vel_match.group(2))))
                continue

            control_match = self._PAYLOAD_CONTROL_RE.fullmatch(step)
            if control_match:
                steps.append((control_match.group(1).lower(), float(control_match.group(2))))
                continue

            raise ValueError(f"bad_payload_step:{step}")

        return steps

    def _looks_like_payload(self, cmd: str) -> bool:
        text = (cmd or "").strip()
        if not text or "," not in text:
            return False

        try:
            self._parse_payload(text)
            return True
        except Exception:
            return False

    def handle_payload_command(self, payload: str):
        self._ensure_not_busy()

        try:
            steps = self._parse_payload(payload)
        except Exception as e:
            return f"E {e}"

        if any(step[0] == "move" for step in steps) and not self.baseline_ready:
            return "E baseline_not_recorded"

        self.busy = True
        try:
            for step in steps:
                token = step[0]

                if token == "move":
                    _, b_mag, e_mag = step
                    self._apply_trial_relative(b_mag=b_mag, e_mag=e_mag)
                    time.sleep(self.SETTLE_SECONDS)
                    continue

                if token == "single_move":
                    _, motor_id, rel_deg = step
                    self._apply_single_relative(motor_id, rel_deg)
                    time.sleep(self.SETTLE_SECONDS)
                    continue

                if token == "multi_single_move":
                    _, moves = step
                    self._apply_multi_single_relative(moves)
                    time.sleep(self.SETTLE_SECONDS)
                    continue

                if token == "single_vel":
                    _, motor_id, vel = step
                    self._set_single_profile_velocity(motor_id, vel)
                    continue

                value = step[1]

                if token in ("v", "vb", "ve"):
                    self._set_payload_velocity(token, value)
                    continue

                if token == "t":
                    time.sleep(max(0.0, float(value)))
                    continue

                raise RuntimeError(f"unsupported_payload_token:{token}")

            # After a full payload sequence finishes, always return to baseline.
            if self.baseline_ready:
                self.motor.move_all_to_baseline()
                time.sleep(self.SETTLE_SECONDS)
                self._clear_last_rel()
                self.at_initial = False

            return "K"
        except Exception as e:
            return f"E {e}"
        finally:
            self.busy = False

    def handle_payload_preset(self, preset_name: str):
        payload = UNITY_DEMO_PAYLOAD_PRESETS.get((preset_name or "").strip().lower())
        if payload is None:
            return f"E unknown_payload_preset:{preset_name}"
        return self.handle_payload_command(payload)

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

    def handle_go_baseline(self):
        self._ensure_not_busy()
        if not self.baseline_ready:
            return "E baseline_not_recorded"

        self.busy = True
        try:
            self.motor.move_all_to_baseline()
            time.sleep(self.SETTLE_SECONDS)
            self.at_initial = False
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

            time.sleep(0.05)  # small pause before moving to final pose
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

    def handle_sequence_command(self, seq_key: str):
        self._ensure_not_busy()
        if not self.baseline_ready:
            return "E baseline_not_recorded"

        steps = self.SEQUENCE_COMMANDS.get(seq_key)
        if not steps:
            return f"E unknown_sequence:{seq_key}"

        self.busy = True
        try:
            for idx, (b_mag, e_mag) in enumerate(steps):
                self._apply_trial_relative(b_mag=b_mag, e_mag=e_mag)
                time.sleep(self.SETTLE_SECONDS)

                if idx < len(steps) - 1:
                    time.sleep(self.SEQUENCE_DELAY_SECONDS)

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
        cmd_lower = cmd.lower()

        if cmd in self.LOOPING_PRESET_HOTKEYS:
            return self.start_looping_preset(self.LOOPING_PRESET_HOTKEYS[cmd])

        if cmd == "k":
            return self.stop_looping_preset()

        if cmd in self.PRESET_HOTKEYS:
            return self.handle_payload_preset(self.PRESET_HOTKEYS[cmd])

        if cmd_upper == "B":
            return self.handle_record_baseline()

        if cmd_upper == "SB":
            return self.handle_go_baseline()

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

        if cmd == "K":
            return self.handle_anim_done()

        if cmd_upper == "Q":
            return "B"

        if cmd_upper in self.SEQUENCE_COMMANDS:
            return self.handle_sequence_command(cmd_upper)

        m = self._MOVE_RE.fullmatch(cmd)
        if m:
            b_mag = m.group(2) if m.group(1) else None
            e_mag = m.group(4) if m.group(3) else None

            if b_mag is None and e_mag is None:
                return f"E bad_command:{cmd}"

            return self.handle_trial_move(b_mag=b_mag, e_mag=e_mag)

        m_single = self._SINGLE_MOVE_RE.fullmatch(cmd)
        if m_single:
            return self.handle_payload_command(cmd)

        m_multi_single = self._MULTI_SINGLE_MOVE_RE.fullmatch(cmd)
        if m_multi_single:
            return self.handle_payload_command(cmd)

        m_single_vel = self._SINGLE_VEL_RE.fullmatch(cmd)
        if m_single_vel:
            return self.handle_payload_command(cmd)

        if cmd_lower.startswith("pl:"):
            preset_name = cmd[3:].strip()
            return self.handle_payload_preset(preset_name)

        if self._looks_like_payload(cmd):
            return self.handle_payload_command(cmd)
    

        return f"E unknown_command:{cmd_upper}"
