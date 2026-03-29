import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

import sys
import threading
import traceback
import queue
import time

mode = sys.argv[1].strip().lower() if len(sys.argv) > 1 and sys.argv[1].strip() else "match"

from shared_state import SharedState
from config import HX_PORT, HX_BAUD, HX_TIMEOUT, DT_LOOP


def run_thread(name, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except Exception:
        traceback.print_exc()
        raise


def main():
    state = SharedState()
    threads = []

    # ---------- HX711 thread ----------
    if mode not in ("unity_demo", "unitydemo", "demo"):
        from hx711_reader import hx711_reader

        threads.append(
            threading.Thread(
                target=run_thread,
                args=("HX711", hx711_reader, state, HX_PORT, HX_BAUD, HX_TIMEOUT),
                daemon=True,
            )
        )

    for t in threads:
        t.start()

    # ---------- INTERACTIVE MODE ----------
    def interactive_loop():
        from dynamixel_keyboard_pos_control import KeyboardPosController
        from dynamixel_free_force_control import FreeForceController
        from dynamixel_stiffness_control import StiffnessController

        cmd_q = queue.Queue()

        def stdin_reader():
            while not state.stop:
                try:
                    line = input()
                except EOFError:
                    state.stop = True
                    break
                cmd_q.put(line)

        threading.Thread(target=run_thread, args=("STDIN", stdin_reader), daemon=True).start()

        state.active_mode = "pos"
        current = KeyboardPosController(state)

        def switch_to(new_mode: str):
            nonlocal current

            if new_mode == state.active_mode:
                return

            try:
                current.close()
            except Exception:
                pass

            state.active_mode = new_mode

            if new_mode == "free":
                current = FreeForceController(state)
            elif new_mode == "stiff":
                current = StiffnessController(state)
            else:
                current = KeyboardPosController(state)

        print("\n[Interactive] Ready. Keys: p=POS, m=FREE, k=STIFF, q=quit")

        try:
            while not state.stop:
                while not cmd_q.empty():
                    line = cmd_q.get()
                    cmd = line.strip().lower()

                    if cmd in ("q", "quit", "exit"):
                        state.stop = True
                        break

                    if cmd in ("p", "pos"):
                        switch_to("pos")
                        continue

                    if cmd in ("m", "free"):
                        switch_to("free")
                        continue

                    if cmd in ("k", "stiff"):
                        switch_to("stiff")
                        continue

                    act = current.handle_line(line)
                    if act == "quit":
                        state.stop = True
                        break

                if state.active_mode in ("free", "stiff"):
                    current.step()
                    time.sleep(float(DT_LOOP))
                else:
                    time.sleep(0.01)

        finally:
            try:
                current.close()
            except Exception:
                pass

    # ---------- MAIN MODE SWITCH ----------
    try:

        # ===== UNITY DEMO (最重要) =====
        if mode in ("unity_demo", "unitydemo", "demo"):

            # ★ Boot signal FIRST
            print("K", flush=True)

            try:
                from unity_demo_controller import UnityDemoController
                ctrl = UnityDemoController(state)
            except Exception as e:
                traceback.print_exc()
                print(f"E init_failed:{e}", flush=True)
                return

            try:
                while not state.stop:
                    try:
                        line = input()
                    except EOFError:
                        break

                    out = ctrl.handle_line(line)
                    if out:
                        print(out, flush=True)

                    if out == "B":
                        break
            finally:
                ctrl.close()

        # ===== INTERACTIVE =====
        elif mode in ("interactive", "posfree", "pf"):
            interactive_loop()

        # ===== POSITION =====
        elif mode in ("pos", "position", "keyboard"):
            from dynamixel_keyboard_pos_control import dynamixel_keyboard_pos_control
            dynamixel_keyboard_pos_control(state)

        # ===== STIFFNESS =====
        elif mode in ("stiff", "stiffness"):
            from dynamixel_stiffness_control import dynamixel_stiffness_control
            dynamixel_stiffness_control(state)

        # ===== FREE MODE =====
        elif mode in ("free", "freemode"):
            from dynamixel_free_force_control import dynamixel_free_force_control
            dynamixel_free_force_control(state)

        # ===== DEFAULT MATCH =====
        else:
            from dynamixel_match_control import dynamixel_match_control
            dynamixel_match_control(state)

    except KeyboardInterrupt:
        state.stop = True

    finally:
        state.stop = True
        for t in threads:
            t.join(timeout=0.5)


if __name__ == "__main__":
    main()