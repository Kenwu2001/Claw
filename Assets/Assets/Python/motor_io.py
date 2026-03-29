"""
DC motor/encoder IO on MOTOR_PORT (COM21 in your script).

- motor_reader: reads "E,enc1,enc2,enc3"
- send_override / send_cmd: writes commands
"""
import serial
import time
from shared_state import SharedState

def motor_reader(state: SharedState, ser: serial.Serial):
    ser.reset_input_buffer()
    while True:
        if state.stop:
            break
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if not line or not line.startswith("E,"):
                continue
            parts = line.split(",")
            if len(parts) != 4:
                continue
            enc1 = int(parts[1]); enc2 = int(parts[2]); enc3 = int(parts[3])
            state.set_enc(enc1, enc2, enc3)
        except Exception:
            continue

def send_override(motor_ser: serial.Serial, pwm1: int, pwm2: int, duration_s: float):
    dur_ms = int(max(0, duration_s) * 1000)
    cmd = f"C,{pwm1},{pwm2},{dur_ms}\n"
    motor_ser.write(cmd.encode("ascii"))
    motor_ser.flush()

def send_cmd(motor_ser: serial.Serial, cmd: bytes):
    motor_ser.write(cmd)
    motor_ser.flush()
