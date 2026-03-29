"""
HX711 (measured force) reader.

Update (2026-01):
- One serial port streams 3 comma-separated values per line: f1,f2,f3
  These correspond to the 3 motor loops (index 0/1/2).
- Robust to spaces and occasional malformed lines.
"""

import serial
from shared_state import SharedState
from config import HX_MAX_VALUES

def _parse_csv_floats(line: str):
    parts = [p.strip() for p in line.split(",") if p.strip() != ""]
    out = []
    for p in parts[:HX_MAX_VALUES]:
        try:
            out.append(float(p))
        except ValueError:
            return None
    return out if out else None

def hx711_reader(state: SharedState, port: str, baud: int, timeout_s: float):
    ser = serial.Serial(port, baud, timeout=timeout_s)
    ser.reset_input_buffer()

    while True:
        if state.stop:
            break
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        vals = _parse_csv_floats(line)
        if vals is None:
            continue
        state.set_hx3(vals)

    try:
        ser.close()
    except Exception:
        pass
