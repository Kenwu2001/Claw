"""
Thread-safe shared state.

Update (2026-01):
- PaXini: 3 sensors, each updated independently by its own serial reader (one port -> one sensor).
  Each sensor updates 5 values:
    Fz, Ft, theta_deg, sliding, slip_window_mm
- HX711: 3 measured forces per line from one serial port (e.g., "0.11,0.22,0.33").
"""

from dataclasses import dataclass, field
import threading
import time
from typing import Optional, List, Dict


@dataclass
class SharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)

    # -----------------------------
    # PaXini (targets)
    # -----------------------------
    # Backward-compatible scalar fields mirroring sensor index 0.
    Fz: Optional[float] = None
    Ft: Optional[float] = None
    theta_deg: Optional[float] = None
    sliding: Optional[bool] = None
    slip_window_mm: Optional[float] = None

    # Per-sensor arrays (length=3)
    Fz3: List[Optional[float]] = field(default_factory=lambda: [None, None, None])
    Ft3: List[Optional[float]] = field(default_factory=lambda: [None, None, None])
    theta3_deg: List[Optional[float]] = field(default_factory=lambda: [None, None, None])
    sliding3: List[Optional[bool]] = field(default_factory=lambda: [None, None, None])
    slip_window3_mm: List[Optional[float]] = field(default_factory=lambda: [None, None, None])

    t_sensor: float = 0.0

    # -----------------------------
    # HX711 (measured forces)
    # -----------------------------
    # Backward-compatible scalar mirroring index 0.
    F_meas: float = 0.0
    # Per-motor measured forces (length=3)
    F_meas3: List[float] = field(default_factory=lambda: [0.0]*9)  # up to 9 channels by default
    t_hx: float = 0.0

    # -----------------------------
    # Motor encoders (optional)
    # -----------------------------
    enc1: int = 0
    enc2: int = 0
    enc3: int = 0
    t_enc: float = 0.0

    # Stop flag
    stop: bool = False

    # -----------------------------
    # Interactive mode state
    # -----------------------------
    # active_mode: 'pos' or 'free' (used by interactive main loop)
    active_mode: str = "pos"
    # baseline ticks for keyboard absolute commands (0 deg reference)
    baseline_ticks: Dict[int, int] = field(default_factory=dict)


    # FREE mode runtime tunables (can be changed via keyboard commands in interactive mode)
    free_target_n: float = 0.0
    free_kp_pos: float = 0.007
    free_kp_neg: float = 0.01
    free_kd: float = 0.0
    free_params_initialized: bool = False

    # STIFFNESS mode runtime tunables
    stiffness_enable_damper: bool = False
    stiffness_k_n_per_m: float = 400.0
    stiffness_b_n_s_per_m: float = 80.0
    stiffness_goal_must_be_leq_p0: bool = True
    stiffness_params_initialized: bool = False

    # -----------------------------
    # Helper methods
    # -----------------------------
    def set_paxini(
        self,
        sensor_index: int,
        Fz: float,
        Ft: float,
        theta_deg: float,
        sliding: bool,
        slip_window_mm: float,
    ):
        """Update PaXini data for one sensor index (0/1/2)."""
        if sensor_index < 0 or sensor_index >= 3:
            return
        with self.lock:
            self.Fz3[sensor_index] = float(Fz)
            self.Ft3[sensor_index] = float(Ft)
            self.theta3_deg[sensor_index] = float(theta_deg)
            self.sliding3[sensor_index] = bool(sliding)
            self.slip_window3_mm[sensor_index] = float(slip_window_mm)

            # Mirror sensor 0 to scalar fields for backward compatibility.
            if sensor_index == 0:
                self.Fz = float(Fz)
                self.Ft = float(Ft)
                self.theta_deg = float(theta_deg)
                self.sliding = bool(sliding)
                self.slip_window_mm = float(slip_window_mm)

            self.t_sensor = time.time()

    def set_hx3(self, forces: List[float]):
        """Update measured forces from HX711 line values (length up to 3)."""
        with self.lock:
            for i in range(min(len(self.F_meas3), len(forces))):
                self.F_meas3[i] = float(forces[i])
            # Mirror index 0 for backward compatibility
            self.F_meas = self.F_meas3[0]
            self.t_hx = time.time()

    def set_enc(self, enc1: int, enc2: int, enc3: int):
        with self.lock:
            self.enc1, self.enc2, self.enc3 = int(enc1), int(enc2), int(enc3)
            self.t_enc = time.time()
