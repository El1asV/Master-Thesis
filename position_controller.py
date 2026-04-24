"""
Architecture:
  - ODrive in VELOCITY_CONTROL mode
  - AksIM-2 on gearbox output provides position feedback
  - Python PID closes the position loop by commanding velocity
"""

import serial
import threading
import time
import sys
from collections import deque
import odrive
from odrive.enums import AxisState, ControlMode, InputMode

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

SERIAL_PORT = "COM5"
BAUD_RATE   = 115_200

# Initial target position
TARGET_DEG  = 0.0

GEAR_RATIO  = 1 + 80 / 12

TORSIONAL_STIFFNESS = 35
MOTOR_KT            = 0.083  # Nm/A — torque constant (Kv=100 rpm/V → Kt=60/(2π×100)≈0.083)

# No-Go-Zone — targets inside [FORBIDDEN_MIN, FORBIDDEN_MAX] are rejected.
FORBIDDEN_MIN = 225.0
FORBIDDEN_MAX = 315.0

MAX_PLOT_POINTS = 600   # 30 s of history at 20 Hz

KP = 0.8
KI = 0.25
KD = 0.0

# Motor direction: +1 or -1.
MOTOR_DIRECTION = -1

# Safety limits
MAX_VELOCITY_TURNS_S = 3.0   # hard cap on ODrive velocity command [turns/s]
DEADBAND_DEG         = 0.1

# Control loop rate
LOOP_HZ = 20


# ---------------------------------------------------------------------------
# Encoder reader (background thread)
# ---------------------------------------------------------------------------

class AksimReader:
    """Thread-safe reader for the Arduino CSV stream."""

    def __init__(self, port: str, baud: int = 115_200):
        self._ser    = serial.Serial(port, baud, timeout=1.0)
        self._lock   = threading.Lock()
        self._degrees:  float | None = None
        self._position: int   | None = None
        self._error:    bool  = False
        self._warning:  bool  = False
        self._running   = False
        self._thread: threading.Thread | None = None

    def start(self):
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True, name="aksim-reader")
        self._thread.start()

    def stop(self):
        self._running = False
        try:
            self._ser.close()
        except Exception:
            pass

    def _loop(self):
        while self._running:
            try:
                raw  = self._ser.readline()
                line = raw.decode("ascii", errors="ignore").strip()
            except Exception:
                continue
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 5:
                continue
            try:
                pos  = int(parts[0])
                deg  = float(parts[1])
                err  = bool(int(parts[2]))
                warn = bool(int(parts[3]))
            except ValueError:
                continue
            with self._lock:
                self._position = pos
                self._degrees  = deg
                self._error    = err
                self._warning  = warn

    @property
    def degrees(self) -> float | None:
        with self._lock:
            return self._degrees

    @property
    def error(self) -> bool:
        with self._lock:
            return self._error

    def wait_for_data(self, timeout: float = 5.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.degrees is not None:
                return True
            time.sleep(0.05)
        return False


# ---------------------------------------------------------------------------
# PID controller with wrap-around error and anti-windup
# ---------------------------------------------------------------------------

class PID:
    def __init__(self, kp: float, ki: float, kd: float, limit: float):
        self.kp    = kp
        self.ki    = ki
        self.kd    = kd
        self.limit = limit
        self._integral   = 0.0
        self._prev_error: float | None = None
        self._prev_time:  float | None = None

    def reset(self):
        self._integral   = 0.0
        self._prev_error = None
        self._prev_time  = None

    def compute_from_error(self, error: float) -> float:
        """Run the PID with a pre-computed signed error. Returns output."""
        now = time.monotonic()
        dt  = (now - self._prev_time) if self._prev_time is not None else (1.0 / LOOP_HZ)

        int_limit = self.limit / self.ki if self.ki > 1e-9 else float("inf")
        self._integral += error * dt
        self._integral  = max(-int_limit, min(int_limit, self._integral))

        derivative = 0.0
        if self._prev_error is not None and dt > 0:
            derivative = (error - self._prev_error) / dt

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        output = max(-self.limit, min(self.limit, output))

        self._prev_error = error
        self._prev_time  = now
        return output

    def compute(self, setpoint: float, measured: float) -> tuple[float, float]:
        """Returns (output, error_deg). Error wrapped to [-180, 180]."""
        error = setpoint - measured
        error = (error + 180.0) % 360.0 - 180.0
        return self.compute_from_error(error), error


# ---------------------------------------------------------------------------
# ODrive helpers
# ---------------------------------------------------------------------------

def connect_odrive():
    print("Searching for ODrive (USB)…")
    odrv = odrive.find_any(timeout=15)
    print(f"Connected to ODrive  serial={hex(odrv.serial_number)}")
    return odrv


def _read(obj, *attrs, default=0):
    for a in attrs:
        try:
            return getattr(obj, a)
        except AttributeError:
            pass
    return default


def dump_odrive_errors(odrv) -> bool:
    axis  = odrv.axis0
    found = False
    for label, obj in [("system", odrv), ("axis0", axis),
                        ("motor", axis.motor), ("controller", axis.controller)]:
        code = _read(obj, "active_errors", "error", default=0)
        if code:
            print(f"  {label} active_errors: 0x{code:08X}")
            found = True
    try:
        enc_err = _read(axis.encoder, "active_errors", "error", default=0)
        if enc_err:
            print(f"  encoder active_errors: 0x{enc_err:08X}")
            found = True
    except AttributeError:
        pass
    disarm = _read(axis, "disarm_reason", default=None)
    if disarm is not None and disarm != 0:
        print(f"  disarm_reason: {disarm}")
        found = True
    proc = _read(axis, "procedure_result", default=None)
    if proc is not None and proc not in (0, 1):
        labels = {2: "Cancelled", 3: "No response", 4: "Pole pairs mismatch",
                  5: "Phase resistance out of range", 6: "Phase inductance out of range",
                  10: "Timeout"}
        print(f"  procedure_result: {proc} ({labels.get(proc, 'unknown')})")
        found = True
    if not found:
        print("  All OK.")
    return found


def setup_odrive(odrv) -> None:
    """Calibrate if needed, then enter closed-loop velocity control."""
    axis = odrv.axis0

    print("Checking ODrive errors…")
    dump_odrive_errors(odrv)
    odrv.clear_errors()

    axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
    axis.controller.config.input_mode   = InputMode.PASSTHROUGH

    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    time.sleep(0.8)

    if axis.current_state == AxisState.CLOSED_LOOP_CONTROL:
        print("ODrive: closed-loop velocity control active.")
        return

    print(f"Closed-loop not reached (state={axis.current_state}). Errors:")
    dump_odrive_errors(odrv)

    ans = input("\nRun full calibration? Motor will spin briefly. (y/N): ").strip().lower()
    if ans != "y":
        raise RuntimeError("ODrive not calibrated. Run calibration in odrivetool and retry.")

    print("Running full calibration sequence…")
    odrv.clear_errors()
    axis.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    for _ in range(60):
        time.sleep(0.5)
        if axis.current_state == AxisState.IDLE:
            break
    else:
        raise RuntimeError("Calibration timed out.")

    if dump_odrive_errors(odrv):
        raise RuntimeError("Errors remain after calibration.")

    axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
    axis.controller.config.input_mode   = InputMode.PASSTHROUGH

    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    time.sleep(0.8)

    if axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
        dump_odrive_errors(odrv)
        raise RuntimeError(f"Still could not enter closed-loop (state={axis.current_state}).")

    print("ODrive: closed-loop velocity control active.")


def stop_odrive(odrv) -> None:
    try:
        odrv.axis0.controller.input_vel = 0.0
        odrv.axis0.requested_state = AxisState.IDLE
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Live plot window (opened on demand by typing 'p')
# ---------------------------------------------------------------------------

class TorqueWindow:
    """
    Tkinter window showing live Iq and τ_Iq values.
    Runs its own Tk mainloop in a daemon thread — never blocks the control loop.
    Pass the same deque the main loop appends (elapsed_s, iq_A, tau_iq_Nm) to.
    """

    def __init__(self, data: deque):
        self._data = data
        self._thread = threading.Thread(target=self._run, daemon=True, name="torque-window")
        self._thread.start()

    def _run(self):
        import tkinter as tk
        from tkinter import font as tkfont

        root = tk.Tk()
        root.title("ODrive — Iq & τ_Iq")
        root.resizable(False, False)

        pad  = {"padx": 16, "pady": 6}
        big  = tkfont.Font(family="Consolas", size=36, weight="bold")
        med  = tkfont.Font(family="Consolas", size=18)
        small = tkfont.Font(family="Consolas", size=11)

        # Iq
        tk.Label(root, text="Iq  (A)", font=small).grid(
            row=0, column=0, sticky="w", **pad)
        iq_var = tk.StringVar(value="---")
        tk.Label(root, textvariable=iq_var, font=big, width=12,
                 anchor="e", fg="#0066cc").grid(row=1, column=0, **pad)

        # τ_Iq
        tk.Label(root, text="τ_Iq  (Nm)", font=small).grid(
            row=2, column=0, sticky="w", **pad)
        tau_var = tk.StringVar(value="---")
        tk.Label(root, textvariable=tau_var, font=big, width=12,
                 anchor="e", fg="#cc6600").grid(row=3, column=0, **pad)

        # Elapsed time
        tk.Label(root, text="Elapsed  (s)", font=small).grid(
            row=4, column=0, sticky="w", **pad)
        time_var = tk.StringVar(value="---")
        tk.Label(root, textvariable=time_var, font=med, width=12,
                 anchor="e").grid(row=5, column=0, **pad)

        def _poll():
            if self._data:
                elapsed, iq, tau = self._data[-1]
                iq_var.set(f"{iq:>+10.3f}")
                tau_var.set(f"{tau:>+10.3f}")
                time_var.set(f"{elapsed:>10.1f}")
            root.after(200, _poll)

        root.after(200, _poll)
        root.mainloop()


# ---------------------------------------------------------------------------
# Target-input thread
# ---------------------------------------------------------------------------

def in_forbidden_zone(deg: float) -> bool:
    return FORBIDDEN_MIN <= deg <= FORBIDDEN_MAX


def path_crosses_forbidden(start: float, delta: float) -> bool:
    """Return True if the arc from start by delta degrees overlaps the forbidden zone."""
    if abs(delta) < 1e-3:
        return in_forbidden_zone(start % 360.0)
    arc_start = start % 360.0
    arc_end   = arc_start + delta          
    lo, hi    = min(arc_start, arc_end), max(arc_start, arc_end)
    for offset in (0.0, 360.0, -360.0):
        if (FORBIDDEN_MAX + offset) >= lo and (FORBIDDEN_MIN + offset) <= hi:
            return True
    return False


def safe_angular_error(target: float, current: float) -> float:
    """
    Return the signed angular error that routes around the forbidden zone.
    If the shortest path crosses [FORBIDDEN_MIN, FORBIDDEN_MAX] the longer
    path (opposite direction) is returned instead.
    """
    short = (target - current + 180.0) % 360.0 - 180.0
    if not path_crosses_forbidden(current, short):
        return short
    
    long = short - 360.0 * (1.0 if short >= 0.0 else -1.0)
    return long


def input_thread(shared: dict) -> None:
    print(f"  Type a new target angle (degrees) and press Enter at any time.")
    print(f"  Type 'z' + Enter to zero the compliance offset.")
    print(f"  Type 'p' + Enter to open the Iq / τ_Iq live plot window.")
    print(f"  Forbidden zone: {FORBIDDEN_MIN}°–{FORBIDDEN_MAX}° (targets in this range are rejected).")
    while shared.get("running", True):
        try:
            raw = input().strip()
            if raw.lower() == "z":
                shared["zero_compliance"] = True
                print("  → Compliance offset zeroed.")
                continue
            if raw.lower() == "p":
                shared["open_plot"] = True
                print("  → Opening plot window…")
                continue
            val = float(raw) % 360.0
            if in_forbidden_zone(val):
                print(f"  ✗ {val:.2f}° is inside the forbidden zone "
                      f"({FORBIDDEN_MIN}°–{FORBIDDEN_MAX}°) — command ignored.")
            else:
                shared["target"] = val
                print(f"  → Target updated to {val:.2f}°")
        except ValueError:
            print("  (not a number, ignored)")
        except EOFError:
            break


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    shared = {
        "target":          TARGET_DEG % 360.0,
        "running":         True,
        "zero_compliance": False,
        "open_plot":       False,
        "plot_data":       deque(maxlen=MAX_PLOT_POINTS),
        "_plot_window":    None,
    }

    print(f"Opening AksIM-2 encoder on {SERIAL_PORT} …")
    enc = AksimReader(SERIAL_PORT, BAUD_RATE)
    enc.start()

    if not enc.wait_for_data(timeout=6.0):
        print("ERROR: No data from encoder after 6 s. Check COM5 and Arduino.")
        enc.stop()
        sys.exit(1)

    print(f"Encoder online — current position: {enc.degrees:.3f}°")
    if enc.error:
        print("WARNING: Encoder error flag is SET. Verify wiring / CRC.")

    odrv = connect_odrive()
    setup_odrive(odrv)

    pid = PID(KP, KI, KD, MAX_VELOCITY_TURNS_S)

    inp = threading.Thread(target=input_thread, args=(shared,), daemon=True)
    inp.start()

    dt = 1.0 / LOOP_HZ

    print()
    print(f"  KP={KP}  KI={KI}  KD={KD}  MAX_VEL={MAX_VELOCITY_TURNS_S} t/s  DEADBAND={DEADBAND_DEG}°")
    def _find_attr(root, candidates):
        for _attr in candidates:
            obj = root
            try:
                for part in _attr.split("."):
                    obj = getattr(obj, part)
                return _attr
            except AttributeError:
                pass
        return None

    def _read_attr(root, attr):
        if attr is None:
            return None
        obj = root
        try:
            for part in attr.split("."):
                obj = getattr(obj, part)
            return float(obj)
        except Exception:
            return None

    _iq_attr   = _find_attr(odrv.axis0,
                    ("motor.current_control.Iq_measured", "motor.foc.Iq_measured"))
    _temp_attr = _find_attr(odrv.axis0,
                    ("motor.motor_thermistor.temperature", "motor_thermistor.temperature"))

    print(f"  Iq readback:   {'axis0.' + _iq_attr   if _iq_attr   else 'unavailable'}")
    print(f"  Temp readback: {'axis0.' + _temp_attr if _temp_attr else 'unavailable'}")
    if _temp_attr is None:
        print("  NOTE: thermistor not detected — check GPIO pin config in odrivetool.")

    def read_iq(odrv)   -> float | None: return _read_attr(odrv.axis0, _iq_attr)
    def read_temp(odrv) -> float | None: return _read_attr(odrv.axis0, _temp_attr)

    import math

    def read_pos(odrv) -> float | None:
        """Return motor position in turns from axis0.pos_estimate, or None if nan."""
        try:
            v = odrv.axis0.pos_estimate
            return None if (v is None or math.isnan(v)) else float(v)
        except Exception:
            return None


    _pos_ref: float | None = None
    for _ in range(10):
        p = read_pos(odrv)
        if p is not None:
            _pos_ref = p
            break
        time.sleep(0.1)

    _output_ref_deg    = enc.degrees
    _compliance_offset = 0.0

    if _pos_ref is not None:
        print(f"  Compliance ref: output={_output_ref_deg:.3f}°, motor pos={_pos_ref:.4f} turns")
    else:
        print("  WARNING: pos_estimate is nan — compliance (δ) will show n/a.")
    print(f"  Type 'z' + Enter to zero the compliance offset at any time.")

    def compliance_delta(output_deg: float) -> float | None:
        """
        Returns the torsional compliance angle δ in degrees at the output shaft.
        δ > 0: output lags behind input prediction (load in positive direction).
        δ < 0: output leads (load in negative direction).
        """
        if _pos_ref is None:
            return None
        p = read_pos(odrv)
        if p is None:
            return None
        motor_turns = p - _pos_ref
        predicted = _output_ref_deg + (MOTOR_DIRECTION * motor_turns / GEAR_RATIO) * 360.0
        raw = predicted - output_deg
        return (raw + 180.0) % 360.0 - 180.0 - _compliance_offset

    print()
    print(f"{'Elapsed':>9}  {'Target':>9}  {'Actual':>9}  {'Error':>8}  "
          f"{'Vel cmd':>12}  {'δ (deg)':>8}  {'τ_δ (Nm)':>11}  {'Iq (A)':>9}  {'τ_Iq (Nm)':>11}  {'Temp °C':>10}")
    print("-" * 114)

    t_start  = time.monotonic()
    disarmed = False

    try:
        while True:
            t0 = time.monotonic()

            target_deg  = shared["target"]
            current_deg = enc.degrees

            if current_deg is None:
                print("\nERROR: Encoder data lost — stopping motor.")
                break
            if enc.error:
                print("\nWARNING: Encoder error flag active — stopping motor.")
                break

            if in_forbidden_zone(current_deg):
                if not disarmed:
                    odrv.axis0.controller.input_vel = 0.0
                    odrv.axis0.requested_state = AxisState.IDLE
                    pid.reset()
                    disarmed = True
                print(f"  FORBIDDEN ZONE ({current_deg:.2f}°) — motor deactivated.     ", end="\r")
                sleep_t = dt - (time.monotonic() - t0)
                if sleep_t > 0:
                    time.sleep(sleep_t)
                continue

            if disarmed:
                odrv.clear_errors()
                odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
                time.sleep(0.3)
                if odrv.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL:
                    print("\nERROR: Could not re-enter closed-loop after forbidden zone.")
                    break
                disarmed = False
                print(f"\nLeft forbidden zone ({current_deg:.2f}°) — motor re-activated.")


            error_deg = safe_angular_error(target_deg, current_deg)
            vel_cmd   = pid.compute_from_error(error_deg)

            if abs(error_deg) < DEADBAND_DEG:
                vel_cmd = 0.0
                pid.reset()

            odrv.axis0.controller.input_vel = MOTOR_DIRECTION * vel_cmd

            iq   = read_iq(odrv)
            temp = read_temp(odrv)

            delta = compliance_delta(current_deg)

            if shared.pop("zero_compliance", False) and delta is not None:
                _compliance_offset += delta
                delta = 0.0
            torque_iq = MOTOR_KT * iq * GEAR_RATIO if iq is not None else None
            torque_comp = (TORSIONAL_STIFFNESS * delta
                           if (TORSIONAL_STIFFNESS is not None and delta is not None)
                           else None)

            iq_str        = f"{iq:7.3f}"         if iq          is not None else "    n/a"
            temp_str      = f"{temp:7.1f}"        if temp        is not None else "    n/a"
            delta_str     = f"{delta:7.2f}"       if delta       is not None else "   n/a"
            torque_iq_str = f"{torque_iq:8.3f}"   if torque_iq   is not None else "     n/a"
            torque_cp_str = f"{torque_comp:8.3f}" if torque_comp is not None else "     n/a"

            elapsed = time.monotonic() - t_start
            if iq is not None and torque_iq is not None:
                shared["plot_data"].append((elapsed, iq, torque_iq))
            if shared.pop("open_plot", False) and shared["_plot_window"] is None:
                shared["_plot_window"] = TorqueWindow(shared["plot_data"])

            print(
                f"{elapsed:8.1f}s  {target_deg:8.2f}°  {current_deg:8.2f}°  "
                f"{error_deg:7.2f}°  {vel_cmd:8.3f} t/s  {delta_str}°  "
                f"{torque_cp_str} Nm  {iq_str} A  {torque_iq_str} Nm  {temp_str} °C",
                end="\r",
            )

            sleep_t = dt - (time.monotonic() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt — shutting down.")
    finally:
        shared["running"] = False
        stop_odrive(odrv)
        enc.stop()
        print("Motor stopped, encoder closed. Goodbye.")


if __name__ == "__main__":
    main()
