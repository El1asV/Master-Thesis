"""
Output-side position controller — velocity control.

Author:  Elias Vestli
Project: Torque Estimation for Robotic Actuators Using Gearbox Torsional Compliance — Master's Thesis
Content: Closes a position loop using an AksIM-2 encoder on the gearbox output
         and an ODrive S1 in velocity-control mode. Includes forbidden-zone
         rejection and torsional compliance estimation.

"""

import serial
import threading
import time
import sys
from collections import deque
import odrive
from odrive.enums import AxisState, ControlMode, InputMode


SERIAL_PORT = "COM5"
BAUD_RATE   = 115_200

TARGET_DEG  = 0.0

GEAR_RATIO  = 1 + 80 / 12

TORSIONAL_STIFFNESS = 35
MOTOR_KT            = 0.083

FORBIDDEN_MIN = 225.0
FORBIDDEN_MAX = 315.0

MAX_PLOT_POINTS = 600

KP = 0.8
KI = 0.25
KD = 0.0

MOTOR_DIRECTION = -1

MAX_VELOCITY_TURNS_S = 3.0
DEADBAND_DEG         = 0.1

LOOP_HZ = 20



class AksimReader:
    """Thread-safe serial reader for the AksIM-2 encoder Arduino stream.

    Parameters
    ----------
    port : str
        Serial port identifier (e.g. ``"COM5"``).
    baud : int, optional
        Baud rate. Defaults to 115 200.
    """

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
        """Start the background reader thread."""
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True, name="aksim-reader")
        self._thread.start()

    def stop(self):
        """Stop the reader thread and close the serial port."""
        self._running = False
        try:
            self._ser.close()
        except Exception:
            pass

    def _loop(self):
        """Parse CSV lines from the serial stream and update shared state."""
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
        """Current shaft angle in degrees, or ``None`` before the first valid frame."""
        with self._lock:
            return self._degrees

    @property
    def error(self) -> bool:
        """``True`` if the encoder reports an error condition."""
        with self._lock:
            return self._error

    def wait_for_data(self, timeout: float = 5.0) -> bool:
        """Block until a valid angle is received or the timeout expires.

        Parameters
        ----------
        timeout : float, optional
            Maximum wait time in seconds. Defaults to 5.0.

        Returns
        -------
        bool
            ``True`` if data arrived within *timeout*, ``False`` otherwise.
        """
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.degrees is not None:
                return True
            time.sleep(0.05)
        return False



class PID:
    """Discrete PID controller with integral anti-windup and output clamping.

    Parameters
    ----------
    kp : float
        Proportional gain.
    ki : float
        Integral gain.
    kd : float
        Derivative gain.
    limit : float
        Symmetric output clamp (±limit).
    """

    def __init__(self, kp: float, ki: float, kd: float, limit: float):
        self.kp    = kp
        self.ki    = ki
        self.kd    = kd
        self.limit = limit
        self._integral   = 0.0
        self._prev_error: float | None = None
        self._prev_time:  float | None = None

    def reset(self):
        """Clear the integral accumulator and derivative history."""
        self._integral   = 0.0
        self._prev_error = None
        self._prev_time  = None

    def compute_from_error(self, error: float) -> float:
        """Compute the PID output from a pre-computed signed error.

        Parameters
        ----------
        error : float
            Signed control error.

        Returns
        -------
        float
            Clamped velocity command [turns/s].
        """
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
        """Compute the PID output; angular error is wrapped to [-180, 180] degrees.

        Parameters
        ----------
        setpoint : float
            Desired position [deg].
        measured : float
            Current position [deg].

        Returns
        -------
        tuple[float, float]
            ``(output, error_deg)`` where *output* is clamped [turns/s].
        """
        error = setpoint - measured
        error = (error + 180.0) % 360.0 - 180.0
        return self.compute_from_error(error), error



def connect_odrive():
    """Find and return the first available ODrive over USB.

    Returns
    -------
    odrive.ODrive
        Connected ODrive object.
    """
    print("Searching for ODrive (USB)…")
    odrv = odrive.find_any(timeout=15)
    print(f"Connected to ODrive  serial={hex(odrv.serial_number)}")
    return odrv


def _read(obj, *attrs, default=0):
    """Return the first successfully read attribute from *obj*, or *default*.

    Parameters
    ----------
    obj : object
        Target object.
    *attrs : str
        Attribute names to try in order.
    default : any, optional
        Fallback value when all attributes fail. Defaults to 0.

    Returns
    -------
    any
        First successfully read attribute value, or *default*.
    """
    for a in attrs:
        try:
            return getattr(obj, a)
        except AttributeError:
            pass
    return default


def dump_odrive_errors(odrv) -> bool:
    """Print active ODrive errors to stdout.

    Parameters
    ----------
    odrv : odrive.ODrive
        Connected ODrive object.

    Returns
    -------
    bool
        ``True`` if any errors were found.
    """
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
    """Calibrate if needed, then enter closed-loop velocity control.

    Parameters
    ----------
    odrv : odrive.ODrive
        Connected ODrive object.
    """
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
    """Command zero velocity and set the axis to IDLE.

    Parameters
    ----------
    odrv : odrive.ODrive
        Connected ODrive object.
    """
    try:
        odrv.axis0.controller.input_vel = 0.0
        odrv.axis0.requested_state = AxisState.IDLE
    except Exception:
        pass



class TorqueWindow:
    """Tkinter window showing live Iq and τ_Iq values in a daemon thread.

    Parameters
    ----------
    data : deque
        Shared deque of ``(elapsed_s, iq_A, tau_iq_Nm)`` tuples.
    """

    def __init__(self, data: deque):
        self._data = data
        self._thread = threading.Thread(target=self._run, daemon=True, name="torque-window")
        self._thread.start()

    def _run(self):
        """Build the Tk window and start its event loop."""
        import tkinter as tk
        from tkinter import font as tkfont

        root = tk.Tk()
        root.title("ODrive — Iq & τ_Iq")
        root.resizable(False, False)

        pad   = {"padx": 16, "pady": 6}
        big   = tkfont.Font(family="Consolas", size=36, weight="bold")
        med   = tkfont.Font(family="Consolas", size=18)
        small = tkfont.Font(family="Consolas", size=11)

        tk.Label(root, text="Iq  (A)", font=small).grid(row=0, column=0, sticky="w", **pad)
        iq_var = tk.StringVar(value="---")
        tk.Label(root, textvariable=iq_var, font=big, width=12,
                 anchor="e", fg="#0066cc").grid(row=1, column=0, **pad)

        tk.Label(root, text="τ_Iq  (Nm)", font=small).grid(row=2, column=0, sticky="w", **pad)
        tau_var = tk.StringVar(value="---")
        tk.Label(root, textvariable=tau_var, font=big, width=12,
                 anchor="e", fg="#cc6600").grid(row=3, column=0, **pad)

        tk.Label(root, text="Elapsed  (s)", font=small).grid(row=4, column=0, sticky="w", **pad)
        time_var = tk.StringVar(value="---")
        tk.Label(root, textvariable=time_var, font=med, width=12,
                 anchor="e").grid(row=5, column=0, **pad)

        def _poll():
            """Refresh displayed values from the latest data point."""
            if self._data:
                elapsed, iq, tau = self._data[-1]
                iq_var.set(f"{iq:>+10.3f}")
                tau_var.set(f"{tau:>+10.3f}")
                time_var.set(f"{elapsed:>10.1f}")
            root.after(200, _poll)

        root.after(200, _poll)
        root.mainloop()



def in_forbidden_zone(deg: float) -> bool:
    """Return ``True`` if *deg* falls within the configured forbidden zone.

    Parameters
    ----------
    deg : float
        Shaft angle in degrees.

    Returns
    -------
    bool
    """
    return FORBIDDEN_MIN <= deg <= FORBIDDEN_MAX


def path_crosses_forbidden(start: float, delta: float) -> bool:
    """Return ``True`` if the arc from *start* by *delta* degrees overlaps the forbidden zone.

    Parameters
    ----------
    start : float
        Arc start angle [deg].
    delta : float
        Arc extent [deg]; sign indicates direction.

    Returns
    -------
    bool
    """
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
    """Return the signed angular error routed around the forbidden zone.

    Parameters
    ----------
    target : float
        Desired position [deg].
    current : float
        Current position [deg].

    Returns
    -------
    float
        Signed error [deg]; the longer arc is used when the shortest path
        crosses the forbidden zone.
    """
    short = (target - current + 180.0) % 360.0 - 180.0
    if not path_crosses_forbidden(current, short):
        return short
    long = short - 360.0 * (1.0 if short >= 0.0 else -1.0)
    return long


def input_thread(shared: dict) -> None:
    """Read target angles and commands from stdin in a daemon thread.

    Parameters
    ----------
    shared : dict
        Shared state dictionary; updates keys ``'target'``, ``'zero_compliance'``,
        and ``'open_plot'``.
    """
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



def main():
    """Initialize hardware, start the PID control loop, and handle shutdown."""
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
        """Return the first dotted attribute path in *candidates* that resolves on *root*.

        Parameters
        ----------
        root : object
            Object to search.
        candidates : iterable of str
            Dotted attribute paths to try.

        Returns
        -------
        str or None
            First resolving path, or ``None``.
        """
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
        """Read a dotted attribute path from *root* and return it as float.

        Parameters
        ----------
        root : object
            Object to traverse.
        attr : str or None
            Dotted attribute path.

        Returns
        -------
        float or None
            Attribute value cast to float, or ``None`` on any error.
        """
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

    def read_iq(odrv) -> float | None:
        """Return the measured q-axis current [A], or ``None``."""
        return _read_attr(odrv.axis0, _iq_attr)

    def read_temp(odrv) -> float | None:
        """Return the motor thermistor temperature [°C], or ``None``."""
        return _read_attr(odrv.axis0, _temp_attr)

    import math

    def read_pos(odrv) -> float | None:
        """Return motor position in turns from ``axis0.pos_estimate``, or ``None`` if NaN.

        Parameters
        ----------
        odrv : odrive.ODrive
            Connected ODrive object.

        Returns
        -------
        float or None
        """
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
        """Return the torsional compliance angle δ [deg] at the output shaft.

        Positive δ means the output lags behind the input prediction;
        negative means the output leads.

        Parameters
        ----------
        output_deg : float
            Current output shaft angle from the encoder [deg].

        Returns
        -------
        float or None
            Compliance angle [deg], or ``None`` if motor position is unavailable.
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

            iq    = read_iq(odrv)
            temp  = read_temp(odrv)
            delta = compliance_delta(current_deg)

            if shared.pop("zero_compliance", False) and delta is not None:
                _compliance_offset += delta
                delta = 0.0

            torque_iq   = MOTOR_KT * iq * GEAR_RATIO if iq is not None else None
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
