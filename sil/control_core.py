"""
control_core.py - ctypes wrapper for libcontrol_core.so (the same controller code as on the STM32).

Build the library first: make -C sil lib

Units as in the firmware: pitch [deg], pitch_rate [deg/s], position [m], velocity [m/s],
output [A]. The simulator turns the current into torque in its motor model (Kt, step 15.5).

Usage:
    c = CascadeController()               # gains from CascadeParams (= config.h)
    c.reset(pitch, pitch_rate, position, velocity)    # = entering BALANCING
    out_l, out_r = c.step(pitch, pitch_rate, position, velocity, dt=0.005)
"""
import ctypes as C
import os
import sys
from dataclasses import dataclass

_API_VERSION = 1  # must match CTRL_API_VERSION in control_api.h


# --- mirrors of the C structs in control_api.h (same field order, floats only) ---

class _PidParams(C.Structure):
    _fields_ = [(n, C.c_float) for n in ("kp", "ki", "kd", "max_output", "max_i")]


class _CascadeParams(C.Structure):
    _fields_ = [("position", _PidParams), ("velocity", _PidParams), ("pitch", _PidParams)]


class _Input(C.Structure):
    _fields_ = [(n, C.c_float) for n in ("pitch", "pitch_rate", "position", "velocity")]


class _Output(C.Structure):
    _fields_ = [(n, C.c_float) for n in (
        "out_l", "out_r", "position_output", "velocity_output", "pitch_setpoint",
        "pitch_output", "i_position", "i_velocity", "i_pitch")]


# --- public parameters ---

@dataclass
class PidParams:
    """Gains of one PID loop, in firmware units."""
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    max_output: float = 1.0
    max_i: float | None = None          # None = 0.3 * max_output, as in controllers_config.c

    def _c(self) -> _PidParams:
        max_i = 0.3 * self.max_output if self.max_i is None else self.max_i
        return _PidParams(self.kp, self.ki, self.kd, self.max_output, max_i)


@dataclass
class CascadeParams:
    """Gains of the whole cascade. Defaults = config.h; keep them in sync by hand until step 16.1."""
    position: PidParams = None
    velocity: PidParams = None
    pitch: PidParams = None

    def __post_init__(self):
        self.position = self.position or PidParams(0.40, 0.0, 0.0, 1.0)    # CASCADE_POS_*
        self.velocity = self.velocity or PidParams(6.0, 0.50, 0.0, 8.0)    # CASCADE_VEL_*
        self.pitch = self.pitch or PidParams(0.80, 0.0, 0.04, 2.5)         # CASCADE_PITCH_*


def _load(path: str | None) -> C.CDLL:
    """Load the library (default: next to this file) and declare the C signatures for ctypes."""
    if path is None:
        ext = {"win32": "control_core.dll", "darwin": "libcontrol_core.dylib"}.get(sys.platform, "libcontrol_core.so")
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ext)
    lib = C.CDLL(path)
    lib.ctrl_api_version.restype = C.c_int
    lib.ctrl_create.restype = C.c_void_p
    lib.ctrl_create.argtypes = [C.POINTER(_CascadeParams)]
    lib.ctrl_destroy.argtypes = [C.c_void_p]
    lib.ctrl_reset.argtypes = [C.c_void_p, C.POINTER(_Input)]
    lib.ctrl_set_target.argtypes = [C.c_void_p, C.c_float, C.c_float]
    lib.ctrl_step.argtypes = [C.c_void_p, C.POINTER(_Input), C.c_float, C.POINTER(_Output)]
    if lib.ctrl_api_version() != _API_VERSION:
        raise RuntimeError(f"library API version {lib.ctrl_api_version()} != {_API_VERSION}: rebuild with make -C sil lib")
    return lib


class CascadeController:
    """One instance of the firmware cascade (position -> velocity -> pitch -> current)."""

    def __init__(self, params: CascadeParams | None = None, lib_path: str | None = None):
        p = params or CascadeParams()
        self._lib = _load(lib_path)
        cp = _CascadeParams(p.position._c(), p.velocity._c(), p.pitch._c())
        self._h = self._lib.ctrl_create(C.byref(cp))
        if not self._h:
            raise MemoryError("ctrl_create failed")
        self._in, self._out = _Input(), _Output()   # reused buffers, no allocation per step

    def _fill(self, pitch, pitch_rate, position, velocity):
        self._in.pitch, self._in.pitch_rate = pitch, pitch_rate
        self._in.position, self._in.velocity = position, velocity

    def reset(self, pitch=0.0, pitch_rate=0.0, position=0.0, velocity=0.0):
        """Same as entering BALANCING: PIDs reset, target_position = position."""
        self._fill(pitch, pitch_rate, position, velocity)
        self._lib.ctrl_reset(self._h, C.byref(self._in))

    def set_target(self, target_pitch=0.0, target_position=0.0):
        """target_pitch [deg] (added to the velocity loop output), target_position [m]."""
        self._lib.ctrl_set_target(self._h, target_pitch, target_position)

    def step(self, pitch, pitch_rate, position, velocity, dt):
        """One control period. Returns (out_l, out_r) [A]; the loop internals are in .debug."""
        self._fill(pitch, pitch_rate, position, velocity)
        self._lib.ctrl_step(self._h, C.byref(self._in), dt, C.byref(self._out))
        return self._out.out_l, self._out.out_r

    @property
    def debug(self) -> dict:
        """Outputs and integrals of all loops from the last step (same names as in the telemetry)."""
        return {n: getattr(self._out, n) for n, _ in _Output._fields_}

    def close(self):
        if self._h:
            self._lib.ctrl_destroy(self._h)
            self._h = None

    def __enter__(self): return self
    def __exit__(self, *a): self.close()
    def __del__(self): self.close()


if __name__ == "__main__":
    # smoke test: robot tilted by 1 deg -> current against the tilt (-0.8 A with Kp = 0.8)
    with CascadeController() as c:
        c.reset(pitch=1.0)
        print(c.step(1.0, 0.0, 0.0, 0.0, 0.005), c.debug)
