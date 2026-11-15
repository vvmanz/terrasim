"""Python access to the C++ terramechanics core (single source of truth) via ctypes.

    from terramech import Wheel, Soil, Opts, forces, sinkage
    f = forces(Wheel(r=.135, b=.165, h=.01), Soil.table1(), Opts(), s=0.2, z=0.01)

The shared library is built by scripts/build.sh core (core/build/libterramech_c.so).
If it is missing, it is compiled on the fly with g++.
"""
import ctypes, os, subprocess, math
from dataclasses import dataclass, field, asdict

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
_LIB_CANDIDATES = [os.path.join(_ROOT, 'core', 'build', 'libterramech_c.so'), os.path.join(_HERE, '.cache', 'libterramech_c.so')]


def _load():
    for p in _LIB_CANDIDATES:
        if os.path.exists(p):
            return ctypes.CDLL(p)
    out = _LIB_CANDIDATES[1]
    os.makedirs(os.path.dirname(out), exist_ok=True)
    src = os.path.join(_ROOT, 'core', 'src', 'c_api.cpp')
    inc = os.path.join(_ROOT, 'core', 'include')
    subprocess.check_call(['g++', '-O2', '-std=c++17', '-shared', '-fPIC', '-I', inc, src, '-o', out])
    return ctypes.CDLL(out)


_lib = _load()
_D = ctypes.POINTER(ctypes.c_double)
_lib.tm_forces.argtypes = [_D, _D, _D, ctypes.c_double, ctypes.c_double, ctypes.c_double, _D]
_lib.tm_sinkage.argtypes = [_D, _D, _D, ctypes.c_double, ctypes.c_double]
_lib.tm_sinkage.restype = ctypes.c_double
_lib.tm_slip.argtypes = [ctypes.c_double] * 3
_lib.tm_slip.restype = ctypes.c_double


def _arr(v):
    a = (ctypes.c_double * len(v))(*v)
    return a


@dataclass
class Wheel:
    r: float = 0.14
    b: float = 0.20
    h: float = 0.01
    lam: float = 0.0
    Rj: float = -1.0   # <=0 -> r+h

    def vec(self):
        return [self.r, self.b, self.h, self.lam, self.Rj]


@dataclass
class Soil:
    kc: float = 13.6e3
    kphi: float = 2259.1e3
    n0: float = 0.92
    n1: float = 0.5
    c: float = 462.3
    phi_deg: float = 35.0
    K: float = 0.015
    Ky: float = 0.015
    a0: float = 0.4
    a1: float = 0.15
    soil: float = 1.0

    @staticmethod
    def table1():
        return Soil(15.6e3, 2407.6e3, 0.895, 0.995, 250.0, 31.9, 0.01, 0.01)

    @staticmethod
    def table2():
        return Soil(13.6e3, 2259.1e3, 0.92, 0.5, 462.3, 35.0, 0.015, 0.015)

    def vec(self):
        return [self.kc, self.kphi, self.n0, self.n1, self.c, math.radians(self.phi_deg), self.K, self.Ky, self.a0, self.a1, self.soil]


@dataclass
class Opts:
    model: int = 0      # 0 closed form (MarsSim), 1 Wong-Reece integral
    cp_form: int = 0    # 0: cp=1; 1: 1+cp1+cp2*s (as printed); 2: 1+cp1*s+cp2*s^2
    cp1: float = 0.0
    cp2: float = 0.0
    cp3: float = 0.0
    cM: float = 0.0
    W_bar: float = 0.0
    n_integ: int = 64
    sN_lo: float = 0.0   # slip clamp for the sinkage exponent N = n0 + n1*clamp(s, sN_lo, sN_hi)
    sN_hi: float = 0.9   # (paper literal: sN_lo=-1, sN_hi=1)

    def vec(self):
        return [self.model, self.cp_form, self.cp1, self.cp2, self.cp3, self.cM, self.W_bar, self.n_integ, self.sN_lo, self.sN_hi]


_KEYS = ['contact', 'FN', 'FDP', 'FS', 'MR', 'MO', 'MS', 'theta1', 'thetam', 'sigma_m', 'tau_m', 'jm', 'N']


def forces(w: Wheel, s_: Soil, o: Opts, s: float, z: float, beta: float = 0.0) -> dict:
    out = (ctypes.c_double * 13)()
    _lib.tm_forces(_arr(w.vec()), _arr(s_.vec()), _arr(o.vec()), float(s), float(z), float(beta), out)
    return dict(zip(_KEYS, list(out)))


def sinkage(w: Wheel, s_: Soil, o: Opts, s: float, W: float) -> float:
    return _lib.tm_sinkage(_arr(w.vec()), _arr(s_.vec()), _arr(o.vec()), float(s), float(W))


def slip(vx, omega, rs):
    return _lib.tm_slip(vx, omega, rs)
