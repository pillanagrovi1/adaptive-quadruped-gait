"""ctypes binding to build/libtinyrl.so: the SAME C learner code that is
compiled for the Cortex-M4 runs here, driven from Python.

Python only converts floats <-> Q16.16 at the boundary. All learning
arithmetic happens in C.
"""
from __future__ import annotations

import ctypes as C
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
LIB_PATH = ROOT / "build" / "libtinyrl.so"
MAX_OBS = 12

i32 = C.c_int32
P_i32 = C.POINTER(i32)


class Cfg(C.Structure):
    # Must match tr_cfg_t in c/include/tinyrl.h field for field.
    _fields_ = [
        ("obs_dim", i32), ("n_actions", i32),
        ("obs_lo", i32 * MAX_OBS), ("obs_hi", i32 * MAX_OBS),
        ("alpha", i32), ("gamma", i32),
        ("bins", i32), ("td_clip", i32),
        ("lam", i32), ("obgd_kappa", i32), ("round_mode", i32), ("trace_cap", i32),
        ("watkins", i32),
    ]


_lib = None


def lib():
    global _lib
    if _lib is None:
        if not LIB_PATH.exists():
            raise FileNotFoundError(f"{LIB_PATH} missing: run `make` in the repo root")
        L = C.CDLL(str(LIB_PATH))
        L.tr_mem_bytes.argtypes = [C.POINTER(Cfg)]
        L.tr_mem_bytes.restype = C.c_size_t
        L.tr_agent_sizeof.restype = C.c_size_t
        L.tr_param_count.argtypes = [C.POINTER(Cfg)]
        L.tr_param_count.restype = C.c_uint32
        L.tr_init.argtypes = [C.c_void_p, C.POINTER(Cfg), C.c_void_p, C.c_size_t, C.c_uint32]
        L.tr_init.restype = C.c_int
        L.tr_act.argtypes = [C.c_void_p, P_i32, i32]
        L.tr_act.restype = C.c_int
        L.tr_greedy.argtypes = [C.c_void_p, P_i32]
        L.tr_greedy.restype = C.c_int
        L.tr_qvalues.argtypes = [C.c_void_p, P_i32, P_i32]
        L.tr_observe.argtypes = [C.c_void_p, P_i32, C.c_int, i32, P_i32, C.c_int]
        L.tr_observe.restype = C.c_int
        L.tr_episode_start.argtypes = [C.c_void_p]
        L.tr_underflow_count.argtypes = [C.c_void_p]
        L.tr_underflow_count.restype = C.c_uint32
        L.tr_update_terms.argtypes = [C.c_void_p]
        L.tr_update_terms.restype = C.c_uint32
        L.tr_saturation_count.argtypes = [C.c_void_p]
        L.tr_saturation_count.restype = C.c_uint32
        _lib = L
    return _lib


def q16(x):
    """float or array -> int32 Q16.16 (round half away from zero, saturate)."""
    a = np.asarray(x, dtype=np.float64) * 65536.0
    a = np.where(a >= 0, np.floor(a + 0.5), -np.floor(-a + 0.5))
    return np.clip(a, -2**31, 2**31 - 1).astype(np.int32)


LEARNER_DEFAULTS = {
    "alpha": 0.1, "gamma": 0.98,
    "bins": 10, "td_clip": 0.0,
    "lam": 0.0,          # Q(lambda) trace decay, 0 = off
    "obgd_kappa": 0.0,   # ObGD step-size cap (Elsayed et al. 2024), typical 2.0, 0 = off
    "round_mode": 0,     # 1 = stochastic rounding of updates
    "trace_cap": 0,      # sparse-trace entries, 0 = automatic
    "watkins": 0,        # traces: 1 = cut after a non-greedy action (Watkins Q(lambda)), 0 = never
}


def make_cfg(learner: dict, obs_lo, obs_hi, n_actions: int) -> Cfg:
    p = {**LEARNER_DEFAULTS, **learner}
    d = len(obs_lo)
    if d > MAX_OBS:
        raise ValueError(f"obs_dim {d} > {MAX_OBS}")
    c = Cfg()
    c.obs_dim = d
    c.n_actions = n_actions
    lo, hi = q16(obs_lo), q16(obs_hi)
    for i in range(d):
        c.obs_lo[i], c.obs_hi[i] = int(lo[i]), int(hi[i])
    c.alpha, c.gamma = int(q16(p["alpha"])), int(q16(p["gamma"]))
    c.bins, c.td_clip = int(p["bins"]), int(q16(p["td_clip"]))
    c.lam, c.obgd_kappa = int(q16(p["lam"])), int(q16(p["obgd_kappa"]))
    c.round_mode, c.trace_cap = int(p["round_mode"]), int(p["trace_cap"])
    c.watkins = int(p["watkins"])
    unknown = set(learner) - set(LEARNER_DEFAULTS)
    if unknown:
        raise ValueError(f"unknown learner keys {sorted(unknown)}")
    return c


def cfg_dict(c: Cfg) -> dict:
    out = {}
    for name, _ in Cfg._fields_:
        v = getattr(c, name)
        out[name] = list(v) if not isinstance(v, int) else v
    return out


def cfg_id(c: Cfg) -> str:
    """Stable short hash of the C config; links host runs to M4 measurements."""
    s = json.dumps(cfg_dict(c), sort_keys=True)
    return hashlib.sha1(s.encode()).hexdigest()[:10]


class Agent:
    def __init__(self, learner: dict, obs_lo, obs_hi, n_actions: int, seed: int):
        L = lib()
        self.cfg = make_cfg(learner, obs_lo, obs_hi, n_actions)
        self.mem_bytes = int(L.tr_mem_bytes(C.byref(self.cfg)))
        if self.mem_bytes == 0:
            raise ValueError(f"invalid learner config: {learner}")
        self.params = int(L.tr_param_count(C.byref(self.cfg)))
        self.host_agent_bytes = int(L.tr_agent_sizeof())
        self._agent = C.create_string_buffer(self.host_agent_bytes)
        self._arena = C.create_string_buffer(self.mem_bytes)
        rc = L.tr_init(self._agent, C.byref(self.cfg), self._arena, self.mem_bytes, seed & 0xFFFFFFFF)
        if rc != 0:
            raise RuntimeError(f"tr_init failed ({rc}) for {learner}")
        d = len(obs_lo)
        self._s = np.zeros(d, np.int32)
        self._s2 = np.zeros(d, np.int32)
        self._q = np.zeros(n_actions, np.int32)
        self._ps = self._s.ctypes.data_as(P_i32)
        self._ps2 = self._s2.ctypes.data_as(P_i32)
        self._pq = self._q.ctypes.data_as(P_i32)
        self.id = cfg_id(self.cfg)

    def act(self, obs, eps: float) -> int:
        self._s[:] = q16(obs)
        return lib().tr_act(self._agent, self._ps, int(q16(eps)))

    def greedy(self, obs) -> int:
        self._s[:] = q16(obs)
        return lib().tr_greedy(self._agent, self._ps)

    def qvalues(self, obs) -> np.ndarray:
        self._s[:] = q16(obs)
        lib().tr_qvalues(self._agent, self._ps, self._pq)
        return self._q / 65536.0

    def episode_start(self) -> None:
        lib().tr_episode_start(self._agent)

    def underflow(self) -> tuple[int, int]:
        """(updates that rounded to zero, update products computed) since init."""
        L = lib()
        return int(L.tr_underflow_count(self._agent)), int(L.tr_update_terms(self._agent))

    def saturation(self) -> int:
        """Update terms clipped to the int32 range since init."""
        return int(lib().tr_saturation_count(self._agent))

    def observe(self, s, a: int, r: float, s2, done: bool) -> int:
        self._s[:] = q16(s)
        self._s2[:] = q16(s2)
        return lib().tr_observe(self._agent, self._ps, int(a), int(q16(r)), self._ps2, int(bool(done)))
