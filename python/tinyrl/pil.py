"""Processor-in-the-loop client: the learner runs on the microcontroller.

`PilAgent` has the same interface as `cbind.Agent` (act, greedy, observe,
episode_start), so the same driver code can run an on-chip learner unchanged:

    learner = {"bins": 11, "pil": "tcp:127.0.0.1:4444"}   # QEMU
    learner = {"bins": 11, "pil": "serial:/dev/ttyACM0"}   # Nucleo (pip install pyserial)

The board firmware is m4/pil/pil_main.c (frame format documented there).
Every call returns the chip's cycle count for that call (0 in QEMU, which has no
cycle counter); `cycle_log` keeps them for the timing analysis on real hardware.
"""
from __future__ import annotations

import socket
import struct
import time

import numpy as np

from .cbind import cfg_id, make_cfg, q16


class TcpLink:
    def __init__(self, host: str, port: int, timeout: float = 30.0, retries: int = 50):
        last = None
        for _ in range(retries):
            try:
                self.s = socket.create_connection((host, port), timeout=timeout)
                self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                return
            except OSError as e:          # QEMU may still be starting
                last = e
                time.sleep(0.1)
        raise ConnectionError(f"cannot connect to {host}:{port}: {last}")

    def write(self, b: bytes):
        self.s.sendall(b)

    def read(self, n: int) -> bytes:
        out = b""
        while len(out) < n:
            chunk = self.s.recv(n - len(out))
            if not chunk:
                raise ConnectionError("link closed")
            out += chunk
        return out

    def close(self):
        self.s.close()


class SerialLink:
    def __init__(self, dev: str, baud: int = 115200):
        import serial  # pyserial, only needed for real hardware
        self.p = serial.Serial(dev, baud, timeout=10)
        time.sleep(0.2)
        self.p.reset_input_buffer()

    def write(self, b: bytes):
        self.p.write(b)

    def read(self, n: int) -> bytes:
        b = self.p.read(n)
        if len(b) != n:
            raise TimeoutError("serial read timed out")
        return b

    def close(self):
        self.p.close()


def open_link(spec: str):
    kind, rest = spec.split(":", 1)
    if kind == "tcp":
        host, port = rest.rsplit(":", 1)
        return TcpLink(host, int(port))
    if kind == "serial":
        dev, _, baud = rest.partition("@")
        return SerialLink(dev, int(baud) if baud else 115200)
    raise ValueError(f"unknown PIL link {spec!r} (use tcp:HOST:PORT or serial:DEV[@BAUD])")


class PilAgent:
    def __init__(self, learner: dict, obs_lo, obs_hi, n_actions: int, seed: int):
        spec = learner["pil"]
        host_learner = {k: v for k, v in learner.items() if k != "pil"}
        self.cfg = make_cfg(host_learner, obs_lo, obs_hi, n_actions)
        self.id = cfg_id(self.cfg)
        self.d = len(obs_lo)
        self.link = open_link(spec)
        self.cycle_log = {"act": [], "observe": []}
        ver, cfg_size, max_obs, cap, hz = struct.unpack("<5I", self._call(0x00, b""))
        if cfg_size != len(bytes(self.cfg)):
            raise RuntimeError(f"tr_cfg_t size mismatch: board {cfg_size} B, host {len(bytes(self.cfg))} B "
                               "(rebuild the firmware from the same commit)")
        self.cpu_hz = hz
        rc, used, agent_bytes = struct.unpack("<iII", self._call(0x01, bytes(self.cfg) + struct.pack("<I", (seed + 0) & 0xFFFFFFFF)))
        if rc != 0:
            raise RuntimeError(f"tr_init on the board failed ({rc}); arena capacity {cap} B")
        self.mem_bytes, self.board_agent_bytes, self.host_agent_bytes = used, agent_bytes, agent_bytes
        self.params = 0

    # ---- framing ----
    def _call(self, typ: int, payload: bytes) -> bytes:
        self.link.write(struct.pack("<BBH", 0xA5, typ, len(payload)) + payload + bytes([sum(payload) & 0xFF]))
        while self.link.read(1) != b"\xa5":
            pass
        rtyp, n = struct.unpack("<BH", self.link.read(3))
        body = self.link.read(n)
        if self.link.read(1)[0] != (sum(body) & 0xFF):
            raise IOError("checksum error from board")
        if rtyp == 0xFF:
            raise RuntimeError(f"board error code {body[0]}")
        if rtyp != (typ | 0x80):
            raise IOError(f"unexpected reply type {rtyp:#x} to {typ:#x}")
        return body

    def _obs(self, o) -> bytes:
        return q16(np.asarray(o, dtype=np.float64)).astype("<i4").tobytes()

    # ---- agent interface ----
    def act(self, obs, eps: float) -> int:
        a, cyc = struct.unpack("<BI", self._call(0x02, self._obs(obs) + struct.pack("<i", int(q16(eps)))))
        self.cycle_log["act"].append(cyc)
        return a

    def greedy(self, obs) -> int:
        a, _ = struct.unpack("<BI", self._call(0x03, self._obs(obs)))
        return a

    def observe(self, s, a: int, r: float, s2, done: bool) -> int:
        body = self._obs(s) + struct.pack("<Bi", int(a), int(q16(r))) + self._obs(s2) + struct.pack("<B", int(bool(done)))
        k, cyc = struct.unpack("<iI", self._call(0x04, body))
        self.cycle_log["observe"].append(cyc)
        return k

    def episode_start(self) -> None:
        self._call(0x05, b"")

    def stats(self) -> dict:
        peak, max_act, max_obs = struct.unpack("<3I", self._call(0x06, b""))
        return {"stack_peak_bytes": peak, "max_act_cycles": max_act, "max_observe_cycles": max_obs}

    def close(self):
        self.link.close()
