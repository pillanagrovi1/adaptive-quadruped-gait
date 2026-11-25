"""Minimal binary/ASCII STL reader: triangles, solid volume, bounding box. No dependencies but numpy."""
from __future__ import annotations

import struct
from pathlib import Path

import numpy as np


def load(path: Path) -> np.ndarray:
    """Return an (n, 3, 3) array of triangle vertices in the file's units (mm here)."""
    data = Path(path).read_bytes()
    n = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    if len(data) == 84 + 50 * n:
        rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                            count=n, offset=84)
        return rec["v"].astype(np.float64)
    verts = [list(map(float, line.split()[1:4])) for line in data.decode(errors="replace").splitlines()
             if line.strip().startswith("vertex")]
    return np.asarray(verts, dtype=np.float64).reshape(-1, 3, 3)


def volume(tri: np.ndarray) -> float:
    """Enclosed volume by the divergence theorem (sum of signed tetrahedra). Needs a closed mesh."""
    return float(abs(np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum()) / 6.0)


def bbox(tri: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    v = tri.reshape(-1, 3)
    return v.min(0), v.max(0)
