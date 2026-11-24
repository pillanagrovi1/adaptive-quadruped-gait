"""Bit-exact Python mirror of c/include/tr_fixed.h and tr_rng.h.

Python ints never overflow, so we emulate int32 saturation explicitly.
Python's >> on negative ints is a floor shift, the same as GCC's arithmetic
shift on signed ints, so the rounding rule `(p + 2**15) >> 16` matches C.
"""

ONE = 1 << 16
INT32_MAX = (1 << 31) - 1
INT32_MIN = -(1 << 31)


def sat(v: int) -> int:
    return INT32_MAX if v > INT32_MAX else INT32_MIN if v < INT32_MIN else v


def q16(x: float) -> int:
    """Same rounding as the C Q16() macro (round half away from zero)."""
    return int(x * 65536.0 + 0.5) if x >= 0 else -int(-x * 65536.0 + 0.5)


def to_float(q: int) -> float:
    return q / 65536.0


def add(a: int, b: int) -> int:
    return sat(a + b)


def sub(a: int, b: int) -> int:
    return sat(a - b)


def mul(a: int, b: int) -> int:
    return sat((a * b + (1 << 15)) >> 16)


def clip(x: int, lo: int, hi: int) -> int:
    return lo if x < lo else hi if x > hi else x


class XorShift32:
    """Mirror of tr_rng.h."""

    def __init__(self, seed: int):
        self.s = (seed & 0xFFFFFFFF) or 0x9E3779B9

    def next(self) -> int:
        x = self.s
        x ^= (x << 13) & 0xFFFFFFFF
        x ^= x >> 17
        x ^= (x << 5) & 0xFFFFFFFF
        self.s = x
        return x

    def below(self, n: int) -> int:
        return self.next() % n

    def unit_q16(self) -> int:
        return self.next() >> 16
