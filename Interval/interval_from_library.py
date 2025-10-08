"""Small examples / utilities showing how to use the `interval` (pyinterval)
library for interval arithmetic and a demo interval bisection root isolator.

This file is intentionally practical: import it and run it to see example
output. It requires an interval library such as `pyinterval` (common import
name: `interval`) which provides `interval`, `inf`, and `imath`.

If that package is not installed the module will print an installation hint.
"""
from __future__ import annotations

try:
    from interval import interval, inf, imath
except Exception:  # ImportError or other
    interval = None
    inf = float('inf')
    imath = None

from typing import Optional


def check_interval_available() -> bool:
    if interval is None or imath is None:
        print("Required package `interval` (pyinterval) not found.")
        print("Install with: pip install pyinterval")
        return False
    return True


def demo_creation():
    """Show basic interval creation and printing."""
    if not check_interval_available():
        return

    a = interval[1.0, 2.0]
    b = interval[3.0, 4.0]
    whole = interval[-inf, inf]
    print("Creation examples:")
    print(" a =", a)
    print(" b =", b)
    print(" whole =", whole)


def demo_arithmetic():
    if not check_interval_available():
        return
    a = interval[1.0, 2.0]
    b = interval[3.0, 4.0]
    print("Arithmetic examples:")
    print(" a + b =", a + b)
    print(" a * b =", a * b)
    print(" a - 1.5 =", a - 1.5)
    print(" 2 * a =", 2 * a)
    try:
        print("a / interval[-1,1] ->", a / interval[-1.0, 1.0])
    except ZeroDivisionError:
        print("Division by interval containing 0 raised ZeroDivisionError")
    # use imath for functions
    print("sin(a) =", imath.sin(a))


def demo_set_ops():
    if not check_interval_available():
        return
    I1 = interval[0.0, 2.0]
    I2 = interval[1.0, 3.0]
    print("Set-like operations:")
    print(" I1 & I2 =", I1 & I2)
    print(" I1 | I2 =", I1 | I2)
    print(" 1.5 in I1?", 1.5 in I1)


def interval_contains_zero(iv) -> bool:
    """Return True if interval iv's range includes 0 (works for union objects)."""
    # interval objects are iterable over subintervals; each sub is a tuple-like
    # where sub[0][0] is left endpoint and sub[0][1] is right endpoint.
    for sub in iv:
        try:
            lb = float(sub[0][0])
            ub = float(sub[0][1])
        except Exception:
            # fallback: try reshape
            vals = list(map(float, sub))
            if len(vals) >= 2:
                lb, ub = vals[0], vals[1]
            else:
                continue
        if lb <= 0.0 <= ub:
            return True
    return False


def interval_bisect_root(f_imath, X, tol=1e-6, max_iters=60) -> Optional[object]:
    """Narrow an interval X until its width <= tol using interval evaluation.

    - f_imath: function accepting interval objects (built with imath) and
      returning interval results.
    - X: starting interval (e.g. interval[1,2]).

    Returns the narrowed interval (or None if no root is present in X).
    """
    if not check_interval_available():
        return None

    for _ in range(max_iters):
        fX = f_imath(X)
        if not interval_contains_zero(fX):
            return None
        # extract endpoints of X (assume single contiguous interval)
        sub = list(X)[0]
        lb = float(sub[0][0]); ub = float(sub[0][1])
        if (ub - lb) <= tol:
            return X
        mid = 0.5 * (lb + ub)
        left = interval[lb, mid]
        right = interval[mid, ub]
        fleft = f_imath(left)
        fright = f_imath(right)
        left_has = interval_contains_zero(fleft)
        right_has = interval_contains_zero(fright)
        if left_has:
            X = left
        elif right_has:
            X = right
        else:
            # both sides fail — return current enclosure
            return X
    return X


def demo_root_bisect():
    if not check_interval_available():
        return
    print("\nRoot bisection demo: find sqrt(2)")
    f_imath = lambda X: imath.power(X, 2) - 2
    initial = interval[1.0, 2.0]
    root_iv = interval_bisect_root(f_imath, initial, tol=1e-8)
    print("approx root interval:", root_iv)


def main():
    print("Running interval examples (requires pyinterval/interval package)")
    demo_creation()
    demo_arithmetic()
    demo_set_ops()
    demo_root_bisect()


if __name__ == '__main__':
    main()
