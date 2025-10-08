import math
import pytest

from Interval import Interval


def test_constructors_and_bounds():
    a = Interval()
    assert math.isinf(a.lb) and a.lb < 0
    assert math.isinf(a.ub) and a.ub > 0

    b = Interval(4)
    assert b.lb == 4 and b.ub == 4

    c = Interval(-2, 7)
    assert c.lb == -2 and c.ub == 7

    # swapped args should result in proper ordering
    d = Interval(7, -2)
    assert d.lb == -2 and d.ub == 7


def test_str_and_equality():
    a = Interval(1, 2)
    s = str(a)
    assert '[' in s and ']' in s

    assert Interval(1, 2) == Interval(1, 2)
    assert not (Interval(1, 2) == Interval(1.0, 3.0))


def test_contains_and_overlaps():
    outer = Interval(-2, 7)
    inner = Interval(0, 1)
    assert inner in outer

    a = Interval(0, 2)
    b = Interval(1, 3)
    assert a.__overlaps__(b) or b.__overlaps__(a)

    non_overlap = Interval(3.1, 4.0)
    assert not a.__overlaps__(non_overlap)


def test_invalid_eq_arg():
    with pytest.raises(ValueError):
        _ = (Interval(0, 1) == 5)  # equality with non-Interval should raise


def test_add_intervals_and_scalar():
    a = Interval(1, 2)
    b = Interval(3, 4)
    c = a + b
    assert isinstance(c, Interval)
    assert c.lb == 4 and c.ub == 6

    d = a + 3
    assert d.lb == 4 and d.ub == 5


def test_sub_intervals_and_scalar():
    a = Interval(1, 2)
    b = Interval(3, 4)
    c = a - b
    assert isinstance(c, Interval)
    assert c.lb == -3 and c.ub == -1

    d = a - 1
    assert d.lb == 0 and d.ub == 1


def test_mul_intervals_and_scalar():
    a = Interval(1, 3)
    b = Interval(2, 4)
    c = a * b
    # products: [1*2,1*4,3*2,3*4] -> [2,4,6,12] -> interval [2,12]
    assert c.lb == 2 and c.ub == 12

    # multiplication with negative scalar
    d = Interval(1, 2) * -2
    assert d.lb == -4 and d.ub == -2

    # mixed-sign interval
    e = Interval(-1, 2) * Interval(3, 4)
    # products: [-3, -4, 6, 8] -> interval [-4, 8]
    assert e.lb == -4 and e.ub == 8


def test_truediv_interval_and_scalar():
    a = Interval(4, 8)
    b = Interval(2, 4)
    c = a / b
    # quotients: [4/2, 4/4, 8/2, 8/4] -> [2.0,1.0,4.0,2.0] -> [1.0,4.0]
    assert math.isclose(c.lb, 1.0)
    assert math.isclose(c.ub, 4.0)

    d = a / 2
    assert math.isclose(d.lb, 2.0) and math.isclose(d.ub, 4.0)


def test_division_by_zero_interval_and_scalar_raises():
    a = Interval(1, 2)
    zero_interval = Interval(-1, 1)
    with pytest.raises(ZeroDivisionError):
        _ = a / zero_interval

    with pytest.raises(ZeroDivisionError):
        _ = a / 0