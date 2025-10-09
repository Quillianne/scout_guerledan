import math



class Interval:
    def __init__(self, *args: float):
        """Construct an Interval in three ways:

        - Interval() -> entire real line (-inf, +inf)
        - Interval(value) -> point interval [value, value]
        - Interval(start, end) -> interval [min(start,end), max(start,end)]

        Raises TypeError if more than 2 arguments are provided.
        """
        if len(args) == 0:
            self.lb = -math.inf
            self.ub = math.inf
        elif len(args) == 1:
            v = float(args[0])
            self.lb = v
            self.ub = v
        elif len(args) == 2:
            a = float(args[0])
            b = float(args[1])
            if a > b:
                self.lb = b
                self.ub = a
            else:
                self.lb = a
                self.ub = b
        else:
            raise TypeError(f"Interval() takes 0, 1 or 2 numeric arguments ({len(args)} given)")

    def __str__(self):
        if self.is_empty():
            return "empty"
        return f"[{self.lb}, {self.ub}]"
    
    def __is_empty__(self):
        return self.is_empty()

    def is_empty(self):
        """Return True when this interval is the empty set.

        We treat intervals with NaN endpoints or lb>ub as empty.
        """
        return math.isnan(self.lb) or math.isnan(self.ub) or (self.lb > self.ub)

    @classmethod
    def empty_set(cls):
        """Return a canonical empty interval object."""
        inst = cls.__new__(cls)
        inst.lb = float('nan')
        inst.ub = float('nan')
        return inst

    def __contains__(self, other):
        # Support membership testing for Interval instances or numeric values
        if isinstance(other, Interval):
            return (self.lb <= other.lb) and (self.ub >= other.ub)
        try:
            v = float(other)
            return (self.lb <= v) and (v <= self.ub)
        except Exception:
            raise ValueError("Argument must be an Interval or numeric value")
    
    def __eq__(self, other):
        if not isinstance(other, Interval):
            raise ValueError("Argument must be an Interval")
        return (self.lb == other.lb) and (self.ub == other.ub)

    def __overlaps__(self, other):
        if not isinstance(other, Interval):
            raise ValueError("Argument must be an Interval")
        # strict overlap: touching at boundaries does not count
        return (self.lb < other.ub) and (other.lb < self.ub)



    def __add__(self, other):
        if isinstance(other, Interval):
            return Interval(self.lb + other.lb, self.ub + other.ub)
        else:
            v = float(other)
            return Interval(self.lb + v, self.ub + v)

    def __sub__(self, other):
        if isinstance(other, Interval):
            return Interval(self.lb - other.ub, self.ub - other.lb)
        else:
            v = float(other)
            return Interval(self.lb - v, self.ub - v)

    def __neg__(self):
        """Unary negation of an interval: -[lb,ub] == [-ub, -lb].

        Returns the canonical empty set for empty inputs.
        """
        if self.is_empty():
            return Interval.empty_set()
        return Interval(-self.ub, -self.lb)

    def __mul__(self, other):
        if isinstance(other, Interval):
            products = [
                self.lb * other.lb,
                self.lb * other.ub,
                self.ub * other.lb,
                self.ub * other.ub
            ]
            return Interval(min(products), max(products))
        else:
            v = float(other)
            products = [self.lb * v, self.ub * v]
            return Interval(min(products), max(products))

    def __truediv__(self, other):
        if isinstance(other, Interval):
            # Use the interval inverse helper
            inv = other.inverse()
            if inv.is_empty():
                return Interval.empty_set()
            return self * inv
        else:
            v = float(other)
            if v == 0.0:
                raise ZeroDivisionError("Interval division by zero")
            quotients = [self.lb / v, self.ub / v]
            return Interval(min(quotients), max(quotients))
    
    def inverse(self):
        """Return the multiplicative inverse interval 1 / self.

        Follows the C++ semantics:
        - if self == [0,0] -> empty set
        - if self does not contain 0 -> Interval(1/self.ub, 1/self.lb)
        - if self.lb == 0 and self.ub > 0 -> Interval(1/self.ub, +inf)
        - if self.lb < 0 and self.ub == 0 -> Interval(-inf, 1/self.lb)
        - if self spans zero -> Interval(-inf, +inf)
        """
        if self.is_empty():
            return Interval.empty_set()
        # exact zero interval
        if self.lb == 0.0 and self.ub == 0.0:
            return Interval.empty_set()
        # does not contain 0
        if (self.lb > 0.0) or (self.ub < 0.0):
            return Interval(1.0 / self.ub, 1.0 / self.lb)
        # left endpoint zero
        if self.lb == 0.0 and self.ub > 0.0:
            return Interval(1.0 / self.ub, math.inf)
        # right endpoint zero
        if self.lb < 0.0 and self.ub == 0.0:
            return Interval(-math.inf, 1.0 / self.lb)
        # spans zero
        return Interval(-math.inf, math.inf)

    def __rtruediv__(self, other):
        """Handle numeric / Interval by promoting numeric to Interval and dividing."""
        # other is numeric
        v = float(other)
        return Interval(v, v) / self

    def __and__(self, other):
        """Intersection: self & other"""
        if not isinstance(other, Interval):
            return NotImplemented
        if self.is_empty() or other.is_empty():
            return Interval.empty_set()
        lb = max(self.lb, other.lb)
        ub = min(self.ub, other.ub)
        if lb > ub:
            return Interval.empty_set()
        return Interval(lb, ub)

    def __or__(self, other):
        """Union-like operation: self | other

        If either interval is empty, return the other. Otherwise return
        the interval spanning both: [min(lb), max(ub)].
        """
        if not isinstance(other, Interval):
            return NotImplemented
        if self.is_empty():
            return other
        if other.is_empty():
            return self
        lb = min(self.lb, other.lb)
        ub = max(self.ub, other.ub)
        return Interval(lb, ub)

    def sqr(self):
        raise AttributeError("sqr was removed; use pow (e.g. x**2) instead")


    def __pow__(self, exponent):
        """Raise interval elements to an integer power.

        Supported exponent types: int (positive, zero, negative).

        Semantics:
        - empty interval -> empty
        - exponent == 0 -> Interval(1.0, 1.0) for non-empty intervals
        - positive integer n:
            - if n is odd: monotone, map endpoints -> [lb**n, ub**n]
            - if n is even: include 0 if interval spans zero; otherwise take min/max
              of endpoint powers
        - negative integer n: compute positive power and return its multiplicative inverse

        Raises TypeError for non-integer exponents.
        """
        if self.is_empty():
            return Interval.empty_set()

        if not isinstance(exponent, int):
            raise TypeError("Exponent must be an integer")

        # zero-th power
        if exponent == 0:
            return Interval(1.0, 1.0)

        # negative exponent: compute positive power then inverse
        if exponent < 0:
            pos = self.__pow__(-exponent)
            return pos.inverse()

        # exponent > 0
        n = exponent
        # fast-path for n == 1
        if n == 1:
            return Interval(self.lb, self.ub) if not self.is_empty() else Interval.empty_set()

        # For odd n the power function is monotone increasing: map endpoints
        if (n % 2) == 1:
            return Interval(self.lb ** n, self.ub ** n)

        # For even n, x->x^n is symmetric: consider whether interval spans zero
        a = self.lb ** n
        b = self.ub ** n
        if self.lb <= 0.0 and self.ub >= 0.0:
            # spans zero -> minimum is 0
            return Interval(0.0, max(a, b))
        else:
            return Interval(min(a, b), max(a, b))


def exp(x):
    """Return the interval of exponentials for x.

    Mirrors the simple C++ mapping exp applied to endpoints since exp is
    monotone: Interval(exp(lb), exp(ub)). Accepts Interval or numeric.
    """
    if isinstance(x, Interval):
        if x.is_empty():
            return Interval.empty_set()
        return Interval(math.exp(x.lb), math.exp(x.ub))
    else:
        v = float(x)
        return Interval(math.exp(v), math.exp(v))

def sqrt(x):
    """Return the interval of square roots for x.

    Mirrors the C++ implementation:
        Interval x_ = Interval(0,oo) & x;
        return Interval(sqrt(x_.lb()), sqrt(x_.ub()));

    Accepts an Interval or a numeric value. If the intersection with
    [0, +inf) is empty, returns the canonical empty interval.
    """
    if isinstance(x, Interval):
        # intersect with [0, +inf)
        x_pos = Interval(0.0, math.inf) & x
    else:
        # numeric input: promote to Interval
        x_pos = Interval(0.0, math.inf) & Interval(float(x), float(x))

    if x_pos.is_empty():
        return Interval.empty_set()

    return Interval(math.sqrt(x_pos.lb), math.sqrt(x_pos.ub))


def log(x):
    """Return the interval of natural logarithms for x.

    Implements the C++ semantics:
      if x.ub() <= 0 -> empty set
      else if x contains 0 -> Interval(-inf, log(x.ub()))
      else -> Interval(log(x.lb()), log(x.ub()))

    Accepts Interval or numeric inputs.
    """
    if isinstance(x, Interval):
        if x.is_empty():
            return Interval.empty_set()
        # if upper bound <= 0, domain empty
        if x.ub <= 0.0:
            return Interval.empty_set()
        # contains 0 -> lower is -inf, upper is log(ub) (ub > 0 here)
        if 0.0 in x:
            return Interval(-math.inf, math.log(x.ub))
        # strictly positive interval
        return Interval(math.log(x.lb), math.log(x.ub))
    else:
        v = float(x)
        if v <= 0.0:
            return Interval.empty_set()
        return Interval(math.log(v), math.log(v))


def min(x, y):
    """Return the interval min of x and y.

    Implements: Interval(min(x.lb, y.lb), min(x.ub, y.ub))

    Accepts Interval or numeric inputs for both x and y.
    """
    # promote to Interval if needed
    if not isinstance(x, Interval):
        x = Interval(float(x))
    if not isinstance(y, Interval):
        y = Interval(float(y))
    
    if x.is_empty() or y.is_empty():
        return Interval.empty_set()
    
    return Interval(min(x.lb, y.lb), min(x.ub, y.ub))

def max(x, y):
    """Return the interval max of x and y.

    Implements: Interval(max(x.lb, y.lb), max(x.ub, y.ub))

    Accepts Interval or numeric inputs for both x and y.
    """
    # promote to Interval if needed
    if not isinstance(x, Interval):
        x = Interval(float(x))
    if not isinstance(y, Interval):
        y = Interval(float(y))
    
    if x.is_empty() or y.is_empty():
        return Interval.empty_set()
    
    return Interval(max(x.lb, y.lb), max(x.ub, y.ub))



class Interval2D:
    def __init__(self, *args):
        if 
