import codac as cd
import matplotlib.pyplot as plt

from Interval_observer import IntervalObserver


x = cd.IntervalVector([[1,2], [2,3]])
y = cd.IntervalVector([3, 4])

z = cd.IntervalVector([0])

print("Interval x:", x)
print("Interval y:", y)
print("Empty Interval z:", z)