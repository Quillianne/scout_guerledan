"""
Fichier test pour apprendre à utiliser CODAC
"""

import codac
import math

print("=" * 60)
print("CODAC - Interval Arithmetic Demo")
print("=" * 60)

# 1. Basic Interval creation
print("\n1. Creating Intervals:")
a = codac.Interval(2, 5)
b = codac.Interval(3, 7)
print(f"a = {a}")
print(f"b = {b}")

# 2. Interval arithmetic
print("\n2. Interval Arithmetic:")
print(f"a + b = {a + b}")
print(f"a - b = {a - b}")
print(f"a * b = {a * b}")
print(f"a / b = {a / b}")

# 3. Interval functions
print("\n3. Mathematical Functions:")
x = codac.Interval(1, 4)
print(f"x = {x}")
print(f"sqrt(x) = {codac.sqrt(x)}")
print(f"exp(x) = {codac.exp(x)}")
print(f"log(x) = {codac.log(x)}")
print(f"x^2 = {codac.sqr(x)}")

# 4. Trigonometric functions
print("\n4. Trigonometric Functions:")
theta = codac.Interval(0, math.pi/2)
print(f"theta = {theta}")
print(f"cos(theta) = {codac.cos(theta)}")
print(f"sin(theta) = {codac.sin(theta)}")

# 5. Interval vector (2D)
print("\n5. Interval Vectors (2D boxes):")
box = codac.IntervalVector(2)  # 2D unbounded box
box[0] = codac.Interval(-5, 5)
box[1] = codac.Interval(-3, 3)
print(f"box = {box}")
print(f"box[0] = {box[0]}")
print(f"box[1] = {box[1]}")

# 6. Operations on IntervalVectors
print("\n6. IntervalVector Operations:")
v1 = codac.IntervalVector([[1, 3], [2, 4]])
v2 = codac.IntervalVector([[0, 2], [1, 3]])
print(f"v1 = {v1}")
print(f"v2 = {v2}")
print(f"v1 + v2 = {v1 + v2}")
print(f"Intersection: v1 & v2 = {v1 & v2}")

# 7. Set operations
print("\n7. Set Operations:")
i1 = codac.Interval(1, 5)
i2 = codac.Interval(3, 7)
print(f"i1 = {i1}")
print(f"i2 = {i2}")
print(f"Union: i1 | i2 = {i1 | i2}")
print(f"Intersection: i1 & i2 = {i1 & i2}")
print(f"Is 4 in i1? {4 in i1}")
print(f"Is 6 in i1? {6 in i1}")

# 8. Constraint propagation example
print("\n8. Simple Constraint Propagation:")
print("Given: x + y = 10 with x in [2,5] and y in [3,8]")
x = codac.Interval(2, 5)
y = codac.Interval(3, 8)
target = codac.Interval(10, 10)
print(f"Before: x = {x}, y = {y}")
# Forward constraint: x + y should be in target
# Backward: refine x and y
y_refined = target - x
x_refined = target - y
x = x & x_refined
y = y & y_refined
print(f"After:  x = {x}, y = {y}")

# 9. Distance calculation
print("\n9. Distance Between Boxes:")
p1 = codac.IntervalVector([[1, 2], [3, 4]])
p2 = codac.IntervalVector([[5, 6], [7, 8]])
print(f"p1 = {p1}")
print(f"p2 = {p2}")
# Note: distance calculation may vary by codac version
print(f"p1.max_diam() = {p1.max_diam()}")
print(f"p2.max_diam() = {p2.max_diam()}")

# 10. Empty and unbounded intervals
print("\n10. Special Intervals:")
empty = codac.Interval.EMPTY_SET
unbounded = codac.Interval.ALL_REALS
print(f"Empty interval: {empty}")
print(f"Unbounded interval: {unbounded}")
print(f"Is empty set empty? {empty.is_empty()}")

print("\n" + "=" * 60)
print("Demo completed!")
print("=" * 60)





# ============================================================
# CONTRACTORS DEMONSTRATION
# ============================================================
print("\n" + "=" * 60)
print("CODAC - Contractors Demo")
print("=" * 60)

# 11. Basic Contractor: CtcFwdBwd
print("\n11. Forward-Backward Contractor:")
print("Constraint: x^2 + y^2 = 25 (circle of radius 5)")
from codac import Function, CtcFwdBwd

# Define the constraint function
f = Function("x", "y", "x^2 + y^2 - 25")
ctc = CtcFwdBwd(f)

# Initial box (large domain)
box = codac.IntervalVector([[-10, 10], [-10, 10]])
print(f"Before contraction: {box}")
ctc.contract(box)
print(f"After contraction:  {box}")

# 12. Contractor for equality constraint
print("\n12. Equality Contractor:")
print("Constraint: x + y = 5")
f_eq = Function("x", "y", "x + y - 5")
ctc_eq = CtcFwdBwd(f_eq)

box_eq = codac.IntervalVector([[0, 10], [0, 10]])
print(f"Before: {box_eq}")
ctc_eq.contract(box_eq)
print(f"After:  {box_eq}")

# 13. Distance contractor
print("\n13. Distance Contractor:")
print("Constraint: distance from point (3,4) to (x,y) in [2,6]")
f_dist = Function("x", "y", "sqrt((x-3)^2 + (y-4)^2)")
ctc_dist = CtcFwdBwd(f_dist, codac.Interval(2, 6))

box_dist = codac.IntervalVector([[-5, 10], [-5, 10]])
print(f"Before: {box_dist}")
ctc_dist.contract(box_dist)
print(f"After:  {box_dist}")

# 14. Union of contractors
print("\n14. Union of Contractors (CtcUnion):")
print("Constraint: x^2 + y^2 <= 9 OR (x-5)^2 + (y-5)^2 <= 4")
from codac import CtcUnion

f1 = Function("x", "y", "x^2 + y^2 - 9")
f2 = Function("x", "y", "(x-5)^2 + (y-5)^2 - 4")
ctc1 = CtcFwdBwd(f1)
ctc2 = CtcFwdBwd(f2)
ctc_union = CtcUnion(ctc1, ctc2)  # Pass contractors directly, not as list

box_union = codac.IntervalVector([[2, 8], [2, 8]])
print(f"Before: {box_union}")
ctc_union.contract(box_union)
print(f"After:  {box_union}")

# 15. Composition of contractors
print("\n15. Composition (multiple constraints):")
print("Constraints: x + y = 10 AND x - y = 2")
f_sum = Function("x", "y", "x + y - 10")
f_diff = Function("x", "y", "x - y - 2")
ctc_sum = CtcFwdBwd(f_sum)
ctc_diff = CtcFwdBwd(f_diff)

box_comp = codac.IntervalVector([[0, 20], [0, 20]])
print(f"Before: {box_comp}")
ctc_sum.contract(box_comp)
print(f"After ctc_sum: {box_comp}")
ctc_diff.contract(box_comp)
print(f"After ctc_diff: {box_comp}")
# Result should be close to x=6, y=4

# 16. Fixed-point contractor
print("\n16. Fixed-Point Iteration:")
print("Applying contractors repeatedly until convergence")
box_fp = codac.IntervalVector([[0, 20], [0, 20]])
print(f"Initial: {box_fp}")
for i in range(5):
    box_before = codac.IntervalVector(box_fp)
    ctc_sum.contract(box_fp)
    ctc_diff.contract(box_fp)
    if box_fp == box_before:
        print(f"Converged at iteration {i+1}: {box_fp}")
        break
    print(f"Iteration {i+1}: {box_fp}")

# 17. Polar to Cartesian contractor
print("\n17. Polar to Cartesian Contractor:")
print("Constraint: x = r*cos(theta), y = r*sin(theta)")
from codac import CtcPolar

# Create a box for [x, y, r, theta]
box_polar = codac.IntervalVector(4)
box_polar[0] = codac.Interval(1, 3)      # x in [1, 3]
box_polar[1] = codac.Interval(1, 3)      # y in [1, 3]
box_polar[2] = codac.Interval(0, 10)     # r in [0, 10]
box_polar[3] = codac.Interval(-math.pi, math.pi)  # theta in [-π, π]

print(f"Before: x={box_polar[0]}, y={box_polar[1]}, r={box_polar[2]}, theta={box_polar[3]}")
ctc_polar = CtcPolar()
ctc_polar.contract(box_polar)
print(f"After:  x={box_polar[0]}, y={box_polar[1]}, r={box_polar[2]}, theta={box_polar[3]}")

# 18. Example: Localization with distance measurements
print("\n18. Robot Localization Example:")
print("Robot at unknown position (x,y), with distance measurements to beacons")

# Beacons at known positions
beacon1 = (0, 0)
beacon2 = (10, 0)
beacon3 = (5, 8)

# True robot position is around (4, 3)
# Distance measurements (with uncertainty)
d1 = codac.Interval(4.5, 5.5)  # distance to beacon1: sqrt(16+9)=5
d2 = codac.Interval(5.8, 6.8)  # distance to beacon2: sqrt(36+9)=6.7
d3 = codac.Interval(5.0, 6.0)  # distance to beacon3: sqrt(1+25)=5.1

# Robot position (unknown, large initial domain)
robot_pos = codac.IntervalVector([[-5, 15], [-5, 15]])
print(f"Initial domain: {robot_pos}")

# Create contractors for each distance constraint
f1 = Function("x", "y", f"sqrt((x-{beacon1[0]})^2 + (y-{beacon1[1]})^2)")
f2 = Function("x", "y", f"sqrt((x-{beacon2[0]})^2 + (y-{beacon2[1]})^2)")
f3 = Function("x", "y", f"sqrt((x-{beacon3[0]})^2 + (y-{beacon3[1]})^2)")

ctc1_loc = CtcFwdBwd(f1, d1)
ctc2_loc = CtcFwdBwd(f2, d2)
ctc3_loc = CtcFwdBwd(f3, d3)

# Apply contractors iteratively
for iteration in range(10):
    robot_before = codac.IntervalVector(robot_pos)
    ctc1_loc.contract(robot_pos)
    ctc2_loc.contract(robot_pos)
    ctc3_loc.contract(robot_pos)
    if robot_pos == robot_before:
        break

print(f"Estimated position after {iteration+1} iterations: {robot_pos}")
if not robot_pos.is_empty():
    print(f"Midpoint: ({robot_pos[0].mid():.2f}, {robot_pos[1].mid():.2f})")
    print(f"Uncertainty: ±{robot_pos[0].diam()/2:.2f}m in x, ±{robot_pos[1].diam()/2:.2f}m in y")
else:
    print("No solution found (inconsistent measurements)")

print("\n" + "=" * 60)
print("Contractors Demo completed!")
print("=" * 60)


