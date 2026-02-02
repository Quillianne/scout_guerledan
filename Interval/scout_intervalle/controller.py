#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Contrôleur pour les scouts dans la simulation de flotte de bateaux."""


from codac import *
from boat import Boat
from fleet import Fleet
import numpy as np


def interval_controller(current_position_box: IntervalVector, 
                       target_position_box: IntervalVector,
                       current_heading_interval: Interval,
                       Kp: float = 0.5,
                       reference_distance: float = 1.0,
                       max_speed: float = 10.0) -> tuple:
    """
    Compute left/right thrusts from interval-based position and heading.
    
    Args:
        current_position_box: IntervalVector [x, y] representing current position uncertainty
        target_position_box: IntervalVector [x, y] representing target position
        current_heading_interval: Interval representing current heading uncertainty (radians)
        Kp: Proportional gain for angular correction
        reference_distance: Reference distance for speed scaling (meters)
        max_speed: Maximum motor speed
        
    Returns:
        tuple: (left_thrust, right_thrust) as floats
    """
    # Extract midpoints for control (robust approach: use center of intervals)
    x_current = current_position_box[0].mid()
    y_current = current_position_box[1].mid()
    x_target = target_position_box[0].mid()
    y_target = target_position_box[1].mid()
    theta = current_heading_interval.mid()
    
    # Compute position error
    delta_x = x_target - x_current
    delta_y = y_target - y_current
    
    # Distance to target
    distance_target = np.sqrt(delta_x**2 + delta_y**2)
    
    # Check if we're close enough to the target
    if distance_target <= reference_distance:
        return (0.0, 0.0)  # Stop when target is reached
    
    # Compute desired heading
    target_heading_rad = np.arctan2(delta_y, delta_x)
    target_heading_deg = np.degrees(target_heading_rad)
    current_heading_deg = np.degrees(theta)
    
    # Angular error normalized to [-180, 180]
    error_deg = current_heading_deg - target_heading_deg
    if error_deg > 180:
        error_deg -= 360
    elif error_deg < -180:
        error_deg += 360
    
    # Proportional correction
    correction = Kp * error_deg
    
    # Distance-based feedforward (smooth ramp using tanh)
    distance_correction = np.tanh(distance_target / reference_distance)
    
    # Compute motor commands
    base_speed = max_speed * 0.9
    left_motor = distance_correction * base_speed + correction
    right_motor = distance_correction * base_speed - correction
    
    # Clip motor speeds
    left_motor = np.clip(left_motor, -max_speed, max_speed)
    right_motor = np.clip(right_motor, -max_speed, max_speed)
    
    return (float(left_motor), float(right_motor))


def robust_interval_controller(current_position_box: IntervalVector,
                               target_position_box: IntervalVector,
                               current_heading_interval: Interval,
                               Kp: float = 0.5,
                               reference_distance: float = 1.0,
                               max_speed: float = 10.0,
                               use_pessimistic: bool = True) -> tuple:
    """
    Compute left/right thrusts using worst-case interval analysis.
    
    This version considers the uncertainty bounds for more robust control.
    
    Args:
        current_position_box: IntervalVector [x, y] representing current position uncertainty
        target_position_box: IntervalVector [x, y] representing target position
        current_heading_interval: Interval representing current heading uncertainty (radians)
        Kp: Proportional gain for angular correction
        reference_distance: Reference distance for speed scaling (meters)
        max_speed: Maximum motor speed
        use_pessimistic: If True, use pessimistic estimates (max distance, max error)
        
    Returns:
        tuple: (left_thrust, right_thrust) as floats
    """
    # Compute interval arithmetic for delta position
    delta_x_interval = target_position_box[0] - current_position_box[0]
    delta_y_interval = target_position_box[1] - current_position_box[1]
    
    # Distance interval (pessimistic: use maximum possible distance)
    # sqr returns x² as Interval
    dist_squared_interval = sqr(delta_x_interval) + sqr(delta_y_interval)
    
    if use_pessimistic:
        # Use maximum possible distance (pessimistic approach)
        distance_target = np.sqrt(dist_squared_interval.ub())
        # Use center of target for heading
        x_target = target_position_box[0].mid()
        y_target = target_position_box[1].mid()
        x_current = current_position_box[0].mid()
        y_current = current_position_box[1].mid()
    else:
        # Use midpoint estimates (optimistic approach)
        distance_target = np.sqrt(dist_squared_interval.mid())
        x_target = target_position_box[0].mid()
        y_target = target_position_box[1].mid()
        x_current = current_position_box[0].mid()
        y_current = current_position_box[1].mid()
    
    # Check if we're close enough
    if distance_target <= reference_distance:
        return (0.0, 0.0)
    
    # Compute target heading with interval awareness
    delta_x = x_target - x_current
    delta_y = y_target - y_current
    target_heading_rad = np.arctan2(delta_y, delta_x)
    target_heading_deg = np.degrees(target_heading_rad)
    
    # Use midpoint of current heading for control
    current_heading_deg = np.degrees(current_heading_interval.mid())
    
    # Angular error normalized to [-180, 180]
    error_deg = current_heading_deg - target_heading_deg
    if error_deg > 180:
        error_deg -= 360
    elif error_deg < -180:
        error_deg += 360
    
    # Proportional correction
    correction = Kp * error_deg
    
    # Distance-based feedforward
    distance_correction = np.tanh(distance_target / reference_distance)
    
    # Compute motor commands
    base_speed = max_speed * 0.9
    left_motor = distance_correction * base_speed + correction
    right_motor = distance_correction * base_speed - correction
    
    # Clip motor speeds
    left_motor = np.clip(left_motor, -max_speed, max_speed)
    right_motor = np.clip(right_motor, -max_speed, max_speed)
    
    return (float(left_motor), float(right_motor))


def compute_interval_target_points(mothership_position_box: IntervalVector,
                                   mothership_heading_interval: Interval,
                                   distance: float = 6.0) -> tuple:
    """
    Compute target interval boxes for scouts A and B based on mothership interval position.
    
    The scouts are positioned to form an equilateral triangle with the mothership:
    - Mothership is at the rear vertex of the triangle
    - Scouts A and B are at the two front vertices
    - All three sides have equal length (equilateral triangle)
    - The triangle points in the mothership's heading direction
    
    This version propagates uncertainties through interval arithmetic.
    
    Args:
        mothership_position_box: IntervalVector [x, y] representing mothership position uncertainty
        mothership_heading_interval: Interval representing mothership heading uncertainty (radians)
        distance: Length of each side of the equilateral triangle (meters)
        
    Returns:
        tuple: (target_A_box, target_B_box)
            target_A_box: IntervalVector [x, y] for scout A (left front vertex)
            target_B_box: IntervalVector [x, y] for scout B (right front vertex)
    """
    # Angle offset for equilateral triangle: 30 degrees = pi/6 radians
    angle_offset = np.pi / 6.0
    
    # Scout A target: at angle (heading + 30°), distance = side_length
    angle_A_interval = mothership_heading_interval + angle_offset
    
    # Compute position using interval trigonometry
    # x_A = x_ms + distance * cos(angle_A)
    # y_A = y_ms + distance * sin(angle_A)
    dx_A_interval = distance * cos(angle_A_interval)
    dy_A_interval = distance * sin(angle_A_interval)
    
    target_A_x = mothership_position_box[0] + dx_A_interval
    target_A_y = mothership_position_box[1] + dy_A_interval
    target_A_box = IntervalVector([target_A_x, target_A_y])
    
    # Scout B target: at angle (heading - 30°), distance = side_length
    angle_B_interval = mothership_heading_interval - angle_offset
    
    # Compute position using interval trigonometry
    # x_B = x_ms + distance * cos(angle_B)
    # y_B = y_ms + distance * sin(angle_B)
    dx_B_interval = distance * cos(angle_B_interval)
    dy_B_interval = distance * sin(angle_B_interval)
    
    target_B_x = mothership_position_box[0] + dx_B_interval
    target_B_y = mothership_position_box[1] + dy_B_interval
    target_B_box = IntervalVector([target_B_x, target_B_y])
    
    return target_A_box, target_B_box


if __name__ == "__main__":
    # Test the interval controller
    print("Test du contrôleur par intervalles\n")
    
    # Create test positions
    current_pos = IntervalVector([[0.0, 1.0], [0.0, 1.0]])  # Position with uncertainty
    target_pos = IntervalVector([[10.0, 10.5], [10.0, 10.5]])  # Target with small uncertainty
    current_heading = Interval(0.0, 0.1)  # Heading with small uncertainty (nearly east)
    
    print(f"Current position: {current_pos}")
    print(f"Target position: {target_pos}")
    print(f"Current heading: {current_heading} rad\n")
    
    # Test simple controller
    left, right = interval_controller(current_pos, target_pos, current_heading)
    print(f"Simple controller output:")
    print(f"  Left thrust: {left:.2f}")
    print(f"  Right thrust: {right:.2f}\n")
    
    # Test robust controller
    left_r, right_r = robust_interval_controller(current_pos, target_pos, current_heading)
    print(f"Robust controller output (pessimistic):")
    print(f"  Left thrust: {left_r:.2f}")
    print(f"  Right thrust: {right_r:.2f}\n")
    
    # Test when near target
    near_target = IntervalVector([[9.5, 10.0], [9.5, 10.0]])
    left_n, right_n = interval_controller(near_target, target_pos, current_heading)
    print(f"Controller output when near target:")
    print(f"  Left thrust: {left_n:.2f}")
    print(f"  Right thrust: {right_n:.2f}\n")
    
    # Test interval target computation
    print("="*60)
    print("Test du calcul des cibles par intervalles\n")
    
    mothership_pos = IntervalVector([[5.0, 6.0], [3.0, 4.0]])  # MS position with uncertainty
    mothership_heading = Interval(0.0, 0.2)  # Heading with uncertainty (nearly east, ~0-11°)
    
    print(f"Mothership position: {mothership_pos}")
    print(f"Mothership heading: {mothership_heading} rad")
    print(f"Distance: 6.0 m\n")
    
    target_A, target_B = compute_interval_target_points(mothership_pos, mothership_heading, distance=6.0)
    
    print(f"Scout A target box: {target_A}")
    print(f"  X range: [{target_A[0].lb():.2f}, {target_A[0].ub():.2f}] (width: {target_A[0].diam():.2f} m)")
    print(f"  Y range: [{target_A[1].lb():.2f}, {target_A[1].ub():.2f}] (width: {target_A[1].diam():.2f} m)\n")
    
    print(f"Scout B target box: {target_B}")
    print(f"  X range: [{target_B[0].lb():.2f}, {target_B[0].ub():.2f}] (width: {target_B[0].diam():.2f} m)")
    print(f"  Y range: [{target_B[1].lb():.2f}, {target_B[1].ub():.2f}] (width: {target_B[1].diam():.2f} m)\n")
    
    # Test with precise mothership position
    print("Test with precise mothership position:")
    ms_pos_precise = IntervalVector([[10.0, 10.0], [0.0, 0.0]])
    ms_heading_precise = Interval(np.pi/4, np.pi/4)  # Exactly 45 degrees
    
    target_A_p, target_B_p = compute_interval_target_points(ms_pos_precise, ms_heading_precise, distance=6.0)
    print(f"Scout A target (precise): center=({target_A_p[0].mid():.2f}, {target_A_p[1].mid():.2f})")
    print(f"Scout B target (precise): center=({target_B_p[0].mid():.2f}, {target_B_p[1].mid():.2f})")

