"""Path planning for scout boats in formation with mothership.

This module computes target points for two scout boats (A and B) that maintain
a V-formation in front of the mothership. The boats are positioned symmetrically
at equal distances from the mothership's centerline.
"""

import math


def compute_target_points(mothership_state, scout_A_state, scout_B_state, 
                          distance=6.0):
    """Compute target points for scouts A and B based on mothership position.

    The scouts are positioned to form an equilateral triangle with the mothership:
    - Mothership is at the rear vertex of the triangle
    - Scouts A and B are at the two front vertices
    - All three sides have equal length (equilateral triangle)
    - The triangle points in the mothership's heading direction
    
    Parameters:
    -----------
    mothership_state : list or tuple
        [x, y, theta, vx, vy, omega] - mothership state
    scout_A_state : list or tuple
        [x, y, theta, vx, vy, omega] - current state of scout A
    scout_B_state : list or tuple
        [x, y, theta, vx, vy, omega] - current state of scout B
    distance : float
        Length of each side of the equilateral triangle (meters)
    lateral_separation : float, optional
        Deprecated parameter kept for backward compatibility (ignored)
    
    Returns:
    --------
    tuple : (target_A, target_B)
        target_A : [x, y] - target position for scout A (left front vertex)
        target_B : [x, y] - target position for scout B (right front vertex)
    """
    # Extract mothership position and heading
    ms_x, ms_y, ms_theta = mothership_state[0], mothership_state[1], mothership_state[2]
    
    # For an equilateral triangle with mothership at rear vertex:
    # - Each scout is at distance = side_length from mothership
    # - Each scout is at ±30° from the heading direction
    # - This ensures all three sides have equal length
    
    side = distance
    
    # Angle offset for equilateral triangle: 30 degrees = pi/6 radians
    angle_offset = math.pi / 6.0
    
    # Boat A target: at angle (heading + 30°), distance = side_length
    angle_A = ms_theta + angle_offset
    target_A = [
        ms_x + side * math.cos(angle_A),
        ms_y + side * math.sin(angle_A)
    ]
    
    # Boat B target: at angle (heading - 30°), distance = side_length
    angle_B = ms_theta - angle_offset
    target_B = [
        ms_x + side * math.cos(angle_B),
        ms_y + side * math.sin(angle_B)
    ]
    
    return target_A, target_B



def compute_target_points_2(mothership_state, scout_A_state, scout_B_state, distance=6.0):
    """
    Computes the target position for the scouts based on the positions of the other scout
    and the Mothership, but no information on the heading of the Motherboat.

    The scouts are positioned to keep a constant distance to the other boats.
    This creates a triangle that rotates around the Mothership.

    Parameters:
    -----------
    mothership_state : list or tuple
        [x, y, theta, vx, vy, omega] - mothership state
    scout_A_state : list or tuple
        [x, y, theta, vx, vy, omega] - current state of scout A
    scout_B_state : list or tuple
        [x, y, theta, vx, vy, omega] - current state of scout B
    distance : float
        Desired distance between boats and from mothership (meters). Default is 6.0.
    
    Returns:
    --------
    tuple : (target_A, target_B)
        target_A : [x, y] - target position for scout A
        target_B : [x, y] - target position for scout B
    """
    # extract positions
    ms_x, ms_y = mothership_state[0], mothership_state[1]
    a_x, a_y = scout_A_state[0], scout_A_state[1]
    b_x, b_y = scout_B_state[0], scout_B_state[1]

    # Compute current midpoint between scouts
    mid_x = (a_x + b_x) / 2
    mid_y = (a_y + b_y) / 2

    # Vector from mothership to midpoint
    vec_x = mid_x - ms_x
    vec_y = mid_y - ms_y
    vec_len = math.hypot(vec_x, vec_y)
    if vec_len < 1e-6:
        vec_x, vec_y = 1.0, 0.0
        vec_len = 1.0

    # Normalize direction
    dir_x = vec_x / vec_len
    dir_y = vec_y / vec_len

    # Place midpoint at the desired distance from mothership
    target_mid_x = ms_x + dir_x * distance
    target_mid_y = ms_y + dir_y * distance

    # Vector perpendicular to direction
    perp_x = -dir_y
    perp_y = dir_x

    # 🔧 FIX: use fixed desired separation (not current one)
    desired_separation = distance

    # Compute target positions for A and B symmetrically
    target_A = [
        target_mid_x + perp_x * (desired_separation / 2),
        target_mid_y + perp_y * (desired_separation / 2)
    ]
    target_B = [
        target_mid_x - perp_x * (desired_separation / 2),
        target_mid_y - perp_y * (desired_separation / 2)
    ]

    return target_A, target_B
    

