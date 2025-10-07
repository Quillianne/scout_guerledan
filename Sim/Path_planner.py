"""Path planning for scout boats in formation with mothership.

This module computes target points for two scout boats (A and B) that maintain
a V-formation in front of the mothership. The boats are positioned symmetrically
at equal distances from the mothership's centerline.
"""

import math


def compute_target_points(mothership_state, distance=6.0):
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
    distance : float
        Length of each side of the equilateral triangle (meters)
    
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



def compute_target_points_2(mothership_state, last_ms_position, distance=6.0):
    """
    Computes the target position for the scouts based on the positions of the Mothership, 
    but no information on the heading of the Motherboat.

    The scouts are positioned to keep a constant distance to the other boats.
    The heading of the Motherboat is estimated using its previous position.
    This creates a triangle that rotates around the Mothership.

    Parameters:
    -----------
    mothership_state : list or tuple
        [x, y, theta, vx, vy, omega] - mothership state
    last_ms_position: list or tuple
        [last_ms_x, last_ms_y]
    distance : float
        Desired distance between boats and from mothership (meters). Default is 6.0.
    
    Returns:
    --------
    tuple : (target_A, target_B)
        target_A : [x, y] - target position for scout A
        target_B : [x, y] - target position for scout B
    """
    # extract position of Mothership
    ms_x, ms_y = mothership_state[0], mothership_state[1]

    # estimate heading of Motherboat
    prev_x, prev_y = last_ms_position[0], last_ms_position[1]
    dx = ms_x - prev_x
    dy = ms_y - prev_y
    norm = math.hypot(dx, dy)
    if norm == 0:
        dir_x, dir_y = 1.0, 0.0  # Default direction if no movement
    else:
        dir_x, dir_y = dx / norm, dy / norm

    # Place midpoint at the desired distance from mothership
    target_mid_x = ms_x + dir_x * distance
    target_mid_y = ms_y + dir_y * distance

    # Vector perpendicular to direction
    perp_x = -dir_y
    perp_y = dir_x

    # Compute target positions for A and B symmetrically
    target_A = [
        target_mid_x + perp_x * (distance / 2),
        target_mid_y + perp_y * (distance / 2)
    ]
    target_B = [
        target_mid_x - perp_x * (distance / 2),
        target_mid_y - perp_y * (distance / 2)
    ]
    last_ms_position = (ms_x, ms_y)

    return target_A, target_B, last_ms_position



def compute_target_points_3(mothership_state, last_ms_position, t:float, distance=6.0):
    """
    Computes the target position for the scouts based on the positions of the Mothership (with low refresh rate), 
    but no information on the heading of the Motherboat.

    The scouts are positioned to keep a constant distance to the other boats.
    The heading of the Motherboat is estimated using its previous position.
    This creates a triangle that rotates around the Mothership.

    Parameters:
    -----------
    mothership_state : list or tuple
        [x, y, theta, vx, vy, omega] - mothership state
    last_ms_position: list or tuple
        [last_ms_x, last_ms_y]
    t: float
        Current time (seconds) to modulate the formation update rate.
    distance : float
        Desired distance between boats and from mothership (meters). Default is 6.0.
    
    Returns:
    --------
    tuple : (target_A, target_B)
        target_A : [x, y] - target position for scout A
        target_B : [x, y] - target position for scout B
    """
    if t % 2.0 <= 1.9:  # Output None during the first 1.9 seconds of each 2s period
        return None, None, last_ms_position
    
    # extract position of Mothership
    ms_x, ms_y = mothership_state[0], mothership_state[1]

    # estimate heading of Motherboat
    prev_x, prev_y = last_ms_position[0], last_ms_position[1]
    dx = ms_x - prev_x
    dy = ms_y - prev_y
    norm = math.hypot(dx, dy)
    if norm == 0:
        dir_x, dir_y = 1.0, 0.0  # Default direction if no movement
    else:
        dir_x, dir_y = dx / norm, dy / norm

    # Place midpoint at the desired distance from mothership
    target_mid_x = ms_x + dir_x * distance
    target_mid_y = ms_y + dir_y * distance

    # Vector perpendicular to direction
    perp_x = -dir_y
    perp_y = dir_x

    # Compute target positions for A and B symmetrically
    target_A = [
        target_mid_x + perp_x * (distance / 2),
        target_mid_y + perp_y * (distance / 2)
    ]
    target_B = [
        target_mid_x - perp_x * (distance / 2),
        target_mid_y - perp_y * (distance / 2)
    ]
    last_ms_position = (ms_x, ms_y)

    return target_A, target_B, last_ms_position



def compute_target_points_4(mothership_state, scout_A_state, scout_B_state, last_ms_position, distance=6.0):
    """
    Computes the target position for the scouts based on the distance to the other scout
    and the Mothership.

    The scouts are positioned to keep a constant distance to the other boats.
    """
    # extract all positions
    ms_x, ms_y = 
    sa_x, sa_y = 
    sb_x, sb_y = 



    return target_A, target_B