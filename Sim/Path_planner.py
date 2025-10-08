"""Path planning for scout boats in formation with mothership.

This module computes target points for two scout boats (A and B) that maintain
a V-formation in front of the mothership. The boats are positioned symmetrically
at equal distances from the mothership's centerline.

All positions are represented as 2D column vectors: np.array([[x], [y]])
"""

import math
import numpy as np


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
        target_A : np.array([[x], [y]]) - target position for scout A (left front vertex)
        target_B : np.array([[x], [y]]) - target position for scout B (right front vertex)
    """
    # Extract mothership position and heading
    ms_pos = np.array([[mothership_state[0]], [mothership_state[1]]])
    ms_theta = mothership_state[2]
    
    # For an equilateral triangle with mothership at rear vertex:
    # - Each scout is at distance = side_length from mothership
    # - Each scout is at ±30° from the heading direction
    # - This ensures all three sides have equal length
    
    side = distance
    
    # Angle offset for equilateral triangle: 30 degrees = pi/6 radians
    angle_offset = np.pi / 6.0
    
    # Boat A target: at angle (heading + 30°), distance = side_length
    angle_A = ms_theta + angle_offset
    target_A = ms_pos + side * np.array([[np.cos(angle_A)], [np.sin(angle_A)]])
    
    # Boat B target: at angle (heading - 30°), distance = side_length
    angle_B = ms_theta - angle_offset
    target_B = ms_pos + side * np.array([[np.cos(angle_B)], [np.sin(angle_B)]])
    
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
    last_ms_position: np.array([[x], [y]])
        Previous mothership position as 2D column vector
    distance : float
        Desired distance between boats and from mothership (meters). Default is 6.0.
    
    Returns:
    --------
    tuple : (target_A, target_B, updated_last_ms_position)
        target_A : np.array([[x], [y]]) - target position for scout A
        target_B : np.array([[x], [y]]) - target position for scout B
        updated_last_ms_position : np.array([[x], [y]]) - current MS position for next iteration
    """
    # extract position of Mothership as column vector
    ms_pos = np.array([[mothership_state[0]], [mothership_state[1]]])

    # estimate heading of Motherboat from movement
    dx_vec = ms_pos - last_ms_position
    norm = np.linalg.norm(dx_vec)
    
    if norm < 1e-6:
        # Default direction if no movement
        dir_vec = np.array([[1.0], [0.0]])
    else:
        dir_vec = dx_vec / norm

    # Place midpoint at the desired distance from mothership
    target_mid = ms_pos + dir_vec * distance

    # Vector perpendicular to direction (rotate 90° left)
    perp_vec = np.array([[-dir_vec[1, 0]], [dir_vec[0, 0]]])

    # Compute target positions for A and B symmetrically
    target_A = target_mid + perp_vec * (distance / 2)
    target_B = target_mid - perp_vec * (distance / 2)
    
    # Update last position
    updated_last_ms_position = ms_pos.copy()

    return target_A, target_B, updated_last_ms_position



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
    last_ms_position: np.array([[x], [y]])
        Previous mothership position as 2D column vector
    t: float
        Current time (seconds) to modulate the formation update rate.
    distance : float
        Desired distance between boats and from mothership (meters). Default is 6.0.
    
    Returns:
    --------
    tuple : (target_A, target_B, updated_last_ms_position)
        target_A : np.array([[x], [y]]) or None - target position for scout A
        target_B : np.array([[x], [y]]) or None - target position for scout B
        updated_last_ms_position : np.array([[x], [y]]) - MS position for next iteration
    """
    if t % 2.0 <= 1.9:  # Output None during the first 1.9 seconds of each 2s period
        return None, None, last_ms_position
    
    # extract position of Mothership as column vector
    ms_pos = np.array([[mothership_state[0]], [mothership_state[1]]])

    # estimate heading of Motherboat from movement
    dx_vec = ms_pos - last_ms_position
    norm = np.linalg.norm(dx_vec)
    
    if norm < 1e-6:
        # Default direction if no movement
        dir_vec = np.array([[1.0], [0.0]])
    else:
        dir_vec = dx_vec / norm

    # Place midpoint at the desired distance from mothership
    target_mid = ms_pos + dir_vec * distance

    # Vector perpendicular to direction (rotate 90° left)
    perp_vec = np.array([[-dir_vec[1, 0]], [dir_vec[0, 0]]])

    # Compute target positions for A and B symmetrically
    target_A = target_mid + perp_vec * (distance / 2)
    target_B = target_mid - perp_vec * (distance / 2)
    
    # Update last position
    updated_last_ms_position = ms_pos.copy()

    return target_A, target_B, updated_last_ms_position



def compute_target_points_4(mothership_state, scout_A_state, scout_B_state, estimation_A, estimation_B, distance=6.0):
    """
    Computes the target position for the scouts based on the distance to the other scout
    and the Mothership.

    The scouts are positioned to keep a constant distance to the other boats.

    Parameters:
    -----------
    mothership_state : list or tuple
        [x, y, theta, vx, vy, omega] - mothership state
    scout_A_state : list or tuple
        [x, y, theta, vx, vy, omega] - scout A state
    scout_B_state : list or tuple
        [x, y, theta, vx, vy, omega] - scout B state
    estimation_A : list or tuple
        [A_estimated_MS, A_estimated_B] - estimated positions of MS and B from scout A in the global frame
    estimation_B : list or tuple
        [B_estimated_MS, B_estimated_A] - estimated positions of MS and A from scout B in the global frame
    distance : float
        Desired distance between boats and from mothership (meters). Default is 6.0.
    """
    # extract all positions
    pos_ms = np.array([[mothership_state[0]], [mothership_state[1]]])
    pos_a = np.array([[scout_A_state[0]], [scout_A_state[1]]])
    pos_b = np.array([[scout_B_state[0]], [scout_B_state[1]]])

    # compute all distances
    dist_ms_a = np.norm(pos_ms - pos_a)
    dist_ms_b = np.norm(pos_ms - pos_b)
    dist_a_b = np.norm(pos_a - pos_b)

    # Scout A
    # estimation des positions des bateaux
    if estimation_A is not None:
        # ancienne estimation
        A_estimated_MS = estimation_A[0]
        A_estimated_B = estimation_A[1]
        # nouvelle estimation à partir des distances
        A_estimated_MS = 0.8 * A_estimated_MS + 0.2 * dist_ms_a
        A_estimated_B = 0.8 * A_estimated_B + 0.2 * dist_a_b


    # Scout B




    
    target_A, target_B = None, None

    return target_A, target_B