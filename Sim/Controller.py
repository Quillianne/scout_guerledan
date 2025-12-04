import numpy as np


def controller(state, goal):
    """Compute left/right thrusts from boat state and a 2×1 goal column vector.

    The function accepts legacy inputs (lists/1-D arrays) or the newer
    numpy column vectors (shape (2,1) for positions or shape (6,1) for full
    state). Internally positions are treated as 2×1 column arrays.

    Parameters
    - state: sequence or numpy array containing [x, y, theta, vx, vy, omega]
    - goal: sequence or numpy array containing [x_goal, y_goal] (or None)

    Returns
    - numpy.ndarray shape (2,) with [thrust_left, thrust_right]
    """
    if goal is None:
        return np.array([5.0, 5.0])  # no goal, small forward command

    # Normalize inputs to 1-D arrays so indexing works for lists and arrays
    s = np.asarray(state).reshape(-1)
    g = np.asarray(goal).reshape(-1)

    # Extract scalars robustly (works for lists, 1-D arrays, or column arrays)
    try:
        x = float(s[0]); y = float(s[1]); theta = float(s[2])
    except Exception:
        raise ValueError("state must contain at least [x, y, theta, ...]")

    try:
        xg = float(g[0]); yg = float(g[1])
    except Exception:
        raise ValueError("goal must contain [x_goal, y_goal]")

    # column vectors (2x1)
    current_coords = np.array([[x], [y]])
    target_coords = np.array([[xg], [yg]])

    # gains and parameters
    Kp = 0.5
    reference_distance = 1.0
    max_speed = 10.0

    delta = target_coords - current_coords  # 2x1
    distance_target = float(np.linalg.norm(delta))

    if distance_target > reference_distance:
        # compute headings using column components
        target_heading = np.degrees(np.arctan2(float(delta[1, 0]), float(delta[0, 0])))
        current_heading = np.degrees(float(theta))

        # angular error in degrees normalized to [-180,180]
        error = current_heading - target_heading
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        correction = Kp * error

        # distance-based feedforward (smooth ramp)
        distance_correction = np.tanh(distance_target / reference_distance)

        # Proportional command to the motors
        base_speed = max_speed * 0.9
        left_motor = distance_correction * base_speed + correction
        right_motor = distance_correction * base_speed - correction

        # Clip motor speeds within valid range
        left_motor = np.clip(left_motor, -max_speed, max_speed)
        right_motor = np.clip(right_motor, -max_speed, max_speed)

        return np.array([float(left_motor), float(right_motor)])

    # Target reached
    return np.array([0.0, 0.0])