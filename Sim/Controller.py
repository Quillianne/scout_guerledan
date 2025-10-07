import numpy as np


def controller(state, goal):
    """
    Uses the current state and the goal to compute the inputs to the boat
    
    state: [x, y, theta, vx, vy, omega]
    goal: [x_goal, y_goal]
    
    returns: [thrust_left, thrust_right]
    """
    x, y, theta, vx, vy, omega = state
    xg, yg = goal
    target_coords = np.array([xg, yg])
    current_coords = np.array([x, y])
    current_heading = np.degrees(theta)  # in degrees

    # gain
    Kp = 0.5
    # reference distance
    distance = 1.0
    # max speed
    max_speed = 10.0

    delta_coords = target_coords - current_coords
    distance_target = np.linalg.norm(delta_coords)

    if distance_target > distance:
        target_heading = np.degrees(np.arctan2(delta_coords[1], delta_coords[0]))
            
        # Error
        error = current_heading - target_heading
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        correction = Kp * error

        # correction
        reference_distance = distance
        distance_correction = np.tanh(distance_target/reference_distance)

        # Proportional command to the motors
        base_speed = max_speed * 0.9
        left_motor = distance_correction*base_speed + correction
        right_motor = distance_correction*base_speed - correction

        # Clip motor speeds within valid range
        left_motor = np.clip(left_motor, -max_speed, max_speed)
        right_motor = np.clip(right_motor, -max_speed, max_speed)

        # Send speed commands to motors
        return [left_motor, right_motor]

    print("Target reached")
    return [0, 0]  # stop if at goal