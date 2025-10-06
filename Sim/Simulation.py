import math
import time
try:
    import numpy as np
    import matplotlib.pyplot as plt
except Exception:
    # If plotting libraries are not available the file will still be syntactically valid
    np = None
    plt = None

from Controller import controller  # reuse the controller from Controller.py


DT = 0.05  # simulation timestep (s)


def update_state(state, inputs):
    """
    Uses the previous state and the inputs to compute the next state of the boat

    state: [x, y, theta, vx, vy, omega]
    inputs: [thrust_left, thrust_right]
    """
    # simple planar rigid-body model with two thrusters located at y = +-d
    x, y, theta, vx, vy, omega = state
    thrust_l, thrust_r = inputs

    # physical parameters (reasonable defaults)
    m = 5.0           # mass (kg)
    I = 0.5           # rotational inertia (kg*m^2)
    d = 0.5           # half-distance between thrusters (m)
    linear_drag = 3.0  # linear damping coefficient
    angular_drag = 2.0  # angular damping

    # Total forward force in body frame and torque around COM
    F_forward = thrust_l + thrust_r
    torque = (thrust_r - thrust_l) * d

    # Convert world velocities to body frame
    c = math.cos(theta)
    s = math.sin(theta)
    # rotation matrix from body -> world is R = [[c, -s], [s, c]]
    # Therefore world -> body is R.T = [[c, s], [-s, c]]
    v_world = np.array([vx, vy])
    R_T = np.array([[c, s], [-s, c]])
    v_body = R_T.dot(v_world)

    # accelerations in body frame
    ax_b = (F_forward - linear_drag * v_body[0]) / m
    ay_b = (- linear_drag * v_body[1]) / m
    alpha = (torque - angular_drag * omega) / I

    # convert acceleration back to world frame
    R = np.array([[c, -s], [s, c]])
    a_world = R.dot(np.array([ax_b, ay_b]))

    # integrate (Euler)
    vx_new = vx + a_world[0] * DT
    vy_new = vy + a_world[1] * DT
    x_new = x + vx_new * DT
    y_new = y + vy_new * DT
    omega_new = omega + alpha * DT
    theta_new = theta + omega_new * DT

    return [x_new, y_new, theta_new, vx_new, vy_new, omega_new]



def display(state, goals):
    """
    Displays the current state of the boat and the goal points

    state: [x, y, theta, vx, vy, omega]
    goals: list of points [[x1, y1], [x2, y2], ...]
    """
    if plt is None or np is None:
        # plotting libraries not available; do nothing
        return

    x, y, theta, vx, vy, omega = state

    # persistent figure/axes
    if not hasattr(display, "fig"):
        display.fig, display.ax = plt.subplots(figsize=(6, 6))
        display.ax.set_aspect('equal')

    ax = display.ax
    ax.cla()

    # plot goals
    if goals:
        gx = [g[0] for g in goals]
        gy = [g[1] for g in goals]
        ax.plot(gx, gy, 'rx', markersize=10, label='goals')

    # draw boat as triangle in body frame
    boat_length = 0.8
    boat_width = 0.5
    # triangle points in body coords (nose, rear-left, rear-right)
    pts_body = np.array([
        [ boat_length/2, 0.0],
        [-boat_length/2, -boat_width/2],
        [-boat_length/2,  boat_width/2],
    ])
    c = math.cos(theta)
    s = math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    pts_world = (R.dot(pts_body.T)).T + np.array([x, y])
    poly = plt.Polygon(pts_world, color='C0', alpha=0.8)
    ax.add_patch(poly)

    # heading arrow
    ax.arrow(x, y, 0.7 * math.cos(theta), 0.7 * math.sin(theta),
             head_width=0.1, head_length=0.15, fc='k', ec='k')

    # draw path history if provided
    if hasattr(display, 'history'):
        display.history.append((x, y))
    else:
        display.history = [(x, y)]

    hx = [p[0] for p in display.history]
    hy = [p[1] for p in display.history]
    ax.plot(hx, hy, color='C0', linewidth=1, alpha=0.6)

    # viewport centered around current position
    margin = 5.0
    ax.set_xlim(x - margin, x + margin)
    ax.set_ylim(y - margin, y + margin)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Boat simulation')
    ax.grid(True)

    plt.pause(0.001)
    


def simulate():
    """
    Main simulation loop
    """
    # initial state [x, y, theta, vx, vy, omega]
    state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # define a sequence of goals
    goals = [[6.0, 2.0], [10.0, 0.0], [8.0, -4.0], [0.0, 0.0]]

    step = 0
    max_steps = 2000
    try:
        while step < max_steps and goals:
            current_goal = goals[0]
            inputs = controller(state, current_goal)
            state = update_state(state, inputs)

            # display
            display(state, goals)

            # check arrival
            dx = current_goal[0] - state[0]
            dy = current_goal[1] - state[1]
            if math.hypot(dx, dy) < 0.4:
                # reached the goal
                goals.pop(0)

            step += 1
            time.sleep(DT * 0.6)
    except KeyboardInterrupt:
        print('Simulation interrupted by user')

    print('Simulation finished at step', step, 'state=', state)


if __name__ == '__main__':
    simulate()


