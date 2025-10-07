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
from Boat import Boat
from Path_planner import compute_target_points, compute_target_points_2, compute_target_points_3


DT = 0.05  # simulation timestep (s)
MAX_SPEED = 10.0  # m/s maximum allowed speed for the boat





def display(boats, goals, mothership=None):
    """
    Displays the current state of boats, mothership, and goal points

    boats: list of Boat instances (scouts)
    goals: list of points or per-boat goal lists
    mothership: Boat instance representing the mothership (optional)
    """
    if plt is None or np is None:
        # plotting libraries not available; do nothing
        return

    # boats: list of Boat instances
    if not isinstance(boats, (list, tuple)):
        boats = [boats]

    # use the mothership or first boat to set viewport
    if mothership:
        x, y, theta = mothership.get_position()
    else:
        x, y, theta = boats[0].get_position()

    # persistent figure/axes
    if not hasattr(display, "fig"):
        display.fig, display.ax = plt.subplots(figsize=(6, 6))
        display.ax.set_aspect('equal')

    ax = display.ax
    ax.cla()

    # simple color cycle (defined early because goals plotting may reference it)
    colors = ['C0', 'C1', 'C2', 'C3', 'C4']

    # plot goals: supports either a shared list or per-boat lists
    # if goals is a list of lists with same length as boats, treat as per-boat goals
    if goals:
        # detect per-boat goals structure
        per_boat = (
            isinstance(goals, (list, tuple))
            and len(goals) == len(boats)
            and all(isinstance(g, (list, tuple)) for g in goals)
        )
        if per_boat:
            for i, g_list in enumerate(goals):
                if g_list:
                    gx = [g[0] for g in g_list if g is not None]
                    gy = [g[1] for g in g_list if g is not None]
                    if gx and gy:
                        ax.plot(gx, gy, marker='x', color=colors[i % len(colors)], linestyle='None', markersize=12, markeredgewidth=2)
        else:
            gx = [g[0] for g in goals if g is not None]
            gy = [g[1] for g in goals if g is not None]
            if gx and gy:
                ax.plot(gx, gy, 'rx', markersize=10, label='goals')

    # draw mothership if provided
    if mothership:
        mx, my, mtheta = mothership.get_position()
        boat_length = 1.2  # larger for mothership
        boat_width = 0.7
        pts_body = np.array([
            [ boat_length/2, 0.0],
            [-boat_length/2, -boat_width/2],
            [-boat_length/2,  boat_width/2],
        ])
        c = math.cos(mtheta)
        s = math.sin(mtheta)
        R = np.array([[c, -s], [s, c]])
        pts_world = (R.dot(pts_body.T)).T + np.array([mx, my])
        poly = plt.Polygon(pts_world, color='red', alpha=0.9, label='Mothership')
        ax.add_patch(poly)
        
        # heading arrow for mothership
        ax.arrow(mx, my, 0.9 * math.cos(mtheta), 0.9 * math.sin(mtheta),
                 head_width=0.12, head_length=0.18, fc='darkred', ec='darkred')

    # draw all scout boats
    for i, boat in enumerate(boats):
        bx, by, btheta = boat.get_position()
        boat_length = 0.8
        boat_width = 0.5
        pts_body = np.array([
            [ boat_length/2, 0.0],
            [-boat_length/2, -boat_width/2],
            [-boat_length/2,  boat_width/2],
        ])
        c = math.cos(btheta)
        s = math.sin(btheta)
        R = np.array([[c, -s], [s, c]])
        pts_world = (R.dot(pts_body.T)).T + np.array([bx, by])
        poly = plt.Polygon(pts_world, color=colors[i % len(colors)], alpha=0.8)
        ax.add_patch(poly)

        # heading arrow
        ax.arrow(bx, by, 0.7 * math.cos(btheta), 0.7 * math.sin(btheta),
                 head_width=0.08, head_length=0.12, fc='k', ec='k')

        # draw path history per-boat
        if not hasattr(display, 'histories'):
            display.histories = [[] for _ in boats]

        # ensure histories list is long enough
        while len(display.histories) < len(boats):
            display.histories.append([])

        display.histories[i].append((bx, by))
        hx = [p[0] for p in display.histories[i]]
        hy = [p[1] for p in display.histories[i]]
        ax.plot(hx, hy, color=colors[i % len(colors)], linewidth=1, alpha=0.6)

    # viewport centered around mothership/first boat position (zoomed out)
    margin = 15.0
    ax.set_xlim(x - margin, x + margin)
    ax.set_ylim(y - margin, y + margin)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Boat simulation')
    ax.grid(True)

    plt.pause(0.001)
    


def simulate(mode:int=1):
    """
    Main simulation loop with mothership and two scout boats in formation
    """
    # Create mothership (slower than scouts)
    mothership = Boat(x=0.0, y=0.0, theta=0.0, max_speed=5.0, max_thrust=10.0, angular_drag=5.0)
    last_ms_position = (0, 0)  # Initialize last mothership position
    
    # Create two scout boats (A and B) - starting behind mothership
    boat_A = Boat(x=4.0, y=2.0, theta=0.1)  # Scout A (left) - default max_speed=10.0
    boat_B = Boat(x=4.0, y=-2.0, theta=-0.1)   # Scout B (right) - default max_speed=10.0
    scouts = [boat_A, boat_B]

    scout_targets = [None, None]  # Initialize scout targets
    
    # Mothership waypoints (simple path)
    mothership_waypoints = [
        [15.0, 0.0],
        [30.0, 10.0],
        [40.0, 10.0],
        [40.0, 0.0],
        [30.0, -10.0],
        [20.0, -1.0],
        [10.0, 0.0]
    ]
    current_waypoint_idx = 0
    
    # Formation parameters (equilateral triangle)
    triangle_side_length = 6.0  # Length of each side of the triangle

    
    t0 = time.time()
    step = 0
    max_steps = 3000
    
    try:
        while step < max_steps and current_waypoint_idx < len(mothership_waypoints):
            # Get current mothership waypoint
            target_waypoint = mothership_waypoints[current_waypoint_idx]
            
            # Simple proportional controller for mothership (direct waypoint following)
            ms_state = mothership.get_state()
            ms_x, ms_y, ms_theta = mothership.get_position()
            
            # Calculate mothership control
            dx = target_waypoint[0] - ms_x
            dy = target_waypoint[1] - ms_y
            desired_theta = math.atan2(dy, dx)
            
            # Angle error
            angle_error = desired_theta - ms_theta
            while angle_error > math.pi: angle_error -= 2*math.pi
            while angle_error < -math.pi: angle_error += 2*math.pi
            
            distance = math.hypot(dx, dy)
            
            # Simple proportional control for mothership (reduced gains for slower movement)
            base_thrust = min(1.0 * distance, 6.0)
            steering = 5.0 * angle_error  # Low gain for slow, wide turns
            
            ms_thrust_l = base_thrust - steering
            ms_thrust_r = base_thrust + steering
            
            # Clamp mothership thrust
            ms_thrust_l = max(-5.0, min(5.0, ms_thrust_l))
            ms_thrust_r = max(-5.0, min(5.0, ms_thrust_r))
            
            # Update mothership
            mothership.step(ms_thrust_l, ms_thrust_r, DT)
            
            # Check if mothership reached waypoint
            if distance < 1.0:
                current_waypoint_idx += 1
                if current_waypoint_idx < len(mothership_waypoints):
                    print(f"Mothership reached waypoint {current_waypoint_idx}")
            
            # Compute formation target points for scouts
            if mode == 1:
                target_A, target_B = compute_target_points(
                mothership.get_state(),
                distance=triangle_side_length,
                )
            elif mode == 2:
                target_A, target_B, last_ms_position = compute_target_points_2(
                mothership.get_state(),
                last_ms_position=last_ms_position,
                distance=triangle_side_length,
                )
            elif mode == 3:
                t = time.time() - t0
                target_A, target_B, last_ms_position = compute_target_points_3(
                    mothership.get_state(),
                    last_ms_position=last_ms_position,
                    t=t,
                    distance=triangle_side_length,
                    )
            else:
                print("Not valid mode selected. Please enter a valid mode")
            
            if target_A is None or target_B is None:
                # No valid targets; keep previous target
                pass
            else:
                scout_targets = [target_A, target_B]
            
            # Update each scout boat using controller to reach formation targets
            for i, scout in enumerate(scouts):
                target = scout_targets[i]
                state = scout.get_state()
                thrust_l, thrust_r = controller(state, target)
                scout.step(thrust_l, thrust_r, DT)
            
            # Display mothership and scouts
            # Pass scout targets as per-boat goals: [[target_A], [target_B]]
            goals_for_display = [[target_A], [target_B]]
            display(scouts, goals_for_display, mothership=mothership)
            
            step += 1
            time.sleep(DT * 0.6)
            
    except KeyboardInterrupt:
        print('Simulation interrupted by user')
    
    print(f'Simulation finished at step {step}')
    print(f'Mothership reached {current_waypoint_idx}/{len(mothership_waypoints)} waypoints')


if __name__ == '__main__':
    # type of path planner (1, 2, or 3)
    mode = 3
    simulate(mode)

