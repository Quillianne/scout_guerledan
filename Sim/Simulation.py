import math
import time
import os
try:
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.animation import PillowWriter
except Exception:
    # If plotting libraries are not available the file will still be syntactically valid
    np = None
    plt = None
    PillowWriter = None

from Controller import controller  # use the controllers from Controller.py
from Boat import Boat
from Path_planner import compute_target_points, compute_target_points_2, compute_target_points_3


DT = 0.05  # simulation timestep (s)
MAX_SPEED = 10.0  # m/s maximum allowed speed for the boat





def display(boats, goals, mothership=None, save_frame=False):
    """
    Displays the current state of boats, mothership, and goal points

    boats: list of Boat instances (scouts)
    goals: list of points or per-boat goal lists
    mothership: Boat instance representing the mothership (optional)
    save_frame: if True, save the current frame for gif creation
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
                    gx = []
                    gy = []
                    for g in g_list:
                        if g is None:
                            continue
                        # accept either tuple/list (x,y) or 2x1 numpy column array
                        if hasattr(g, 'shape') and np.asarray(g).size >= 2:
                            # handle column vector [[x],[y]] or 1-D arrays
                            arr = np.asarray(g).reshape(-1)
                            gx.append(float(arr[0]))
                            gy.append(float(arr[1]))
                        else:
                            gx.append(float(g[0]))
                            gy.append(float(g[1]))
                    if gx and gy:
                        ax.plot(gx, gy, marker='x', color=colors[i % len(colors)], linestyle='None', markersize=12, markeredgewidth=2)
        else:
            gx = []
            gy = []
            for g in goals:
                if g is None:
                    continue
                arr = np.asarray(g).reshape(-1)
                gx.append(float(arr[0]))
                gy.append(float(arr[1]))
            if gx and gy:
                ax.plot(gx, gy, 'rx', markersize=10, label='goals')

    # draw mothership if provided
    if mothership:
        # mothership position may be available as numpy column vector
        try:
            mpos = mothership.get_position_np()
            mx = float(mpos[0, 0]); my = float(mpos[1, 0]); mtheta = float(mothership.theta)
        except Exception:
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
        # accept numpy column position or backward-compatible tuple
        try:
            bpos = boat.get_position_np()
            bx = float(bpos[0, 0]); by = float(bpos[1, 0]); btheta = float(boat.theta)
        except Exception:
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

    # Save frame if requested (for gif creation)
    if save_frame:
        if not hasattr(display, 'frames'):
            display.frames = []
        # Capture current figure as image array
        display.fig.canvas.draw()
        frame = np.frombuffer(display.fig.canvas.tostring_argb(), dtype=np.uint8)
        frame = frame.reshape(display.fig.canvas.get_width_height()[::-1] + (3,))
        display.frames.append(frame)

    plt.pause(0.001)
    


def save_gif(filename='simulation.gif', fps=20):
    """
    Save the captured frames as a GIF file.
    
    filename: name of the output gif file
    fps: frames per second for the gif
    """
    if not hasattr(display, 'frames') or not display.frames:
        print("No frames to save. Make sure save_frame=True was used during simulation.")
        return
    
    if PillowWriter is None:
        print("PillowWriter not available. Cannot save gif.")
        return
    
    # Get the directory of the current file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filepath = os.path.join(script_dir, filename)
    
    print(f"Saving {len(display.frames)} frames to {filepath}...")
    
    # Create a figure for the animation
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111)
    
    # Display first frame to set up the plot
    im = ax.imshow(display.frames[0])
    ax.axis('off')
    
    # Create animation writer
    writer = PillowWriter(fps=fps)
    
    with writer.saving(fig, filepath, dpi=100):
        for frame in display.frames:
            im.set_data(frame)
            writer.grab_frame()
    
    plt.close(fig)
    print(f"GIF saved successfully to {filepath}")


def simulate(mode:int=1, save_as_gif=False, gif_filename='simulation.gif', gif_fps=20):
    """
    Main simulation loop with mothership and two scout boats in formation
    
    mode: path planner mode (1, 2, or 3)
    save_as_gif: if True, save the simulation as a gif file
    gif_filename: name of the output gif file
    gif_fps: frames per second for the gif
    """
    # Reset frames if saving gif
    if save_as_gif and hasattr(display, 'frames'):
        display.frames = []
    
    # Create mothership (slower than scouts)
    mothership = Boat(x=0.0, y=0.0, theta=0.0, max_speed=5.0, max_thrust=10.0, angular_drag=5.0)
    last_ms_position = np.array([[0.0], [0.0]])  # Initialize last mothership position as column vector
    
    # Create two scout boats (A and B) - starting behind mothership
    boat_A = Boat(x=4.0, y=2.0, theta=0.1)  # Scout A (left) - default max_speed=10.0
    boat_B = Boat(x=4.0, y=-2.0, theta=-0.1)   # Scout B (right) - default max_speed=10.0
    scouts = [boat_A, boat_B]

    scout_targets = [None, None]  # Initialize scout targets
    
    # Mothership waypoints (simple path)
    # waypoints as 2x1 column numpy arrays
    mothership_waypoints = [
        np.array([[15.0], [0.0]]),
        np.array([[30.0], [10.0]]),
        np.array([[40.0], [10.0]]),
        np.array([[40.0], [0.0]]),
        np.array([[30.0], [-10.0]]),
        np.array([[20.0], [-1.0]]),
        np.array([[10.0], [0.0]])
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
            # use numpy column vectors for positions
            ms_state_np = mothership.get_state_np()
            ms_pos = ms_state_np[0:2].reshape((2, 1))
            ms_x = float(ms_pos[0, 0]); ms_y = float(ms_pos[1, 0]); ms_theta = float(ms_state_np[2, 0])

            # Calculate mothership control using numpy waypoint
            dx = float(target_waypoint[0, 0]) - ms_x
            dy = float(target_waypoint[1, 0]) - ms_y
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
            
            # Compute formation target points for scouts (pass numpy state)
            if mode == 1:
                target_A, target_B = compute_target_points(
                    mothership.get_state_np(),
                    distance=triangle_side_length,
                )
            elif mode == 2:
                target_A, target_B, last_ms_position = compute_target_points_2(
                    mothership.get_state_np(),
                    last_ms_position=last_ms_position,
                    distance=triangle_side_length,
                )
            elif mode == 3:
                t = time.time() - t0
                target_A, target_B, last_ms_position = compute_target_points_3(
                    mothership.get_state_np(),
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
                # pass numpy column state to controller (it flattens internally)
                state_np = scout.get_state_np()
                thrusts = controller(state_np, target)  # controller returns 1-D array
                # represent thrusters as 2x1 column vector internally
                thrusts_col = np.asarray(thrusts).reshape((2, 1))
                # call Boat.step with scalar thrusts (backward-compatible)
                scout.step(float(thrusts_col[0, 0]), float(thrusts_col[1, 0]), DT)
            
            # Display mothership and scouts
            # Pass scout targets as per-boat goals: [[target_A], [target_B]]
            # keep goals as per-boat lists of column vectors (or None)
            goals_for_display = [[target_A], [target_B]]
            display(scouts, goals_for_display, mothership=mothership, save_frame=save_as_gif)
            
            step += 1
            time.sleep(DT * 0.6)
            
    except KeyboardInterrupt:
        print('Simulation interrupted by user')
    
    print(f'Simulation finished at step {step}')
    print(f'Mothership reached {current_waypoint_idx}/{len(mothership_waypoints)} waypoints')
    
    # Save gif if requested
    if save_as_gif:
        save_gif(gif_filename, gif_fps)


if __name__ == '__main__':
    # type of path planner (1, 2, or 3)
    mode = 3
    
    simulate(mode, save_as_gif=True, gif_filename='simulation'+str(mode)+ '.gif', gif_fps=20)

