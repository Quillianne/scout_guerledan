import time
import numpy as np
import matplotlib.pyplot as plt

from Boat import Boat
from Simulation import display


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

    plt.pause(0.001)



def run_simulation(simulation_time=10.0, time_step=0.1):
    """Run a simple simulation loop for a given duration and time step."""
    # Initialize boats
    mothership = Boat(x=0.0, y=0.0, theta=0.0)
    scout_A = Boat(x=8.0, y=5.0, theta=0.0)
    scout_B = Boat(x=8.0, y=-5.0, theta=0.0)

    current_time = 0.0
    while current_time < simulation_time:
        # Update boat states (placeholder for actual dynamics and control)
        # Display current positions of the boats
        display([scout_A, scout_B], goals=None, mothership=mothership)

        # Increment time
        current_time += time_step
        time.sleep(time_step)  # Simulate real-time progression




if __name__ == "__main__":
    run_simulation()