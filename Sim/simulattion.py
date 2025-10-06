import roblib as rl
import numpy as np
import matplotlib.pyplot as plt
from numpy import cos, sin, pi, array, zeros, dot
import time
import warnings
from controllers import (proportional_controller, simple_controller, pid_controller, 
                        aggressive_controller, smooth_controller, get_controller)

# Suppress specific matplotlib warnings
warnings.filterwarnings('ignore', message='.*fixed.*limits.*adjustable.*', category=UserWarning)

# Boat simulation with 2 motors in 2D
class Boat2D:
    def __init__(self, x=0, y=0, theta=0):
        """
        Initialize a 2D boat with two motors
        State: [x, y, theta, vx, vy, omega]
        - x, y: position in 2D
        - theta: orientation angle
        - vx, vy: linear velocities
        - omega: angular velocity
        """
        self.state = array([[x], [y], [theta], [0], [0], [0]])  # [x, y, theta, vx, vy, omega]
        
        # Physical parameters
        self.mass = 10.0        # boat mass (kg)
        self.inertia = 2.0      # rotational inertia (kg*m^2)
        self.length = 4.0       # boat length (m)
        self.width = 2.0        # boat width (m)
        
        # Damping coefficients
        self.linear_damping = 0.5
        self.angular_damping = 0.3
        
        # Motor positions relative to center (left and right motors)
        self.motor_distance = 1.0  # distance from center to each motor
        


    def dynamics(self, u_left, u_right, dt):
        """Update boat dynamics with motor forces"""
        x, y, theta, vx, vy, omega = self.state.flatten()
        
        # forces from motors
        F_total = u_left + u_right
        torque = (u_right - u_left) * self.motor_distance
        
        # Convert body forces to world frame
        c_theta, s_theta = cos(theta), sin(theta)
        F_world_x = F_total * c_theta - self.linear_damping * vx
        F_world_y = F_total * s_theta - self.linear_damping * vy
        torque -= self.angular_damping * omega
        
        # Accelerations
        ax, ay, alpha = F_world_x / self.mass, F_world_y / self.mass, torque / self.inertia

        # Update velocities and positions (Euler integration)
        vx += ax * dt
        vy += ay * dt
        omega += alpha * dt 
        x += vx * dt
        y += vy * dt
        theta = ((theta + omega * dt + pi) % (2 * pi)) - pi
        
        self.state = array([[x], [y], [theta], [vx], [vy], [omega]])
    


    def get_position(self):
        """Return current position and orientation"""
        return self.state[0,0], self.state[1,0], self.state[2,0]
    


    def get_velocity(self):
        """Return current velocities"""
        return self.state[3,0], self.state[4,0], self.state[5,0]



def draw_boat_2d(x, y, theta, color='blue', size=1.0):
    """
    Draw a simple 2D boat using roblib functions
    """
    # Define boat shape in body coordinates
    boat_length = 4.0 * size
    boat_width = 2.0 * size
    
    # Boat hull points (simple boat shape)
    hull_points = array([
        [-boat_length/2, -boat_width/4],   # back left
        [-boat_length/2, boat_width/4],    # back right
        [boat_length/3, boat_width/2],     # middle right
        [boat_length/2, 0],                # front tip
        [boat_length/3, -boat_width/2],    # middle left
        [-boat_length/2, -boat_width/4]    # back to start
    ]).T
    
    # Add homogeneous coordinate
    hull = rl.add1(hull_points)
    
    # Transform to world coordinates
    boat_transform = rl.tran2H(x, y) @ rl.rot2H(theta)
    transformed_hull = boat_transform @ hull
    
    # Draw the boat
    rl.plot2D(transformed_hull, color, 2)
    
    # Draw direction arrow
    rl.draw_arrow(x, y, theta, boat_length * 0.6, 'red', 2)
    
    # Draw motor positions
    motor_y_offset = 0.8 * size
    left_motor_pos = boat_transform @ array([[0], [-motor_y_offset], [1]])
    right_motor_pos = boat_transform @ array([[0], [motor_y_offset], [1]])
    
    plt.plot(left_motor_pos[0], left_motor_pos[1], 'ko', markersize=8)
    plt.plot(right_motor_pos[0], right_motor_pos[1], 'ko', markersize=8)



def run_simulation(controller_func, waypoints, boat_initial_state=None, 
                   simulation_params=None, visualization_params=None):
    """
    Generic simulation function that can run with any controller and waypoints
    
    Parameters:
    -----------
    controller_func : function
        Controller function that takes (boat, target_x, target_y, **kwargs) 
        and returns (u_left, u_right)
    waypoints : list of tuples
        List of (x, y) target positions. E.g., [(0, 0), (5, 5), (-3, 2)]
    boat_initial_state : dict, optional
        Initial boat state {'x': 0, 'y': 0, 'theta': 0}
        Default: {'x': 0, 'y': 0, 'theta': 0}
    simulation_params : dict, optional
        Simulation parameters {'dt': 0.05, 'max_time': 30.0, 'waypoint_tolerance': 2.0}
        Default: {'dt': 0.05, 'max_time': 30.0, 'waypoint_tolerance': 2.0}
    visualization_params : dict, optional
        Visualization parameters {'xlim': (-15, 15), 'ylim': (-15, 15), 'update_freq': 1}
        Default: {'xlim': (-15, 15), 'ylim': (-15, 15), 'update_freq': 1}
        
    Returns:
    --------
    dict : simulation results
        Contains 'trajectory', 'times', 'completed', 'final_state'
    """
    
    # Set default parameters
    if boat_initial_state is None:
        boat_initial_state = {'x': 0, 'y': 0, 'theta': 0}
    
    if simulation_params is None:
        simulation_params = {'dt': 0.05, 'max_time': 30.0, 'waypoint_tolerance': 2.0}
    
    if visualization_params is None:
        visualization_params = {'xlim': (-15, 15), 'ylim': (-15, 15), 'update_freq': 1}
    
    # Extract parameters
    dt = simulation_params.get('dt', 0.05)
    max_time = simulation_params.get('max_time', 30.0)
    waypoint_tolerance = simulation_params.get('waypoint_tolerance', 2.0)
    
    xlim = visualization_params.get('xlim', (-15, 15))
    ylim = visualization_params.get('ylim', (-15, 15))
    update_freq = visualization_params.get('update_freq', 1)
    
    # Initialize boat
    boat = Boat2D(x=boat_initial_state['x'], 
                  y=boat_initial_state['y'], 
                  theta=boat_initial_state['theta'])
    
    # Simulation state
    t = 0
    current_waypoint_idx = 0
    target_x, target_y = waypoints[current_waypoint_idx]
    
    # Data storage
    trajectory_x, trajectory_y = [], []
    times = []
    motor_commands = []
    
    # Initialize plot
    plt.figure(figsize=(12, 10))
    plt.ion()
    
    print(f"Starting simulation with {len(waypoints)} waypoints")
    print(f"Initial position: ({boat_initial_state['x']}, {boat_initial_state['y']})")
    print(f"Waypoints: {waypoints}")
    
    # Main simulation loop
    frame_count = 0
    while t < max_time and current_waypoint_idx < len(waypoints):
        
        # Get motor commands from controller
        try:
            # Try calling controller with additional parameters
            u_left, u_right = controller_func(boat, target_x, target_y, 
                                            dt=dt, waypoint_idx=current_waypoint_idx)
        except TypeError:
            # Fallback to simple controller signature
            u_left, u_right = controller_func(boat, target_x, target_y)
        
        # Update boat dynamics
        boat.dynamics(u_left, u_right, dt)
        
        # Get current state
        x, y, theta = boat.get_position()
        vx, vy, omega = boat.get_velocity()
        
        # Store data
        trajectory_x.append(x)
        trajectory_y.append(y)
        times.append(t)
        motor_commands.append((u_left, u_right))
        
        # Check if current waypoint is reached
        distance_to_current = np.sqrt((target_x - x)**2 + (target_y - y)**2)
        
        if distance_to_current < waypoint_tolerance:
            print(f"Waypoint {current_waypoint_idx} reached at t={t:.1f}s: ({target_x}, {target_y})")
            current_waypoint_idx += 1
            
            if current_waypoint_idx < len(waypoints):
                target_x, target_y = waypoints[current_waypoint_idx]
                print(f"Next target: ({target_x}, {target_y})")
        
        # Update visualization
        if frame_count % update_freq == 0:
            plt.clf()
            
            # Draw trajectory
            if len(trajectory_x) > 1:
                plt.plot(trajectory_x, trajectory_y, 'b--', alpha=0.6, linewidth=1, label='Trajectory')
            
            # Draw all waypoints
            for i, (wx, wy) in enumerate(waypoints):
                if i < current_waypoint_idx:
                    plt.plot(wx, wy, 'go', markersize=10, alpha=0.6, label='Completed' if i == 0 else '')
                elif i == current_waypoint_idx:
                    plt.plot(wx, wy, 'r*', markersize=15, label='Current Target')
                else:
                    plt.plot(wx, wy, 'yo', markersize=8, alpha=0.6, label='Future' if i == current_waypoint_idx + 1 else '')
            
            # Draw waypoint connections
            if len(waypoints) > 1:
                wp_x, wp_y = zip(*waypoints)
                plt.plot(wp_x, wp_y, 'k--', alpha=0.3, linewidth=1)
            
            # Draw boat
            draw_boat_2d(x, y, theta, 'blue', 1.0)
            
            # Draw motor thrust indicators
            motor_scale = 0.5
            left_thrust_line = [x - 1.0*cos(theta + pi/2), x - 1.0*cos(theta + pi/2) - motor_scale*u_left*cos(theta)]
            left_thrust_line_y = [y - 1.0*sin(theta + pi/2), y - 1.0*sin(theta + pi/2) - motor_scale*u_left*sin(theta)]
            right_thrust_line = [x + 1.0*cos(theta + pi/2), x + 1.0*cos(theta + pi/2) - motor_scale*u_right*cos(theta)]
            right_thrust_line_y = [y + 1.0*sin(theta + pi/2), y + 1.0*sin(theta + pi/2) - motor_scale*u_right*sin(theta)]
            
            plt.plot(left_thrust_line, left_thrust_line_y, 'g-', linewidth=3, alpha=0.7)
            plt.plot(right_thrust_line, right_thrust_line_y, 'g-', linewidth=3, alpha=0.7)
            
            # Set plot properties
            plt.xlim(xlim)
            plt.ylim(ylim)
            plt.gca().set_aspect('equal', adjustable='box')
            plt.grid(True, alpha=0.3)
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            
            # Dynamic title
            speed = np.sqrt(vx**2 + vy**2)
            plt.title(f'Boat Simulation - Waypoint {current_waypoint_idx+1}/{len(waypoints)}\n'
                     f'Time: {t:.1f}s, Speed: {speed:.1f} m/s')
            
            # Remove duplicate labels
            handles, labels = plt.gca().get_legend_handles_labels()
            by_label = dict(zip(labels, handles))
            plt.legend(by_label.values(), by_label.keys())
            
            # Status text
            distance_to_current = np.sqrt((target_x - x)**2 + (target_y - y)**2)
            status_text = (f'Motor Commands:\n'
                          f'Left: {u_left:.1f} N\n'
                          f'Right: {u_right:.1f} N\n'
                          f'Distance to target: {distance_to_current:.1f} m\n'
                          f'Waypoint: {current_waypoint_idx+1}/{len(waypoints)}')
            
            plt.text(xlim[0] + 0.5, ylim[1] - 1, status_text, 
                    fontsize=9, bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue"))
            
            plt.pause(0.02)
        
        frame_count += 1
        t += dt
    
    # Check completion status
    completed = current_waypoint_idx >= len(waypoints)
    
    if completed:
        print(f"All waypoints completed at t={t:.1f}s!")
    else:
        print(f"Simulation ended at t={t:.1f}s. Reached {current_waypoint_idx}/{len(waypoints)} waypoints.")
    
    plt.ioff()
    
    # Return results
    results = {
        'trajectory': list(zip(trajectory_x, trajectory_y)),
        'times': times,
        'motor_commands': motor_commands,
        'completed': completed,
        'final_state': {
            'x': x, 'y': y, 'theta': theta,
            'vx': vx, 'vy': vy, 'omega': omega
        },
        'waypoints_reached': current_waypoint_idx
    }
    
    return results







# Note: Controllers are now imported from controllers.py

# ============================================================================
# EXAMPLE USAGE
# ============================================================================

def example_usage():
    """
    Example of how to use the run_simulation function with different controllers
    """
    
    print("\n" + "="*60)
    print("EXAMPLE: Using run_simulation() with different controllers")
    print("="*60)
    
    # Example 1: Simple square path
    square_waypoints = [(5, 0), (5, 5), (0, 5), (0, 0)]
    
    print("\nExample 1: Square path with simple controller")
    results1 = run_simulation(
        controller_func=simple_controller,
        waypoints=square_waypoints,
        boat_initial_state={'x': 0, 'y': 0, 'theta': 0},
        simulation_params={'dt': 0.05, 'max_time': 25.0, 'waypoint_tolerance': 1.5}
    )
    
    # Example 2: Complex path with aggressive controller
    complex_waypoints = [(10, 0), (10, 10), (-5, 10), (-5, -5), (0, 0)]
    
    print("\nExample 2: Complex path with aggressive controller")
    results2 = run_simulation(
        controller_func=aggressive_controller,
        waypoints=complex_waypoints,
        boat_initial_state={'x': 0, 'y': 0, 'theta': pi/4},
        simulation_params={'dt': 0.05, 'max_time': 30.0, 'waypoint_tolerance': 2.0},
        visualization_params={'xlim': (-8, 12), 'ylim': (-8, 12), 'update_freq': 2}
    )
    
    # Example 3: Gentle movement with pid controller
    gentle_waypoints = [(3, 3), (-3, 3), (-3, -3), (3, -3)]
    
    print("\nExample 3: Gentle path with pid controller")
    results3 = run_simulation(
        controller_func=pid_controller,
        waypoints=gentle_waypoints,
        boat_initial_state={'x': 0, 'y': 0, 'theta': 0},
        simulation_params={'dt': 0.05, 'max_time': 40.0, 'waypoint_tolerance': 1.0}
    )
    
    print(f"\nResults summary:")
    print(f"Example 1 - Completed: {results1['completed']}, Waypoints: {results1['waypoints_reached']}/{len(square_waypoints)}")
    print(f"Example 2 - Completed: {results2['completed']}, Waypoints: {results2['waypoints_reached']}/{len(complex_waypoints)}")
    print(f"Example 3 - Completed: {results3['completed']}, Waypoints: {results3['waypoints_reached']}/{len(gentle_waypoints)}")

# Uncomment the line below to run examples
# example_usage()

if __name__ == "__main__":
    # You can uncomment any of these to test different scenarios
    
    # Or run examples
    example_usage()
