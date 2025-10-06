import roblib as rl
import numpy as np
import matplotlib.pyplot as plt
from numpy import cos, sin, pi, array, zeros, dot
import time

# Enhanced Boat simulation with 2 motors in 2D
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
        
        # Motor parameters
        self.motor_distance = 1.0  # distance from center to each motor
        self.max_thrust = 15.0     # maximum thrust per motor
        
        # Environmental disturbances
        self.current_x = 0.0       # water current in x direction
        self.current_y = 0.0       # water current in y direction
        
    def set_current(self, current_x, current_y):
        """Set water current"""
        self.current_x = current_x
        self.current_y = current_y
        
    def dynamics(self, u_left, u_right, dt):
        """
        Update boat dynamics with motor forces
        u_left, u_right: thrust forces from left and right motors
        """
        x, y, theta, vx, vy, omega = self.state.flatten()
        
        # Limit motor outputs
        u_left = max(-self.max_thrust, min(self.max_thrust, u_left))
        u_right = max(-self.max_thrust, min(self.max_thrust, u_right))
        
        # Total thrust force (forward)
        F_total = u_left + u_right
        
        # Differential thrust creates torque
        torque = (u_right - u_left) * self.motor_distance
        
        # Forces in body frame
        F_body_x = F_total
        F_body_y = 0
        
        # Transform to world frame
        c_theta = cos(theta)
        s_theta = sin(theta)
        
        F_world_x = F_body_x * c_theta - F_body_y * s_theta
        F_world_y = F_body_x * s_theta + F_body_y * c_theta
        
        # Add environmental forces (current)
        F_world_x += self.current_x * self.mass
        F_world_y += self.current_y * self.mass
        
        # Apply damping
        F_world_x -= self.linear_damping * vx
        F_world_y -= self.linear_damping * vy
        torque -= self.angular_damping * omega
        
        # Accelerations
        ax = F_world_x / self.mass
        ay = F_world_y / self.mass
        alpha = torque / self.inertia
        
        # Update velocities (Euler integration)
        vx += ax * dt
        vy += ay * dt
        omega += alpha * dt
        
        # Update positions
        x += vx * dt
        y += vy * dt
        theta += omega * dt
        
        # Normalize angle
        theta = ((theta + pi) % (2 * pi)) - pi
        
        self.state = array([[x], [y], [theta], [vx], [vy], [omega]])
    
    def get_position(self):
        """Return current position and orientation"""
        return self.state[0,0], self.state[1,0], self.state[2,0]
    
    def get_velocity(self):
        """Return current velocities"""
        return self.state[3,0], self.state[4,0], self.state[5,0]

def draw_boat_2d(x, y, theta, color='blue', size=1.0):
    """
    Draw a detailed 2D boat using roblib functions
    """
    # Define boat shape in body coordinates
    boat_length = 4.0 * size
    boat_width = 2.0 * size
    
    # Boat hull points (more detailed boat shape)
    hull_points = array([
        [-boat_length/2, -boat_width/6],   # back left
        [-boat_length/2, boat_width/6],    # back right
        [-boat_length/3, boat_width/3],    # stern right
        [boat_length/4, boat_width/2],     # middle right
        [boat_length/2, 0],                # bow (front tip)
        [boat_length/4, -boat_width/2],    # middle left
        [-boat_length/3, -boat_width/3],   # stern left
        [-boat_length/2, -boat_width/6]    # back to start
    ]).T
    
    # Add homogeneous coordinate
    hull = rl.add1(hull_points)
    
    # Transform to world coordinates
    boat_transform = rl.tran2H(x, y) @ rl.rot2H(theta)
    transformed_hull = boat_transform @ hull
    
    # Draw the boat hull
    rl.plot2D(transformed_hull, color, 3)
    
    # Draw cabin/superstructure
    cabin_points = array([
        [-boat_length/6, -boat_width/8],   
        [-boat_length/6, boat_width/8],    
        [boat_length/6, boat_width/8],     
        [boat_length/6, -boat_width/8],    
        [-boat_length/6, -boat_width/8]    
    ]).T
    cabin = rl.add1(cabin_points)
    transformed_cabin = boat_transform @ cabin
    rl.plot2D(transformed_cabin, 'darkblue', 2)
    
    # Draw direction arrow
    rl.draw_arrow(x, y, theta, boat_length * 0.7, 'red', 3)
    
    # Draw motor positions
    motor_y_offset = 0.8 * size
    left_motor_pos = boat_transform @ array([[0], [-motor_y_offset], [1]])
    right_motor_pos = boat_transform @ array([[0], [motor_y_offset], [1]])
    
    plt.plot(left_motor_pos[0], left_motor_pos[1], 'ko', markersize=10, label='Left Motor')
    plt.plot(right_motor_pos[0], right_motor_pos[1], 'ko', markersize=10, label='Right Motor')

class PIDController:
    """PID Controller for boat navigation"""
    def __init__(self, kp_pos=2.0, ki_pos=0.1, kd_pos=0.5, 
                 kp_angle=8.0, ki_angle=0.2, kd_angle=1.0):
        self.kp_pos = kp_pos
        self.ki_pos = ki_pos
        self.kd_pos = kd_pos
        self.kp_angle = kp_angle
        self.ki_angle = ki_angle
        self.kd_angle = kd_angle
        
        # Error tracking
        self.prev_distance_error = 0
        self.prev_angle_error = 0
        self.integral_distance = 0
        self.integral_angle = 0
        
    def control(self, boat, target_x, target_y, dt):
        """
        PID controller to drive boat toward target
        Returns: (u_left, u_right) motor commands
        """
        x, y, theta = boat.get_position()
        
        # Calculate desired heading
        dx = target_x - x
        dy = target_y - y
        desired_theta = np.arctan2(dy, dx)
        
        # Distance error
        distance = np.sqrt(dx**2 + dy**2)
        distance_error = distance
        
        # Angle error
        angle_error = desired_theta - theta
        # Normalize angle error to [-pi, pi]
        while angle_error > pi:
            angle_error -= 2*pi
        while angle_error < -pi:
            angle_error += 2*pi
        
        # PID for distance
        self.integral_distance += distance_error * dt
        derivative_distance = (distance_error - self.prev_distance_error) / dt
        
        # PID for angle
        self.integral_angle += angle_error * dt
        derivative_angle = (angle_error - self.prev_angle_error) / dt
        
        # Control outputs
        thrust_output = (self.kp_pos * distance_error + 
                        self.ki_pos * self.integral_distance + 
                        self.kd_pos * derivative_distance)
        
        steering_output = (self.kp_angle * angle_error + 
                          self.ki_angle * self.integral_angle + 
                          self.kd_angle * derivative_angle)
        
        # Limit outputs
        base_thrust = min(thrust_output, boat.max_thrust)
        steering_thrust = max(-boat.max_thrust/2, min(boat.max_thrust/2, steering_output))
        
        # Motor commands
        u_left = base_thrust - steering_thrust
        u_right = base_thrust + steering_thrust
        
        # Update previous errors
        self.prev_distance_error = distance_error
        self.prev_angle_error = angle_error
        
        return u_left, u_right

def run_simulation(scenario="basic"):
    """Run different simulation scenarios"""
    
    # Simulation parameters
    dt = 0.05  # time step
    simulation_time = 20.0  # total simulation time
    t = 0
    
    if scenario == "basic":
        # Basic scenario: simple navigation
        boat = Boat2D(x=-10, y=-10, theta=pi/4)
        target_x, target_y = 10, 10
        controller = PIDController()
        boat.set_current(0, 0)
        title = "Basic Navigation"
        
    elif scenario == "current":
        # Scenario with water current
        boat = Boat2D(x=-8, y=-8, theta=0)
        target_x, target_y = 8, 8
        controller = PIDController(kp_angle=12.0)  # Higher gain for current
        boat.set_current(-1.0, 0.5)  # Strong current
        title = "Navigation with Water Current"
        
    elif scenario == "waypoints":
        # Multiple waypoint scenario
        boat = Boat2D(x=0, y=0, theta=0)
        waypoints = [(5, 0), (5, 5), (-5, 5), (-5, -5), (0, 0)]
        current_waypoint = 0
        target_x, target_y = waypoints[current_waypoint]
        controller = PIDController()
        boat.set_current(0, 0)
        title = "Waypoint Following"
    
    # Initialize plot
    plt.figure(figsize=(14, 10))
    plt.ion()  # Interactive mode
    
    # Storage for trajectory
    trajectory_x = []
    trajectory_y = []
    
    print(f"Starting {title} simulation...")
    x, y, theta = boat.get_position()
    print(f"Boat starts at ({x:.1f}, {y:.1f}) with heading {theta:.2f} rad")
    print(f"Target: ({target_x}, {target_y})")
    
    # Main simulation loop
    while t < simulation_time:
        
        # Handle waypoint switching for waypoint scenario
        if scenario == "waypoints":
            distance_to_target = np.sqrt((target_x - boat.state[0,0])**2 + (target_y - boat.state[1,0])**2)
            if distance_to_target < 1.5 and current_waypoint < len(waypoints) - 1:
                current_waypoint += 1
                target_x, target_y = waypoints[current_waypoint]
                print(f"Switching to waypoint {current_waypoint}: ({target_x}, {target_y})")
                # Reset controller integrals
                controller.integral_distance = 0
                controller.integral_angle = 0
        
        # Get motor commands from controller
        u_left, u_right = controller.control(boat, target_x, target_y, dt)
        
        # Update boat dynamics
        boat.dynamics(u_left, u_right, dt)
        
        # Get current state
        x, y, theta = boat.get_position()
        vx, vy, omega = boat.get_velocity()
        
        # Store trajectory
        trajectory_x.append(x)
        trajectory_y.append(y)
        
        # Clear and redraw every few steps to improve performance
        if int(t/dt) % 2 == 0:  # Update display every other step
            plt.clf()
            
            # Draw trajectory
            if len(trajectory_x) > 1:
                plt.plot(trajectory_x, trajectory_y, 'b--', alpha=0.6, linewidth=2, label='Trajectory')
            
            # Draw boat
            draw_boat_2d(x, y, theta, 'blue', 1.0)
            
            # Draw target(s)
            if scenario == "waypoints":
                # Draw all waypoints
                for i, (wx, wy) in enumerate(waypoints):
                    if i == current_waypoint:
                        plt.plot(wx, wy, 'r*', markersize=20, label='Current Target')
                    else:
                        plt.plot(wx, wy, 'yo', markersize=12, alpha=0.6)
                # Draw waypoint path
                wp_x, wp_y = zip(*waypoints)
                plt.plot(wp_x, wp_y, 'y--', alpha=0.5, linewidth=1)
            else:
                plt.plot(target_x, target_y, 'r*', markersize=20, label='Target')
            
            # Draw current vectors if present
            if boat.current_x != 0 or boat.current_y != 0:
                # Draw current arrows at several points
                for i in range(-10, 11, 5):
                    for j in range(-10, 11, 5):
                        plt.arrow(i, j, boat.current_x*2, boat.current_y*2, 
                                head_width=0.3, head_length=0.2, fc='cyan', ec='cyan', alpha=0.6)
            
            # Draw motor thrust indicators
            motor_scale = 0.3
            left_motor_x = x - 1.0*cos(theta + pi/2)
            left_motor_y = y - 1.0*sin(theta + pi/2)
            right_motor_x = x + 1.0*cos(theta + pi/2)
            right_motor_y = y + 1.0*sin(theta + pi/2)
            
            # Left motor thrust
            plt.arrow(left_motor_x, left_motor_y, 
                     -motor_scale*u_left*cos(theta), -motor_scale*u_left*sin(theta),
                     head_width=0.2, head_length=0.15, fc='green', ec='green', alpha=0.8)
            
            # Right motor thrust  
            plt.arrow(right_motor_x, right_motor_y,
                     -motor_scale*u_right*cos(theta), -motor_scale*u_right*sin(theta),
                     head_width=0.2, head_length=0.15, fc='green', ec='green', alpha=0.8)
            
            # Set plot properties
            plt.axis('equal')
            plt.grid(True, alpha=0.3)
            plt.xlim(-12, 12)
            plt.ylim(-12, 12)
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.title(f'{title}\nTime: {t:.1f}s, Speed: {np.sqrt(vx**2 + vy**2):.1f} m/s')
            plt.legend(loc='upper right')
            
            # Add status text
            distance_to_target = np.sqrt((target_x - x)**2 + (target_y - y)**2)
            status_text = f'Motor Commands:\nLeft: {u_left:.1f} N\nRight: {u_right:.1f} N\nDistance: {distance_to_target:.1f} m'
            if boat.current_x != 0 or boat.current_y != 0:
                status_text += f'\nCurrent: ({boat.current_x:.1f}, {boat.current_y:.1f}) m/s'
            
            plt.text(-11.5, 10.5, status_text, 
                     fontsize=10, bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue"))
            
            plt.pause(0.05)
        
        # Check if reached final target
        final_target = waypoints[-1] if scenario == "waypoints" else (target_x, target_y)
        distance_to_final = np.sqrt((final_target[0] - x)**2 + (final_target[1] - y)**2)
        
        if distance_to_final < 1.5:
            print(f"Final target reached at t={t:.1f}s!")
            time.sleep(2)  # Show final position
            break
        
        t += dt
    
    print("Simulation completed!")
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    print("2D Boat Simulation with Dual Motors")
    print("=====================================")
    print("Available scenarios:")
    print("1. Basic - Simple navigation to target")
    print("2. Current - Navigation with water current")
    print("3. Waypoints - Follow multiple waypoints")
    print()
    
    choice = input("Enter scenario (1/2/3) or press Enter for basic: ").strip()
    
    if choice == "2":
        run_simulation("current")
    elif choice == "3":
        run_simulation("waypoints") 
    else:
        run_simulation("basic")