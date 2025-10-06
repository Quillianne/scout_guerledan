"""
2D Boat Simulation Demo with Roblib
====================================

This demo shows a simple 2D boat with dual motors navigating to a target.
The boat uses differential thrust from left and right motors to steer.

Key Features:
- Physics-based dynamics with mass, inertia, and damping
- Dual motor control (left/right motors for differential steering) 
- PID controller for autonomous navigation
- Real-time visualization using roblib functions
- Motor thrust visualization
- Trajectory tracking

Controls:
- Left motor + Right motor = Forward thrust
- Left motor - Right motor = Turning moment (differential steering)
"""

import roblib as rl
import numpy as np
import matplotlib.pyplot as plt
from numpy import cos, sin, pi, array
import time

# Boat simulation class
class SimpleBoat2D:
    def __init__(self, x=0, y=0, theta=0):
        # State: [x, y, theta, vx, vy, omega]
        self.state = array([x, y, theta, 0, 0, 0])
        
        # Physical parameters
        self.mass = 8.0
        self.inertia = 1.5
        self.motor_separation = 1.2  # distance between motors
        
        # Damping
        self.drag = 0.4
        self.angular_drag = 0.3
        
    def update(self, left_thrust, right_thrust, dt):
        x, y, theta, vx, vy, omega = self.state
        
        # Total forward force and turning torque
        total_thrust = left_thrust + right_thrust
        torque = (right_thrust - left_thrust) * self.motor_separation
        
        # Convert body forces to world frame
        fx = total_thrust * cos(theta) - self.drag * vx
        fy = total_thrust * sin(theta) - self.drag * vy
        torque -= self.angular_drag * omega
        
        # Update velocities
        vx += (fx / self.mass) * dt
        vy += (fy / self.mass) * dt
        omega += (torque / self.inertia) * dt
        
        # Update positions
        x += vx * dt
        y += vy * dt
        theta += omega * dt
        
        # Normalize angle
        theta = ((theta + pi) % (2*pi)) - pi
        
        self.state = array([x, y, theta, vx, vy, omega])

def simple_controller(boat, target_x, target_y):
    """Simple proportional controller for boat navigation"""
    x, y, theta = boat.state[:3]
    
    # Calculate desired direction
    dx, dy = target_x - x, target_y - y
    desired_angle = np.arctan2(dy, dx)
    
    # Angle and distance errors
    angle_error = desired_angle - theta
    while angle_error > pi: angle_error -= 2*pi
    while angle_error < -pi: angle_error += 2*pi
    
    distance = np.sqrt(dx*dx + dy*dy)
    
    # Control gains
    kp_thrust = 1.5
    kp_steer = 6.0
    
    # Calculate motor commands
    base_thrust = min(kp_thrust * distance, 8.0)
    steer_correction = kp_steer * angle_error
    
    left_motor = base_thrust - steer_correction
    right_motor = base_thrust + steer_correction
    
    # Limit motor outputs
    max_thrust = 10.0
    left_motor = max(-max_thrust, min(max_thrust, left_motor))
    right_motor = max(-max_thrust, min(max_thrust, right_motor))
    
    return left_motor, right_motor

def draw_simple_boat(x, y, theta, size=1.0):
    """Draw boat using roblib functions"""
    
    # Boat shape (simple hull)
    length = 3.0 * size
    width = 1.5 * size
    
    boat_points = array([
        [-length/2, -width/4],
        [-length/2, width/4], 
        [length/4, width/2],
        [length/2, 0],
        [length/4, -width/2],
        [-length/2, -width/4]
    ]).T
    
    # Transform and draw
    hull = rl.add1(boat_points)
    transform = rl.tran2H(x, y) @ rl.rot2H(theta)
    rl.plot2D(transform @ hull, 'blue', 3)
    
    # Direction arrow
    rl.draw_arrow(x, y, theta, length*0.8, 'red', 2)
    
    # Motor positions
    motor_offset = 0.7 * size
    left_motor = transform @ array([[0], [-motor_offset], [1]])
    right_motor = transform @ array([[0], [motor_offset], [1]])
    
    plt.plot(left_motor[0], left_motor[1], 'ko', markersize=8)
    plt.plot(right_motor[0], right_motor[1], 'ko', markersize=8)

def run_demo():
    """Run the boat simulation demo"""
    print("2D Boat Simulation with Dual Motors")
    print("===================================")
    print("The boat will navigate from (-8,-6) to (8,6)")
    print("Watch how the differential thrust steers the boat!")
    print()
    
    # Setup
    boat = SimpleBoat2D(x=-8, y=-6, theta=-pi/6)
    target_x, target_y = 8, 6
    
    dt = 0.05
    max_time = 15.0
    t = 0
    
    # Trajectory storage
    path_x, path_y = [], []
    
    # Initialize plot
    plt.figure(figsize=(12, 8))
    plt.ion()
    
    while t < max_time:
        # Get motor commands
        left_thrust, right_thrust = simple_controller(boat, target_x, target_y)
        
        # Update boat
        boat.update(left_thrust, right_thrust, dt)
        
        # Get state
        x, y, theta, vx, vy, omega = boat.state
        path_x.append(x)
        path_y.append(y)
        
        # Redraw every few steps
        if len(path_x) % 3 == 0:
            plt.clf()
            
            # Draw trajectory
            if len(path_x) > 1:
                plt.plot(path_x, path_y, 'b--', alpha=0.7, linewidth=2, label='Path')
            
            # Draw boat
            draw_simple_boat(x, y, theta)
            
            # Draw target
            plt.plot(target_x, target_y, 'r*', markersize=15, label='Target')
            
            # Motor thrust visualization
            thrust_scale = 0.2
            motor_offset = 0.7
            
            # Left motor thrust
            lx = x - motor_offset * cos(theta + pi/2)
            ly = y - motor_offset * sin(theta + pi/2)
            plt.arrow(lx, ly, -thrust_scale * left_thrust * cos(theta),
                     -thrust_scale * left_thrust * sin(theta),
                     head_width=0.3, fc='green', ec='green', alpha=0.8)
            
            # Right motor thrust
            rx = x + motor_offset * cos(theta + pi/2) 
            ry = y + motor_offset * sin(theta + pi/2)
            plt.arrow(rx, ry, -thrust_scale * right_thrust * cos(theta),
                     -thrust_scale * right_thrust * sin(theta), 
                     head_width=0.3, fc='green', ec='green', alpha=0.8)
            
            # Plot settings
            plt.axis('equal')
            plt.grid(True, alpha=0.3)
            plt.xlim(-10, 10)
            plt.ylim(-8, 8)
            plt.xlabel('X Position (m)')
            plt.ylabel('Y Position (m)')
            plt.title(f'2D Boat with Dual Motors - Time: {t:.1f}s')
            plt.legend()
            
            # Status display
            speed = np.sqrt(vx*vx + vy*vy)
            distance = np.sqrt((target_x-x)**2 + (target_y-y)**2)
            
            status = f'Speed: {speed:.1f} m/s\nDistance: {distance:.1f} m\nLeft Motor: {left_thrust:.1f} N\nRight Motor: {right_thrust:.1f} N'
            plt.text(-9.5, 7, status, fontsize=10, 
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen"))
            
            plt.pause(0.05)
        
        # Check if reached target
        distance_to_target = np.sqrt((target_x-x)**2 + (target_y-y)**2)
        if distance_to_target < 1.0:
            print(f"Target reached at t={t:.1f}s!")
            break
            
        t += dt
    
    print("Demo completed!")
    print("\nKey observations:")
    print("- Both motors forward: boat moves straight")
    print("- Left motor > Right motor: boat turns right") 
    print("- Right motor > Left motor: boat turns left")
    print("- This is differential steering, like a tank!")
    
    # Keep plot open
    plt.ioff()
    input("\nPress Enter to close...")

if __name__ == "__main__":
    run_demo()