"""
Controller Functions for 2D Boat Simulation
===========================================

This module contains various controller implementations for the Boat2D simulation.
All controllers follow the same interface:
    controller(boat, target_x, target_y, **kwargs) -> (u_left, u_right)

Controllers included:
- proportional_controller: Basic proportional control
- simple_controller: Enhanced proportional with better gains
- pid_controller: PID-style controller (simplified)
- aggressive_controller: High-gain fast response
- smooth_controller: Low-gain gentle movement
"""

import numpy as np
from numpy import pi, sqrt, arctan2


def proportional_controller(boat, target_x, target_y):
    """
    Basic proportional controller to drive boat toward target
    Returns: (u_left, u_right) motor commands
    """
    x, y, theta = boat.get_position()
    
    dx = target_x - x
    dy = target_y - y
    desired_theta = arctan2(dy, dx)
    
    angle_error = desired_theta - theta
    while angle_error > pi: angle_error -= 2*pi
    while angle_error < -pi: angle_error += 2*pi
    
    distance = sqrt(dx**2 + dy**2)
    
    kp_distance = 1.0
    kp_angle = 1.0
    
    base_thrust = min(kp_distance * distance, 10.0)
    steering_thrust = kp_angle * angle_error
    
    u_left = base_thrust - steering_thrust
    u_right = base_thrust + steering_thrust
    
    max_thrust = 15.0
    u_left = max(-max_thrust, min(max_thrust, u_left))
    u_right = max(-max_thrust, min(max_thrust, u_right))
    
    return u_left, u_right


def simple_controller(boat, target_x, target_y, **kwargs):
    """
    Enhanced proportional controller with better gains
    
    Parameters:
    -----------
    boat : Boat2D
        The boat object
    target_x, target_y : float
        Target position
    **kwargs : dict
        Additional parameters (dt, waypoint_idx, etc.)
        
    Returns:
    --------
    tuple : (u_left, u_right) motor commands
    """
    x, y, theta = boat.get_position()
    
    dx = target_x - x
    dy = target_y - y
    desired_theta = arctan2(dy, dx)
    
    angle_error = desired_theta - theta
    while angle_error > pi: angle_error -= 2*pi
    while angle_error < -pi: angle_error += 2*pi
    
    distance = sqrt(dx**2 + dy**2)
    
    kp_distance = 2.0
    kp_angle = 6.0
    
    base_thrust = min(kp_distance * distance, 12.0)
    steering_thrust = kp_angle * angle_error
    
    u_left = base_thrust - steering_thrust
    u_right = base_thrust + steering_thrust
    
    max_thrust = 15.0
    u_left = max(-max_thrust, min(max_thrust, u_left))
    u_right = max(-max_thrust, min(max_thrust, u_right))
    
    return u_left, u_right


def pid_controller(boat, target_x, target_y, **kwargs):
    """
    PID controller with memory for integral and derivative terms
    
    Note: This uses a simplified approach. For production use, you'd want
    to maintain state between calls properly.
    """
    x, y, theta = boat.get_position()
    dt = kwargs.get('dt', 0.05)
    
    dx = target_x - x
    dy = target_y - y
    desired_theta = arctan2(dy, dx)
    
    angle_error = desired_theta - theta
    while angle_error > pi: angle_error -= 2*pi
    while angle_error < -pi: angle_error += 2*pi
    
    distance = sqrt(dx**2 + dy**2)
    
    kp_dist, ki_dist, kd_dist = 2.5, 0.1, 0.3
    kp_angle, ki_angle, kd_angle = 8.0, 0.2, 0.5
    
    # Simplified PID (without proper state management)
    dist_output = kp_dist * distance
    angle_output = kp_angle * angle_error
    
    base_thrust = min(dist_output, 12.0)
    steering_thrust = angle_output
    
    u_left = base_thrust - steering_thrust
    u_right = base_thrust + steering_thrust
    
    max_thrust = 15.0
    u_left = max(-max_thrust, min(max_thrust, u_left))
    u_right = max(-max_thrust, min(max_thrust, u_right))
    
    return u_left, u_right


def aggressive_controller(boat, target_x, target_y, **kwargs):
    """
    Aggressive controller with higher gains for fast response
    """
    x, y, theta = boat.get_position()
    
    dx = target_x - x
    dy = target_y - y
    desired_theta = arctan2(dy, dx)
    
    angle_error = desired_theta - theta
    while angle_error > pi: angle_error -= 2*pi
    while angle_error < -pi: angle_error += 2*pi
    
    distance = sqrt(dx**2 + dy**2)
    
    kp_distance = 4.0
    kp_angle = 12.0
    
    base_thrust = min(kp_distance * distance, 15.0)
    steering_thrust = kp_angle * angle_error
    
    u_left = base_thrust - steering_thrust
    u_right = base_thrust + steering_thrust
    
    max_thrust = 18.0
    u_left = max(-max_thrust, min(max_thrust, u_left))
    u_right = max(-max_thrust, min(max_thrust, u_right))
    
    return u_left, u_right


def smooth_controller(boat, target_x, target_y, **kwargs):
    """
    Smooth controller with lower gains for gentle movement
    """
    x, y, theta = boat.get_position()
    
    dx = target_x - x
    dy = target_y - y
    desired_theta = arctan2(dy, dx)
    
    angle_error = desired_theta - theta
    while angle_error > pi: angle_error -= 2*pi
    while angle_error < -pi: angle_error += 2*pi
    
    distance = sqrt(dx**2 + dy**2)
    
    kp_distance = 1.0
    kp_angle = 3.0
    
    base_thrust = min(kp_distance * distance, 8.0)
    steering_thrust = kp_angle * angle_error
    
    u_left = base_thrust - steering_thrust
    u_right = base_thrust + steering_thrust
    
    max_thrust = 10.0
    u_left = max(-max_thrust, min(max_thrust, u_left))
    u_right = max(-max_thrust, min(max_thrust, u_right))
    
    return u_left, u_right


def normalize_angle(angle):
    """Utility function to normalize angle to [-pi, pi]"""
    while angle > pi: angle -= 2*pi
    while angle < -pi: angle += 2*pi
    return angle


# Example of a more advanced controller with state memory
class StatefulPIDController:
    """
    A stateful PID controller that maintains error history
    """
    def __init__(self, kp_dist=2.5, ki_dist=0.1, kd_dist=0.3,
                 kp_angle=8.0, ki_angle=0.2, kd_angle=0.5):
        self.kp_dist, self.ki_dist, self.kd_dist = kp_dist, ki_dist, kd_dist
        self.kp_angle, self.ki_angle, self.kd_angle = kp_angle, ki_angle, kd_angle
        
        # State memory
        self.prev_dist_error = 0.0
        self.prev_angle_error = 0.0
        self.integral_dist = 0.0
        self.integral_angle = 0.0
        
    def __call__(self, boat, target_x, target_y, **kwargs):
        """Make the class callable like a function"""
        dt = kwargs.get('dt', 0.05)
        x, y, theta = boat.get_position()
        
        dx = target_x - x
        dy = target_y - y
        desired_theta = arctan2(dy, dx)
        
        distance_error = sqrt(dx**2 + dy**2)
        angle_error = normalize_angle(desired_theta - theta)
        
        # Update integrals
        self.integral_dist += distance_error * dt
        self.integral_angle += angle_error * dt
        
        # Calculate derivatives
        derivative_dist = (distance_error - self.prev_dist_error) / dt
        derivative_angle = (angle_error - self.prev_angle_error) / dt
        
        # PID outputs
        dist_output = (self.kp_dist * distance_error + 
                      self.ki_dist * self.integral_dist + 
                      self.kd_dist * derivative_dist)
        
        angle_output = (self.kp_angle * angle_error + 
                       self.ki_angle * self.integral_angle + 
                       self.kd_angle * derivative_angle)
        
        # Motor commands
        base_thrust = min(dist_output, 12.0)
        steering_thrust = angle_output
        
        u_left = base_thrust - steering_thrust
        u_right = base_thrust + steering_thrust
        
        # Limit outputs
        max_thrust = 15.0
        u_left = max(-max_thrust, min(max_thrust, u_left))
        u_right = max(-max_thrust, min(max_thrust, u_right))
        
        # Update previous errors
        self.prev_dist_error = distance_error
        self.prev_angle_error = angle_error
        
        return u_left, u_right
    
    def reset(self):
        """Reset the controller state"""
        self.prev_dist_error = 0.0
        self.prev_angle_error = 0.0
        self.integral_dist = 0.0
        self.integral_angle = 0.0


# Dictionary of available controllers for easy access
CONTROLLERS = {
    'proportional': proportional_controller,
    'simple': simple_controller,
    'pid': pid_controller,
    'aggressive': aggressive_controller,
    'smooth': smooth_controller,
}

def get_controller(name):
    """Get a controller by name"""
    if name in CONTROLLERS:
        return CONTROLLERS[name]
    else:
        raise ValueError(f"Unknown controller: {name}. Available: {list(CONTROLLERS.keys())}")
