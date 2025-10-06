"""
Quick Usage Examples for run_simulation()
========================================

This file shows how to use the new run_simulation() function 
from simulattion.py with different controllers and waypoints.
"""

import sys
import numpy as np
sys.path.append('/home/aurele/Documents/Rob3A/Projet3A/Sim')

from simulattion import (
    run_simulation, 
    simple_controller, 
    aggressive_controller, 
    smooth_controller,
    proportional_controller
)

# Example 1: Basic Square Mission
print("Example 1: Square Mission")
print("=" * 30)

square_waypoints = [(4, 0), (4, 4), (0, 4), (0, 0)]

results = run_simulation(
    controller_func=simple_controller,
    waypoints=square_waypoints,
    boat_initial_state={'x': 0, 'y': 0, 'theta': 0},
    simulation_params={'dt': 0.05, 'max_time': 20.0, 'waypoint_tolerance': 1.0}
)

print(f"Mission completed: {results['completed']}")
print(f"Waypoints reached: {results['waypoints_reached']}/{len(square_waypoints)}")
print(f"Final position: ({results['final_state']['x']:.1f}, {results['final_state']['y']:.1f})")
print()

# Example 2: Custom PID-style controller
print("Example 2: Custom Controller")
print("=" * 30)

def my_controller(boat, target_x, target_y, **kwargs):
    """My custom controller with different behavior"""
    x, y, theta = boat.get_position()
    
    # Calculate errors
    dx = target_x - x
    dy = target_y - y
    desired_angle = np.arctan2(dy, dx)
    
    angle_error = desired_angle - theta
    while angle_error > np.pi: angle_error -= 2*np.pi
    while angle_error < -np.pi: angle_error += 2*np.pi
    
    distance = np.sqrt(dx*dx + dy*dy)
    
    # My custom gains
    forward_gain = 3.0
    steering_gain = 5.0
    
    # Speed based on distance
    base_speed = min(forward_gain * distance, 10.0)
    
    # Steering based on angle error
    turn_effort = steering_gain * angle_error
    
    # Motor commands
    left_motor = base_speed - turn_effort
    right_motor = base_speed + turn_effort
    
    # Limit outputs
    max_thrust = 12.0
    left_motor = max(-max_thrust, min(max_thrust, left_motor))
    right_motor = max(-max_thrust, min(max_thrust, right_motor))
    
    return left_motor, right_motor

# Test custom controller with triangle mission
triangle_waypoints = [(3, 0), (0, 3), (-3, 0), (0, 0)]

results2 = run_simulation(
    controller_func=my_controller,
    waypoints=triangle_waypoints,
    boat_initial_state={'x': 0, 'y': 0, 'theta': np.pi/4},
    simulation_params={'dt': 0.05, 'max_time': 25.0, 'waypoint_tolerance': 0.8}
)

print(f"Triangle mission completed: {results2['completed']}")
print(f"Total simulation time: {len(results2['times']) * 0.05:.1f} seconds")
print()

# Example 3: Comparing different controllers
print("Example 3: Controller Comparison")
print("=" * 30)

# Same mission, different controllers
waypoints = [(5, 5), (-5, 5), (-5, -5), (5, -5)]

controllers_to_test = [
    ("Simple", simple_controller),
    ("Aggressive", aggressive_controller), 
    ("Smooth", smooth_controller),
    ("Proportional", proportional_controller)
]

for name, controller in controllers_to_test:
    result = run_simulation(
        controller_func=controller,
        waypoints=waypoints,
        boat_initial_state={'x': 0, 'y': 0, 'theta': 0},
        simulation_params={'dt': 0.05, 'max_time': 30.0, 'waypoint_tolerance': 1.5}
    )
    
    print(f"{name:12} Controller: {result['waypoints_reached']:1d}/4 waypoints, "
          f"Completed: {'YES' if result['completed'] else 'NO'}")

print()
print("Usage Tips:")
print("- Use 'simple_controller' for basic navigation")
print("- Use 'aggressive_controller' for fast missions")  
print("- Use 'smooth_controller' for gentle movement")
print("- Create your own controller function for custom behavior")
print("- Adjust waypoint_tolerance for mission precision")
print("- Modify simulation_params for different scenarios")