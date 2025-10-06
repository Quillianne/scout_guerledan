#!/usr/bin/env python3
"""
Test script for the new run_simulation function
"""

import sys
sys.path.append('/home/aurele/Documents/Rob3A/Projet3A/Sim')

from simulattion import run_simulation, simple_controller, aggressive_controller, smooth_controller

def test_run_simulation():
    """Test the run_simulation function"""
    
    print("Testing run_simulation function...")
    
    # Test 1: Simple triangle path
    triangle_waypoints = [(5, 0), (2.5, 4.33), (-2.5, 4.33), (0, 0)]
    
    print("\nTest 1: Triangle path with simple controller")
    try:
        results = run_simulation(
            controller_func=simple_controller,
            waypoints=triangle_waypoints,
            boat_initial_state={'x': 0, 'y': 0, 'theta': 0},
            simulation_params={'dt': 0.05, 'max_time': 20.0, 'waypoint_tolerance': 1.5}
        )
        print(f"✓ Test 1 passed! Completed: {results['completed']}, Waypoints reached: {results['waypoints_reached']}")
        
    except Exception as e:
        print(f"✗ Test 1 failed: {e}")
    
    # Test 2: Custom controller
    def custom_controller(boat, target_x, target_y, **kwargs):
        """Custom controller that moves slowly"""
        x, y, theta = boat.get_position()
        
        dx = target_x - x
        dy = target_y - y
        desired_theta = np.arctan2(dy, dx)
        
        angle_error = desired_theta - theta
        while angle_error > np.pi: angle_error -= 2*np.pi
        while angle_error < -np.pi: angle_error += 2*np.pi
        
        # Very gentle movement
        base_thrust = 3.0
        steering = 2.0 * angle_error
        
        return base_thrust - steering, base_thrust + steering
    
    print("\nTest 2: Custom controller")
    try:
        import numpy as np
        simple_waypoints = [(3, 0), (0, 3)]
        
        results = run_simulation(
            controller_func=custom_controller,
            waypoints=simple_waypoints,
            simulation_params={'dt': 0.1, 'max_time': 15.0, 'waypoint_tolerance': 1.0}
        )
        print(f"✓ Test 2 passed! Custom controller worked.")
        
    except Exception as e:
        print(f"✗ Test 2 failed: {e}")
    
    print("\nAll tests completed!")

if __name__ == "__main__":
    test_run_simulation()