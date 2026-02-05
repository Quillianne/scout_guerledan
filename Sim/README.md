This is the Simulation directory

[`Boat.py`](Boat.py) contains the class that models the BlueBoat.

[`Controller.py`](Controller.py) contains the controller that sends commands to the boat to steer it towards a target.

In the [`Path_planner.py`](Path_planner.py) file, there are several algorithms that compute the target points for the scouts:
    
    - compute_target_points() : uses the position and heading of the Mothership to place the scouts on the plane.
    
    - compute_target_points_2() : uses only the position of the mothership to place the scouts (the heading is computed using the last pos of the MS and the current pos). The difficulty is to keep the scouts at the front.

    - compute_target_points_3() : same as n°2 but with lower refresh rate of data.

[`Simulation.py`](Simulation.py) runs the simulation for a given controller (mode 1, 2 or 3) using the 3 files above, and handles the display part.