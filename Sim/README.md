This is the Simulation directory

In the Path_planner file, there are several algorithms to tell the scouts where to go:
    
    - compute_target_points() : uses the position and heading of the Mothership to place the scouts on the plane.
    
    - compute_target_points_2() : uses only the position of the mothership to place the scouts. The difficulty is to keep tdhe scouts at the front.

    - compute_target_points_3() : same as n°2 but with lower refresh rate of data.

    - compute_target_points_4() : #WIP uses only the distances to place the scouts. needs intervals.