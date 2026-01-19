"""This module contains functions to compute the estimation of positions for boats, as intervals (or groups of intervals).
Each boat should have its own observer instance, to estimate its own state as well as other boats' states.


The positions are represented as numpy arrays of codac IntervalVectors (each IntervalVector representing a 2D box)."""

import codac as cd
import numpy as np
import matplotlib.pyplot as plt


class IntervalObserver:
    def __init__(self, 
                 initial_pos=np.array([cd.IntervalVector([0, 0])]), 
                 other_initial_pos_1=np.array([cd.IntervalVector([0, 0])]),
                 other_initial_pos_2=np.array([cd.IntervalVector([0, 0])])):
        
        # initialize the positions
        self.self_pos = initial_pos
        self.other_pos_1 = other_initial_pos_1
        self.other_pos_2 = other_initial_pos_2
        
        # all velocities initialized to zero
        self.self_vel = np.array([cd.IntervalVector([0, 0])])
        self.other_vel_1 = np.array([cd.IntervalVector([0, 0])])
        self.other_vel_2 = np.array([cd.IntervalVector([0, 0])])



    def predict(self, pos, control_input=None, inertial_measurements=None, dt=0.05):
        """Predict the next state based on control input and time step.
        
        Uses Euler integration to propagate position intervals forward in time
        based on current velocity intervals.
        
        Args:
            pos: current self position (2x1 numpy array)
            control_input: control inputs (not used currently)
            inertial_measurements: inertial sensor measurements (not used currently)
            dt: time step in seconds
        """
        print("Predicting next state...")

        # Update self position with small uncertainty
        self.self_pos = np.array([cd.IntervalVector([[pos[0,0]-0.1, pos[0,0]+0.1],
                                                     [pos[1,0]-0.1, pos[1,0]+0.1]])])
        
        # Predict other boats' positions using Euler integration: pos = pos + vel * dt
        # For other_pos_1 (scout A) - handle multiple boxes
        # Velocity is now [speed_interval, angle_interval]
        vel1_speed = self.other_vel_1[0][0]  # speed interval
        vel1_angle = self.other_vel_1[0][1]  # angle interval
        predicted1 = []
        
        for other1_box in self.other_pos_1:
            # Convert polar velocity to Cartesian: vx = speed*cos(angle), vy = speed*sin(angle)
            # Use interval arithmetic for conversion
            vx_min = vel1_speed.lb() * np.cos(vel1_angle.ub()) if vel1_angle.ub() < 0 else vel1_speed.lb() * np.cos(vel1_angle.lb())
            vx_max = vel1_speed.ub() * np.cos(vel1_angle.lb()) if vel1_angle.lb() > 0 else vel1_speed.ub() * np.cos(vel1_angle.ub())
            vy_min = vel1_speed.lb() * np.sin(vel1_angle.lb())
            vy_max = vel1_speed.ub() * np.sin(vel1_angle.ub())
            
            # Handle full interval properly
            cos_vals = [vel1_speed.lb() * np.cos(vel1_angle.lb()), 
                       vel1_speed.lb() * np.cos(vel1_angle.ub()),
                       vel1_speed.ub() * np.cos(vel1_angle.lb()),
                       vel1_speed.ub() * np.cos(vel1_angle.ub())]
            sin_vals = [vel1_speed.lb() * np.sin(vel1_angle.lb()),
                       vel1_speed.lb() * np.sin(vel1_angle.ub()),
                       vel1_speed.ub() * np.sin(vel1_angle.lb()),
                       vel1_speed.ub() * np.sin(vel1_angle.ub())]
            
            vx_interval = cd.Interval(min(cos_vals), max(cos_vals))
            vy_interval = cd.Interval(min(sin_vals), max(sin_vals))
            
            # New position = old position + velocity * dt
            new_x1 = other1_box[0] + vx_interval * dt
            new_y1 = other1_box[1] + vy_interval * dt
            
            predicted1.append(cd.IntervalVector([[new_x1.lb(), new_x1.ub()],
                                                 [new_y1.lb(), new_y1.ub()]]))
        
        self.other_pos_1 = np.array(predicted1)
        
        # For other_pos_2 (scout B) - handle multiple boxes
        vel2_speed = self.other_vel_2[0][0]  # speed interval
        vel2_angle = self.other_vel_2[0][1]  # angle interval
        predicted2 = []
        
        for other2_box in self.other_pos_2:
            # Convert polar velocity to Cartesian
            cos_vals = [vel2_speed.lb() * np.cos(vel2_angle.lb()),
                       vel2_speed.lb() * np.cos(vel2_angle.ub()),
                       vel2_speed.ub() * np.cos(vel2_angle.lb()),
                       vel2_speed.ub() * np.cos(vel2_angle.ub())]
            sin_vals = [vel2_speed.lb() * np.sin(vel2_angle.lb()),
                       vel2_speed.lb() * np.sin(vel2_angle.ub()),
                       vel2_speed.ub() * np.sin(vel2_angle.lb()),
                       vel2_speed.ub() * np.sin(vel2_angle.ub())]
            
            vx_interval = cd.Interval(min(cos_vals), max(cos_vals))
            vy_interval = cd.Interval(min(sin_vals), max(sin_vals))
            
            # New position = old position + velocity * dt
            new_x2 = other2_box[0] + vx_interval * dt
            new_y2 = other2_box[1] + vy_interval * dt
            
            predicted2.append(cd.IntervalVector([[new_x2.lb(), new_x2.ub()],
                                                 [new_y2.lb(), new_y2.ub()]]))
        
        self.other_pos_2 = np.array(predicted2)
        



    def update(self, measurement, dt=0.05):
        """
        Update the state based on distance measurement (as intervals).
        
        measurement: dict with keys 'other_pos_1' and 'other_pos_2' containing distance intervals.
        dt: time step used to estimate velocities from position changes
        """
        print("Updating with measurements...")
        if not measurement:
            return

        # Extract current boxes (before contraction)
        self_box = self.self_pos[0]
        other1_box_old = self.other_pos_1[0]
        other2_box_old = self.other_pos_2[0]

        # Subdivide the extracted boxes before contraction
        other1_subdivided = self._subdivide_interval_vector_array(np.array([other1_box_old]), side=0.01)
        other2_subdivided = self._subdivide_interval_vector_array(np.array([other2_box_old]), side=0.01)

        # Contract other_pos_1 using distance to self_pos
        if 'other_pos_1' in measurement:
            d1_interval = measurement['other_pos_1']
            
            contracted1 = []
            for sub_box in other1_subdivided:
                # Build combined vector for self and other1: [mx, my, ax, ay]
                combined1 = cd.IntervalVector([
                    [self_box[0].lb(), self_box[0].ub()],
                    [self_box[1].lb(), self_box[1].ub()],
                    [sub_box[0].lb(), sub_box[0].ub()],
                    [sub_box[1].lb(), sub_box[1].ub()]
                ])
                
                # Distance between (mx,my) and (ax,ay)
                f_dist1 = cd.Function("mx", "my", "ax", "ay", "sqrt((mx-ax)^2 + (my-ay)^2)")
                ctc1 = cd.CtcFwdBwd(f_dist1, d1_interval)
                ctc1.contract(combined1)
                
                # Extract contracted box and keep it if not empty
                contracted_box = cd.IntervalVector([[combined1[2].lb(), combined1[2].ub()],
                                                    [combined1[3].lb(), combined1[3].ub()]])
                if not contracted_box.is_empty():
                    contracted1.append(contracted_box)
            
            # Store all contracted boxes or keep old box if all were eliminated
            if contracted1:
                self.other_pos_1 = np.array(contracted1)
                other1_box_new = self.other_pos_1[0]  # Use first box for velocity calculation
            else:
                other1_box_new = other1_box_old
        else:
            other1_box_new = other1_box_old

        # Contract other_pos_2 using distance to self_pos
        if 'other_pos_2' in measurement:
            d2_interval = measurement['other_pos_2']
            
            contracted2 = []
            for sub_box in other2_subdivided:
                # Build combined vector for self and other2: [mx, my, bx, by]
                combined2 = cd.IntervalVector([
                    [self_box[0].lb(), self_box[0].ub()],
                    [self_box[1].lb(), self_box[1].ub()],
                    [sub_box[0].lb(), sub_box[0].ub()],
                    [sub_box[1].lb(), sub_box[1].ub()]
                ])
                
                # Distance between (mx,my) and (bx,by)
                f_dist2 = cd.Function("mx", "my", "bx", "by", "sqrt((mx-bx)^2 + (my-by)^2)")
                ctc2 = cd.CtcFwdBwd(f_dist2, d2_interval)
                ctc2.contract(combined2)
                
                # Extract contracted box and keep it if not empty
                contracted_box = cd.IntervalVector([[combined2[2].lb(), combined2[2].ub()],
                                                    [combined2[3].lb(), combined2[3].ub()]])
                if not contracted_box.is_empty():
                    contracted2.append(contracted_box)
            
            # Store all contracted boxes or keep old box if all were eliminated
            if contracted2:
                self.other_pos_2 = np.array(contracted2)
                other2_box_new = self.other_pos_2[0]  # Use first box for velocity calculation
            else:
                other2_box_new = other2_box_old
        else:
            other2_box_new = other2_box_old

        # Update velocities based on position change: vel = (pos_new - pos_old) / dt
        if dt > 0:
            # Velocity for other_pos_1
            # Calculate center (average) of old and new position boxes
            old1_center_x = (other1_box_old[0].lb() + other1_box_old[0].ub()) / 2.0
            old1_center_y = (other1_box_old[1].lb() + other1_box_old[1].ub()) / 2.0
            new1_center_x = (other1_box_new[0].lb() + other1_box_new[0].ub()) / 2.0
            new1_center_y = (other1_box_new[1].lb() + other1_box_new[1].ub()) / 2.0
            
            # Calculate displacement
            dx1 = new1_center_x - old1_center_x
            dy1 = new1_center_y - old1_center_y
            
            # Calculate speed (magnitude) and angle
            speed1 = np.sqrt(dx1**2 + dy1**2) / dt
            angle1 = np.arctan2(dy1, dx1)  # radians
            
            # Add uncertainty: speed ± 0.1 m/s, angle ± 1° (≈ 0.01745 rad)
            speed_uncertainty = 0.1
            angle_uncertainty = np.deg2rad(1.0)
            
            # Store as [speed_interval, angle_interval]
            self.other_vel_1[0] = cd.IntervalVector([
                [max(0.0, speed1 - speed_uncertainty), speed1 + speed_uncertainty],
                [angle1 - angle_uncertainty, angle1 + angle_uncertainty]
            ])
            
            # Velocity for other_pos_2
            # Calculate center (average) of old and new position boxes
            old2_center_x = (other2_box_old[0].lb() + other2_box_old[0].ub()) / 2.0
            old2_center_y = (other2_box_old[1].lb() + other2_box_old[1].ub()) / 2.0
            new2_center_x = (other2_box_new[0].lb() + other2_box_new[0].ub()) / 2.0
            new2_center_y = (other2_box_new[1].lb() + other2_box_new[1].ub()) / 2.0
            
            # Calculate displacement
            dx2 = new2_center_x - old2_center_x
            dy2 = new2_center_y - old2_center_y
            
            # Calculate speed (magnitude) and angle
            speed2 = np.sqrt(dx2**2 + dy2**2) / dt
            angle2 = np.arctan2(dy2, dx2)  # radians
            
            # Store as [speed_interval, angle_interval]
            self.other_vel_2[0] = cd.IntervalVector([
                [max(0.0, speed2 - speed_uncertainty), speed2 + speed_uncertainty],
                [angle2 - angle_uncertainty, angle2 + angle_uncertainty]
            ])
        
        

    def subdivide(self, interval_array, side=0.01):
        """Subdivide an array of interval boxes into smaller boxes of given side length.
        
        Args:
            interval_array: numpy array of codac IntervalVectors to subdivide
            side: the side length of each subdivided box (default: 0.01)
            
        Returns:
            numpy array of subdivided IntervalVectors
        """
        return self._subdivide_interval_vector_array(interval_array, side)
    
    def _subdivide_interval_vector_array(self, interval_array, side):
        """Helper function to subdivide an array of 2D IntervalVectors.
        
        Args:
            interval_array: numpy array of codac IntervalVectors
            side: the side length of each subdivided box
            
        Returns:
            numpy array of subdivided IntervalVectors
        """
        print("Subdividing intervals...")
        subdivided = []
        
        for iv in interval_array:
            # Get bounds of the 2D box
            x_min, x_max = iv[0].lb(), iv[0].ub()
            y_min, y_max = iv[1].lb(), iv[1].ub()
            
            # Calculate number of subdivisions needed in each dimension
            nx = int(np.ceil((x_max - x_min) / side))
            ny = int(np.ceil((y_max - y_min) / side))
            
            # Limit maximum subdivisions to prevent creating too many boxes
            max_subdivisions_per_dim = 20  # Maximum 20x20 = 400 boxes per original box
            nx = min(nx, max_subdivisions_per_dim)
            ny = min(ny, max_subdivisions_per_dim)
            
            # Adjust side length if we hit the limit
            actual_side_x = (x_max - x_min) / nx
            actual_side_y = (y_max - y_min) / ny
            
            # Create subdivided boxes
            for i in range(nx):
                for j in range(ny):
                    x_lb = x_min + i * actual_side_x
                    x_ub = min(x_min + (i + 1) * actual_side_x, x_max)
                    y_lb = y_min + j * actual_side_y
                    y_ub = min(y_min + (j + 1) * actual_side_y, y_max)
                    
                    subdivided.append(cd.IntervalVector([[x_lb, x_ub], [y_lb, y_ub]]))
        
        print(f"Created {len(subdivided)} subdivided boxes")
        return np.array(subdivided)


    def plot(self, ax):
        print("Plotting intervals...")

        # Iterate using indexing instead of direct iteration to avoid numpy/codac conflicts
        for i in range(len(self.self_pos)):
            pos = self.self_pos[i]
            ax.add_patch(plt.Rectangle((pos[0].lb(), pos[1].lb()), 
                                       pos[0].ub() - pos[0].lb(), 
                                       pos[1].ub() - pos[1].lb(),
                                       linewidth=1, edgecolor='none', facecolor='blue', alpha=0.5))
            
        for i in range(len(self.other_pos_1)):
            pos = self.other_pos_1[i]
            ax.add_patch(plt.Rectangle((pos[0].lb(), pos[1].lb()), 
                                       pos[0].ub() - pos[0].lb(), 
                                       pos[1].ub() - pos[1].lb(),
                                       linewidth=1, edgecolor='none', facecolor='green', alpha=0.5))
            
        for i in range(len(self.other_pos_2)):
            pos = self.other_pos_2[i]
            ax.add_patch(plt.Rectangle((pos[0].lb(), pos[1].lb()), 
                                       pos[0].ub() - pos[0].lb(), 
                                       pos[1].ub() - pos[1].lb(),
                                       linewidth=1, edgecolor='none', facecolor='red', alpha=0.5))
        