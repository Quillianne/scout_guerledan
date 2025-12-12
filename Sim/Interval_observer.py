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



    def predict(self, pos, control_input, inertial_measurements, dt=0.05):
        """Predict the next state based on control input and time step.
        
        Uses Euler integration to propagate position intervals forward in time
        based on current velocity intervals.
        
        Args:
            pos: current self position (2x1 numpy array)
            control_input: control inputs (not used currently)
            inertial_measurements: inertial sensor measurements (not used currently)
            dt: time step in seconds
        """
        # Update self position with small uncertainty
        self.self_pos = np.array([cd.IntervalVector([[pos[0,0]-0.1, pos[0,0]+0.1],
                                                     [pos[1,0]-0.1, pos[1,0]+0.1]])])
        
        # Predict other boats' positions using Euler integration: pos = pos + vel * dt
        # For other_pos_1 (scout A)
        other1_box = self.other_pos_1[0]
        other1_vel = self.other_vel_1[0]
        
        # New position = old position + velocity * dt
        new_x1 = other1_box[0] + other1_vel[0] * dt
        new_y1 = other1_box[1] + other1_vel[1] * dt
        
        self.other_pos_1[0] = cd.IntervalVector([[new_x1.lb(), new_x1.ub()],
                                                 [new_y1.lb(), new_y1.ub()]])
        
        # For other_pos_2 (scout B)
        other2_box = self.other_pos_2[0]
        other2_vel = self.other_vel_2[0]
        
        new_x2 = other2_box[0] + other2_vel[0] * dt
        new_y2 = other2_box[1] + other2_vel[1] * dt
        
        self.other_pos_2[0] = cd.IntervalVector([[new_x2.lb(), new_x2.ub()],
                                                 [new_y2.lb(), new_y2.ub()]])
        



    def update(self, measurement, dt=0.05):
        """
        Update the state based on distance measurement (as intervals).
        
        measurement: dict with keys 'other_pos_1' and 'other_pos_2' containing distance intervals.
        dt: time step used to estimate velocities from position changes
        """
        if not measurement:
            return

        # Extract current boxes (before contraction)
        self_box = self.self_pos[0]
        other1_box_old = self.other_pos_1[0]
        other2_box_old = self.other_pos_2[0]

        # Contract other_pos_1 using distance to self_pos
        if 'other_pos_1' in measurement:
            d1_interval = measurement['other_pos_1']
            
            # Build combined vector for self and other1: [mx, my, ax, ay]
            combined1 = cd.IntervalVector([
                [self_box[0].lb(), self_box[0].ub()],
                [self_box[1].lb(), self_box[1].ub()],
                [other1_box_old[0].lb(), other1_box_old[0].ub()],
                [other1_box_old[1].lb(), other1_box_old[1].ub()]
            ])
            
            # Distance between (mx,my) and (ax,ay)
            f_dist1 = cd.Function("mx", "my", "ax", "ay", "sqrt((mx-ax)^2 + (my-ay)^2)")
            ctc1 = cd.CtcFwdBwd(f_dist1, d1_interval)
            ctc1.contract(combined1)
            
            # Extract contracted other_pos_1
            other1_box_new = cd.IntervalVector([[combined1[2].lb(), combined1[2].ub()],
                                                [combined1[3].lb(), combined1[3].ub()]])
        else:
            other1_box_new = other1_box_old

        # Contract other_pos_2 using distance to self_pos
        if 'other_pos_2' in measurement:
            d2_interval = measurement['other_pos_2']
            
            # Build combined vector for self and other2: [mx, my, bx, by]
            combined2 = cd.IntervalVector([
                [self_box[0].lb(), self_box[0].ub()],
                [self_box[1].lb(), self_box[1].ub()],
                [other2_box_old[0].lb(), other2_box_old[0].ub()],
                [other2_box_old[1].lb(), other2_box_old[1].ub()]
            ])
            
            # Distance between (mx,my) and (bx,by)
            f_dist2 = cd.Function("mx", "my", "bx", "by", "sqrt((mx-bx)^2 + (my-by)^2)")
            ctc2 = cd.CtcFwdBwd(f_dist2, d2_interval)
            ctc2.contract(combined2)
            
            # Extract contracted other_pos_2
            other2_box_new = cd.IntervalVector([[combined2[2].lb(), combined2[2].ub()],
                                                [combined2[3].lb(), combined2[3].ub()]])
        else:
            other2_box_new = other2_box_old

        # Update velocities based on position change: vel = (pos_new - pos_old) / dt
        if dt > 0:
            # Velocity for other_pos_1
            vel1_x = (other1_box_new[0] - other1_box_old[0]) / dt
            vel1_y = (other1_box_new[1] - other1_box_old[1]) / dt
            self.other_vel_1[0] = cd.IntervalVector([[vel1_x.lb(), vel1_x.ub()],
                                                      [vel1_y.lb(), vel1_y.ub()]])
            
            # Velocity for other_pos_2
            vel2_x = (other2_box_new[0] - other2_box_old[0]) / dt
            vel2_y = (other2_box_new[1] - other2_box_old[1]) / dt
            self.other_vel_2[0] = cd.IntervalVector([[vel2_x.lb(), vel2_x.ub()],
                                                      [vel2_y.lb(), vel2_y.ub()]])

        # Write contracted intervals back to stored IntervalVectors
        # self_pos stays unchanged
        self.other_pos_1[0] = other1_box_new
        self.other_pos_2[0] = other2_box_new
        


    def plot(self, ax):
        for pos in self.self_pos:
            ax.add_patch(plt.Rectangle((pos[0].lb(), pos[1].lb()), 
                                       pos[0].ub() - pos[0].lb(), 
                                       pos[1].ub() - pos[1].lb(),
                                       linewidth=1, edgecolor='none', facecolor='blue', alpha=0.5))
            
        for pos in self.other_pos_1:
            ax.add_patch(plt.Rectangle((pos[0].lb(), pos[1].lb()), 
                                       pos[0].ub() - pos[0].lb(), 
                                       pos[1].ub() - pos[1].lb(),
                                       linewidth=1, edgecolor='none', facecolor='green', alpha=0.5))
        for pos in self.other_pos_2:
            ax.add_patch(plt.Rectangle((pos[0].lb(), pos[1].lb()), 
                                       pos[0].ub() - pos[0].lb(), 
                                       pos[1].ub() - pos[1].lb(),
                                       linewidth=1, edgecolor='none', facecolor='red', alpha=0.5))
        