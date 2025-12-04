"""Boat model for the simulation using numpy column vectors.

This version stores position and velocity as 2x1 numpy column arrays:
  - self.pos is shape (2,1) containing [x, y]^T
  - self.vel is shape (2,1) containing [vx, vy]^T

Theta and angular velocity remain scalars. The class exposes both
numpy-based getters (get_*_np) and backward-compatible tuple/list
getters (get_*, get_state) so existing code keeps working while new
code can use the column-array API.
"""
from math import cos, sin, hypot
import numpy as np


class Boat:
    def __init__(self,
                 x=0.0, y=0.0, theta=0.0,
                 mass=5.0, inertia=0.5,
                 motor_half_distance=0.5,
                 linear_drag_long=4.0, linear_drag_lat=15.0,
                 angular_drag=4.0,
                 max_thrust=15.0,
                 max_speed=10.0):
        # state stored as numpy column arrays for position and velocity
        self.pos = np.array([[float(x)], [float(y)]])  # shape (2,1)
        self.theta = float(theta)
        self.vel = np.array([[0.0], [0.0]])            # shape (2,1)
        self.omega = 0.0

        # physical properties
        self.mass = float(mass)
        self.inertia = float(inertia)
        self.motor_half_distance = float(motor_half_distance)

        # damping (can be anisotropic)
        self.linear_drag_long = float(linear_drag_long)
        self.linear_drag_lat = float(linear_drag_lat)
        self.angular_drag = float(angular_drag)

        # limits
        self.max_thrust = float(max_thrust)
        self.max_speed = float(max_speed)

    # ----------------------- numpy getters -----------------------
    def get_position_np(self):
        """Return position as a 2x1 numpy column array [[x],[y]]."""
        return self.pos.copy()

    def get_velocity_np(self):
        """Return velocity as a 2x1 numpy column array [[vx],[vy]]."""
        return self.vel.copy()

    def get_state_np(self):
        """Return full state as a 6x1 numpy column array:
        [x, y, theta, vx, vy, omega]^T
        """
        return np.vstack((self.pos, np.array([[self.theta]]), self.vel, np.array([[self.omega]])))

    # -------------------- backward compatible API -----------------
    def get_state(self):
        """Backward-compatible: return a Python list [x,y,theta,vx,vy,omega]."""
        return [float(self.pos[0, 0]), float(self.pos[1, 0]), float(self.theta),
                float(self.vel[0, 0]), float(self.vel[1, 0]), float(self.omega)]

    def set_state(self, state):
        """Accept sequence [x,y,theta,vx,vy,omega] (like before) to set state."""
        x, y, theta, vx, vy, omega = map(float, state)
        self.pos = np.array([[x], [y]])
        self.theta = float(theta)
        self.vel = np.array([[vx], [vy]])
        self.omega = float(omega)

    def get_position(self):
        """Backward-compatible: return (x, y, theta)."""
        return float(self.pos[0, 0]), float(self.pos[1, 0]), float(self.theta)

    def get_velocity(self):
        """Backward-compatible: return (vx, vy, omega)."""
        return float(self.vel[0, 0]), float(self.vel[1, 0]), float(self.omega)

    # ----------------------- display method -----------------------
    def display(self, ax, color='blue', alpha=0.8, boat_length=0.8, boat_width=0.5):
        """Display the boat on the given matplotlib axes.
        
        Args:
            ax: matplotlib axes object
            color: color of the boat polygon
            alpha: transparency of the boat (0-1)
            boat_length: length of the boat in meters
            boat_width: width of the boat in meters
        """
        import matplotlib.pyplot as plt
        
        # Get current position
        bx = float(self.pos[0, 0])
        by = float(self.pos[1, 0])
        btheta = float(self.theta)
        
        # Define boat shape in body frame (triangle)
        pts_body = np.array([
            [boat_length/2, 0.0],
            [-boat_length/2, -boat_width/2],
            [-boat_length/2, boat_width/2],
        ])
        
        # Rotation matrix
        c = cos(btheta)
        s = sin(btheta)
        R = np.array([[c, -s], [s, c]])
        
        # Transform to world frame
        pts_world = (R.dot(pts_body.T)).T + np.array([bx, by])
        
        # Draw boat polygon
        poly = plt.Polygon(pts_world, color=color, alpha=alpha)
        ax.add_patch(poly)
        
        # Draw heading arrow
        ax.arrow(bx, by, 0.7 * cos(btheta), 0.7 * sin(btheta),
                 head_width=0.08, head_length=0.12, fc='k', ec='k')

    # ----------------------- dynamics step -----------------------
    def step(self, thrust_l, thrust_r, dt):
        """Integrate dynamics over one timestep.

        thrust_l, thrust_r: commanded thrust (N). They are clamped to +/- max_thrust.
        dt: timestep in seconds.

        The method accepts numeric values or numpy scalars/0-d arrays for thrusts.
        It updates the internal numpy state and returns the backward-compatible
        Python list state (for existing code). New code can call `get_state_np()`.
        """
        # clamp thrusts and coerce to float
        t_l = float(np.clip(float(thrust_l), -self.max_thrust, self.max_thrust))
        t_r = float(np.clip(float(thrust_r), -self.max_thrust, self.max_thrust))

        # total forward force (body frame) and torque
        F_forward = t_l + t_r
        torque = (t_r - t_l) * self.motor_half_distance

        # rotation matrices (world <-> body)
        c = cos(self.theta)
        s = sin(self.theta)
        R_T = np.array([[c, s], [-s, c]])     # world -> body
        R = np.array([[c, -s], [s, c]])       # body -> world

        # body-frame velocities (2x1)
        v_body = R_T.dot(self.vel)

        # accelerations in body frame (simple linear drag model)
        ax_b = (F_forward - self.linear_drag_long * float(v_body[0, 0])) / self.mass
        ay_b = (- self.linear_drag_lat * float(v_body[1, 0])) / self.mass
        alpha = (torque - self.angular_drag * self.omega) / self.inertia

        # world-frame acceleration (2x1)
        a_world = R.dot(np.array([[ax_b], [ay_b]]))

        # integrate velocities and positions (column arrays)
        self.vel = self.vel + a_world * float(dt)

        # enforce speed limit
        sp = hypot(float(self.vel[0, 0]), float(self.vel[1, 0]))
        if sp > self.max_speed:
            scale = float(self.max_speed) / sp
            self.vel = self.vel * scale

        self.pos = self.pos + self.vel * float(dt)

        # angular dynamics
        self.omega = float(self.omega + alpha * float(dt))
        self.theta = float(self.theta + self.omega * float(dt))

        # return backward-compatible state list
        return self.get_state()
