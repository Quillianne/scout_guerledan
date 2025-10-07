"""Boat model for the simulation.

Provides a Boat class encapsulating state ([x,y,theta,vx,vy,omega]) and
physical parameters like mass, inertia, drag coefficients, motor spacing,
max thrust and max speed. The `step` method integrates dynamics for a
single timestep given left/right motor thrusts.
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
        # state: x, y, theta, vx, vy, omega
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.vx = 0.0
        self.vy = 0.0
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

    def get_state(self):
        return [self.x, self.y, self.theta, self.vx, self.vy, self.omega]

    def set_state(self, state):
        self.x, self.y, self.theta, self.vx, self.vy, self.omega = map(float, state)

    def get_position(self):
        return self.x, self.y, self.theta

    def get_velocity(self):
        return self.vx, self.vy, self.omega

    def step(self, thrust_l, thrust_r, dt):
        """Integrate dynamics over one timestep.

        thrust_l, thrust_r: commanded thrust (N). They are clamped to +/- max_thrust.
        dt: timestep in seconds.
        """
        # clamp thrusts
        t_l = max(-self.max_thrust, min(self.max_thrust, float(thrust_l)))
        t_r = max(-self.max_thrust, min(self.max_thrust, float(thrust_r)))

        # total forward force (body frame) and torque
        F_forward = t_l + t_r
        torque = (t_r - t_l) * self.motor_half_distance

        # body-frame velocities
        c = cos(self.theta)
        s = sin(self.theta)
        v_world = np.array([self.vx, self.vy])
        R_T = np.array([[c, s], [-s, c]])
        v_body = R_T.dot(v_world)

        # accelerations in body frame (simple linear drag model)
        ax_b = (F_forward - self.linear_drag_long * v_body[0]) / self.mass
        ay_b = (- self.linear_drag_lat * v_body[1]) / self.mass
        alpha = (torque - self.angular_drag * self.omega) / self.inertia

        # back to world frame
        R = np.array([[c, -s], [s, c]])
        a_world = R.dot(np.array([ax_b, ay_b]))

        # integrate
        self.vx += a_world[0] * dt
        self.vy += a_world[1] * dt

        # enforce speed limit
        sp = hypot(self.vx, self.vy)
        if sp > self.max_speed:
            scale = self.max_speed / sp
            self.vx *= scale
            self.vy *= scale

        self.x += self.vx * dt
        self.y += self.vy * dt

        self.omega += alpha * dt
        self.theta += self.omega * dt

        return self.get_state()
