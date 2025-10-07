from pymavlink import mavutil
import numpy as np
from Motors import Motors
from GPS import GPS
from IMU import IMU
import time

class Boat:
    def __init__(self, ip_addr):
        self.bb = mavutil.mavlink_connection(ip_addr)
        msg = None
        while not msg:
            self.bb.mav.ping_send(
                int(time.time() * 1e6),  # Unix time in microseconds
                0,  # Ping number
                0,  # Request ping of all systems
                0  # Request ping of all components
            )
            msg = self.bb.recv_match()
            time.sleep(0.5)
        self.bb.mav.ping_send(0, 0, 0, 0)
        self.bb.wait_heartbeat()

        self._motors = Motors(self.bb)
        self._gps = GPS(self.bb)
        self._imu = IMU(self.bb)

    def reach_point(self, point):
        A = self._gps.gps_2_cart(point)
        M = self._gps.get_pos_cart()
        dist = np.linalg.norm(A - M)
        while dist > 5:
            M = self._gps.get_pos_cart()
            AM = M - 1
            target_heading = np.atan2(AM[1, 0], AM[0, 0])
            err = self._imu.get_yaw() - target_heading
            print(f"Computed error : {err}")
            steer = 100 * err
            self._motors.send_com(100, steer)
            dist = np.linalg.norm(A - M)
            print(f"Distance to target : {dist}")
        print("Target reached.")



if __name__ == "__main__":
    boat = Boat("udpin:0.0.0.0:14553")
    boat.reach_point((48.198984, -3.015914))



