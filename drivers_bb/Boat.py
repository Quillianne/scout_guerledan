from pymavlink import mavutil
import numpy as np
from Motors import Motors
from GPS import GPS
from IMU import IMU
import time

def sawtooth(angle):
    return 2 * np.arctan(np.tan(angle / 2))

class Boat:
    def __init__(self, ip_addr):
        self._num = int(ip_addr[-1])
        self._bb = mavutil.mavlink_connection(ip_addr)
        msg = None
        while not msg:
            self._bb.mav.ping_send(
                int(time.time() * 1e6),  # Unix time in microseconds
                0,  # Ping number
                0,  # Request ping of all systems
                0  # Request ping of all components
            )
            msg = self._bb.recv_match()
            time.sleep(0.5)
        self._bb.mav.ping_send(0, 0, 0, 0)
        self._bb.wait_heartbeat()

        self._motors = Motors(self._bb)
        self._gps = GPS(self._bb)
        self._imu = IMU(self._bb)

    def reach_point(self, point):
        A = self._gps.gps_2_cart(point)
        M = self._gps.get_pos_cart()
        dist = np.linalg.norm(A - M)
        while dist > 5:
            M = self._gps.get_pos_cart()
            MA = A - M
            # print(f"Current position: {self._gps.get_pos()}")
            # print(f"Current cartesian position: {M}")
            # print(f"Target cartesian position: {A}")
            target_heading = sawtooth(np.arctan2(MA[1, 0], MA[0, 0]) - np.pi/2)
            print(f'target_heading : {target_heading * 180 / np.pi}')
            print(f'Heading : {self._imu.get_yaw() * 180 / np.pi}')
            err = sawtooth(target_heading - self._imu.get_yaw())
            print(f"Computed error : {err}")
            steer = 300 * err
            self._motors.send_com(200, steer)
            dist = np.linalg.norm(A - M)
            print(f"Distance to target : {dist}")
        print("Target reached.")

    def formation_nav_GPS(self):
        Kp = 100
        Kd = 50
        Ki = 50
        if self._num == 2:
            shift = np.array([[7, 3]]).T
        elif self._num == 3:
            shift = np.array([[7, -3]]).T
        else:
            return
        list_err = []
        list_timestamp = []
        Ms = None
        yaw_ms = None
        speed_ms = None
        R_ms = np.array([[np.cos(yaw_ms), -np.sin(yaw_ms)],
                         [np.sin(yaw_ms), np.cos(yaw_ms)]])
        target_pos = Ms + R_ms @ shift
        BS =  target_pos - self._gps.get_pos_cart()
        target_yaw = np.arctan2(BS[1, 0], BS[0, 0])
        err_yaw = self._imu.get_yaw() - target_yaw
        print(f"Computed yaw error : {err_yaw}")
        steer = np.clip(100 * err_yaw, -400, 400)
        target_speed = speed_ms + np.tanh(np.linalg.norm(BS) / 5)
        err = self._gps.get_SOG() - target_speed
        list_err.append(err)
        list_timestamp.append(self.bb.recv_match(type='SYSTEM_TIME', blocking=True).to_dict()['time_boot_ms'])
        if len(list_err) > 1:
            err_der = (list_err[-1] - list_err[-2])/(list_timestamp[-1] - list_timestamp[-2])
            err_int = np.sum([list_err[i-1] * (list_timestamp[i] - list_timestamp[i-1]) / 1000 for i in range(-1, -1, max(0, -100))])
        else:
            err_der = 0
            err_int = 0
        throtle = 200 + Kp * err + Kd * err_der + Ki * err_int
        self._motors.send_com(throtle, steer)


    def __exit__(self, exc_type, exc_value, traceback):
        del (self._motors)
        del (self._gps)
        del (self._imu)
        self._bb.close()

    def __del__(self):
        del(self._motors)
        del(self._gps)
        del(self._imu)
        self._bb.close()

if __name__ == "__main__":
    boat = Boat("udpin:0.0.0.0:14552")
    boat.reach_point((48.199047, -3.014776))



