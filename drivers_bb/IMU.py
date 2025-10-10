from pymavlink import mavutil
import numpy as np

def sawtooth(angle):
    return 2 * np.arctan(np.tan(angle / 2))

class IMU:
    def __init__(self, bb):
        self.bb = bb

    def get_yaw(self, timestamp = False):
        imu_data = self.bb.recv_match(type='ATTITUDE', blocking=True).to_dict()
        if timestamp:
            return - imu_data['yaw'], imu_data['time_boot_ms']
        return - imu_data['yaw']