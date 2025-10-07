from pymavlink import mavutil
import numpy as np

def sawtooth(angle):
    return 2 * np.atan(np.tan(angle / 2))

class IMU:
    def __init__(self, bb):
        self.bb = bb

    def get_yaw(self):
        imu_data = self.bb.recv_match(type='ATTITUDE', blocking=True).to_dict()
        return -sawtooth(imu_data['yaw'])