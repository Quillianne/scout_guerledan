from pymavlink import mavutil
import numpy as np

class Motors:
    def __init__(self, bb):
        self.bb = bb
        self.arm()

    def arm(self):
        print("Sending arm command")
        self.bb.mav.command_long_send(
            self.bb.target_system,
            self.bb.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)
        print("Waiting for the vehicle to arm")
        self.bb.motors_armed_wait()
        print('Armed!')

    def disarm(self):
        print("Sending disarm command")
        self.bb.mav.command_long_send(
            self.bb.target_system,
            self.bb.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)
        print("Waiting for the vehicle to disarm")
        self.bb.motors_disarmed_wait()
        print('Disarmed!')

    def send_com(self, throttle, steer):
        """
        Send a command to the motors. Command given must be between 400 and -400.
        """
        print(f"Sending command ({throttle}, {steer})")
        self.bb.mav.rc_channels_override_send(self.bb.target_system,
                                         self.bb.target_component,
                                         np.clip(-int(throttle) + 1500, 1100, 1900),
                                         0,
                                         np.clip(int(steer) + 1500, 1100, 1900),
                                         0,
                                         0,
                                         0,
                                         0,
                                         0)  # RC channel list, in microseconds.
        print("Command sent.")

    def stop(self):
        print("Stopping motors...")
        self.bb.mav.rc_channels_override_send(self.bb.target_system,
                                         self.bb.target_component,
                                         1500,
                                         0,
                                         1500,
                                         0,
                                         0,
                                         0,
                                         0,
                                         0)

    def status(self):
        return self.bb.motors_armed()

    def __exit__(self):
        self.stop()
        self.disarm()
    def __del__(self):
        self.stop()
        self.disarm()