# https://mavlink.io/en/mavgen_python/

from pymavlink import mavutil
import time

# Start a connection listening on a UDP port
bb2 = mavutil.mavlink_connection('udpin:0.0.0.0:14553')

def wait_conn(master):
    """
    Sends a ping to stabilish the UDP communication and awaits for a response
    """
    msg = None
    while not msg:
        master.mav.ping_send(
            int(time.time() * 1e6), # Unix time in microseconds
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
        msg = master.recv_match()
        time.sleep(0.5)

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
wait_conn(bb2)
bb2.mav.ping_send(0, 0, 0, 0)
bb2.wait_heartbeat()

#the_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
print("Heartbeat from system (system %u component %u)" % (bb2.target_system, bb2.target_component))
bb2.mav.command_long_send(
    bb2.target_system,
    bb2.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

print("Waiting for the vehicle to arm")
bb2.motors_armed_wait()
print('Armed!')

print("Starting motors...")
bb2.mav.rc_channels_override_send(bb2.target_system,
                                  bb2.target_component,
                                  1600,
                                  0,
                                  1400,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0)  # RC channel list, in microseconds.
time.sleep(2)
print("Stopping motors...")
bb2.mav.rc_channels_override_send(bb2.target_system,
                                  bb2.target_component,
                                  1500,
                                  0,
                                  1500,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0)  # RC channel list, in microseconds.
time.sleep(2)

bb2.mav.command_long_send(
    bb2.target_system,
    bb2.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)