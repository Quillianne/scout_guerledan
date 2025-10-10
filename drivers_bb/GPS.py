from pymavlink import mavutil
import numpy as np

def sawtooth(angle):
    return 2 * np.arctan(np.tan(angle / 2))

class GPS:
    ref_point = (48.199064, -3.015548)
    Rt = 6371e3
    def __init__(self, bb):
        self.bb = bb
        gps_data = self.bb.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()
        # while gps_data['lat'] == 0. or gps_data['lon'] == 0.:
        #     print("Waiting for proper GPS Connection...")
        #     print("Current state:")
        #     print(self.bb.recv_match(type='GPS_STATUS', blocking=True).to_dict())
        #     gps_data = self.bb.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()
        # print("GPS Connected.")

    def get_pos(self, timestamp=False):
        # gps_data = self.bb.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()
        # if gps_data['lat'] != 0 and gps_data['lon'] != 0:
        #     if timestamp:
        #         return (gps_data['lat'] / 1e7, gps_data['lon'] / 1e7), gps_data['time_boot_ms']
        #     return (gps_data['lat'] / 1e7, gps_data['lon'] / 1e7)
        # return
        return (481988630/1e7, -30140880/1e7)

    def get_SOG(self, timestamp = False):
        gps_data = self.bb.recv_match(type='SOG', blocking=True).to_dict()
        if gps_data['lat'] != 0 and gps_data['lon'] != 0:
            if timestamp:
                return np.sqrt(gps_data['vx'] ** 2 + gps_data['vy'] ** 2) / 100, gps_data['time_boot_ms']
            return np.sqrt(gps_data['vx']**2 + gps_data['vy']**2)/100
        return

    def get_COG(self, timestamp = False):
        gps_data = self.bb.recv_match(type='COG', blocking=True).to_dict()
        if gps_data['lat'] != 0 and gps_data['lon'] != 0:
            sog = np.arctan2(gps_data['vy'], gps_data['vx'])
            if timestamp:
                return sawtooth(np.arctan2(gps_data['vy'] / sog, gps_data['vx'] / sog) - np.pi / 2), gps_data['time_boot_ms']
            return sawtooth(np.arctan2(gps_data['vy']/sog, gps_data['vx']/sog) - np.pi/2)
        return

    def get_pos_cart(self, timestamp = False):
    #     gps_data = self.bb.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()
    #     if gps_data['lat'] == 0 or gps_data['lon'] == 0:
    #         return None
        gps_data = {'lat': 481988630, 'lon': -30140880}
        x = GPS.Rt * (gps_data['lon']/1e7 - GPS.ref_point[1])*np.pi/180 * np.cos(gps_data['lat']*np.pi/180/1e7)
        y = GPS.Rt * (gps_data['lat']/1e7 - GPS.ref_point[0])*np.pi/180
        if timestamp:
            return np.array([[x, y]]).T, gps_data['time_boot_ms']
        return np.array([[x, y]]).T

    def gps_2_cart(self, coord):
        lat, long = coord
        x = GPS.Rt * (long - GPS.ref_point[1])*np.pi/180 * np.cos(lat*np.pi/180)
        y = GPS.Rt * (lat - GPS.ref_point[0])*np.pi/180
        return np.array([[x, y]]).T
