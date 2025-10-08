# -*- coding: utf-8 -*-
"""
BlueBoat — IMU/GPS/Motors via MAVLink (ArduRover/BlueOS)
========================================================
- Remplace les drivers bruts IMU/GPS par la télémétrie MAVLink.
- IMU.get_euler_angles() lit ATTITUDE (EKF) → (roll, pitch, yaw) en radians.
- GPS.get_gps()/get_coords() lit GLOBAL_POSITION_INT ou GPS_RAW_INT.
- MotorDriver utilise RC_CHANNELS_OVERRIDE pour contrôler les moteurs.
- Navigation conserve la même API; seules les lectures capteurs changent.
"""

import numpy as np
import time
import datetime
import socket
import math
import json
import requests


import utils.geo_conversion as geo

from utils.settings import DT, KP, MAX_CMD, BASE_SPEED_MULTIPLIER


# ---------------------------------------------------------------------------
# Lien MAVLink2Rest (BlueOS)
# ---------------------------------------------------------------------------
class MavlinkLink:
    def __init__(self, host="192.168.2.202", port=6040, sysid=2, compid=1, timeout=3.0):
        """
        Connexion via mavlink2rest API pour BlueOS
        """
        self.base = f"http://{host}:{port}"
        self.post_url = f"{self.base}/mavlink"
        self.sysid = int(sysid)
        self.compid = int(compid)
        self.timeout = float(timeout)
        self.session = requests.Session()
        self.session.trust_env = False  # ignore proxies
        self.headers = {"Content-Type": "application/json", "Accept": "application/json", "Connection": "close"}

    def _payload(self, message: dict):
        return {"header": {"system_id": 255, "component_id": 0, "sequence": 0}, "message": message}

    def post_message(self, message: dict):
        """Envoie un message MAVLink via REST"""
        try:
            r = self.session.post(self.post_url, data=json.dumps(self._payload(message)),
                                headers=self.headers, timeout=self.timeout)
            if not r.ok:
                raise RuntimeError(f"POST {message.get('type')} -> {r.status_code} {r.text}")
            return r.text.strip()
        except Exception as e:
            print(f"Erreur POST message: {e}")
            return None

    def get_message(self, msg_name: str):
        """Récupère le dernier message du type spécifié"""
        url = f"{self.base}/mavlink/vehicles/{self.sysid}/components/{self.compid}/messages/{msg_name}"
        try:
            r = self.session.get(url, timeout=self.timeout)
            if not r.ok:
                return None
            return r.json()
        except Exception:
            return None

    def send_rc_override(self, ch1=None, ch3=None):
        """Envoi commande RC override pour moteurs"""
        body = {
            "type": "RC_CHANNELS_OVERRIDE",
            "target_system": self.sysid, "target_component": self.compid,
            "chan1_raw": 0, "chan2_raw": 0, "chan3_raw": 0, "chan4_raw": 0,
            "chan5_raw": 0, "chan6_raw": 0, "chan7_raw": 0, "chan8_raw": 0
        }
        if ch1 is not None: body["chan1_raw"] = int(ch1)
        if ch3 is not None: body["chan3_raw"] = int(ch3)
        return self.post_message(body)

    def arm_disarm(self, arm=True):
        """Arme ou désarme le véhicule via REST"""
        return self.post_message({
            "type": "COMMAND_LONG",
            "command": {"type": "MAV_CMD_COMPONENT_ARM_DISARM"},
            "param1": 1 if arm else 0,
            "param2": 0, "param3": 0, "param4": 0, "param5": 0, "param6": 0, "param7": 0,
            "target_system": self.sysid, "target_component": self.compid, "confirmation": 0
        })


# ---------------------------------------------------------------------------
# Moteurs via MAVLink2Rest (BlueOS)
# ---------------------------------------------------------------------------
class MotorDriver:
    def __init__(self, mav: MavlinkLink, max_cmd=250.0):
        """
        Contrôle moteurs BlueBoat via mavlink2rest.
        Compatible avec l'implémentation test_mavlink.py
        """
        self.mav = mav
        self.max_cmd = max_cmd
        
    def __del__(self):
        self.stop_motors()
        self.mav.arm_disarm(arm=False)
        

    @staticmethod
    def clamp(x, lo, hi): 
        return lo if x < lo else hi if x > hi else x

    def lr_to_ts(self, left: float, right: float):
        """
        Convert left/right en throttle/steering normalisés puis PWM
        Basé sur test_mavlink.py
        """
        L = self.clamp(float(left) / self.max_cmd, -1.0, 1.0)
        R = self.clamp(float(right) / self.max_cmd, -1.0, 1.0)
        t = self.clamp((L + R) / 2.0, -1.0, 1.0)  # forward/back
        s = self.clamp((R - L) / 2.0, -1.0, 1.0)  # right/left

        ch1 = int(round(1500 - 500 * t))  # throttle: +t -> forward -> 1000
        ch3 = int(round(1500 + 500 * s))  # steering: +s -> right
        ch1 = self.clamp(ch1, 1000, 2000)
        ch3 = self.clamp(ch3, 1000, 2000)
        return ch1, ch3, t, s

    def send_cmd_motor(self, left_motor, right_motor):
        """Interface compatible avec ancienne API arduino"""
        ch1, ch3, t, s = self.lr_to_ts(left_motor, right_motor)
        return self.mav.send_rc_override(ch1=ch1, ch3=ch3)
    
    def drive_lr(self, left: float, right: float, seconds=3.0, rate_hz=10.0):
        """Commande moteurs pendant une durée donnée (comme test_mavlink.py)"""
        ch1, ch3, t, s = self.lr_to_ts(left, right)
        dt = 1.0 / self.clamp(rate_hz, 1.0, 50.0)
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.mav.send_rc_override(ch1=ch1, ch3=ch3)
            time.sleep(dt)
        # Neutre
        self.mav.send_rc_override(ch1=1500, ch3=1500)
        return {"ch1": ch1, "ch3": ch3, "throttle_norm": t, "steering_norm": s}
    
    def stop_motors(self):
        """Arrête tous les moteurs (position neutre)"""
        return self.mav.send_rc_override(ch1=1500, ch3=1500)


# ---------------------------------------------------------------------------
# IMU via MAVLink2Rest (BlueOS)
# ---------------------------------------------------------------------------
class IMU:
    def __init__(self, mav: MavlinkLink, dt=DT):
        """
        IMU via mavlink2rest (compatible test_mavlink.py)
        """
        self.mav = mav
        self.dt = dt
        self._last_euler = (None, None, None)

    def get_euler_angles(self):
        """
        Retourne (roll, pitch, yaw) en radians depuis mavlink2rest/ATTITUDE
        Compatible avec test_mavlink.py get_attitude()
        """
        msg_data = self.mav.get_message("ATTITUDE")
        if msg_data and "message" in msg_data:
            m = msg_data["message"]
            r = float(m.get("roll", 0.0))
            p = float(m.get("pitch", 0.0))
            y = float(m.get("yaw", 0.0))
            self._last_euler = (r, p, y)
            return r, p, y
        return self._last_euler

    def get_attitude_dict(self):
        """
        Version étendue retournant un dictionnaire comme test_mavlink.py
        """
        msg_data = self.mav.get_message("ATTITUDE")
        if not msg_data or "message" not in msg_data:
            return None
        
        m = msg_data["message"]
        roll = float(m.get("roll", 0.0))
        pitch = float(m.get("pitch", 0.0))
        yaw = float(m.get("yaw", 0.0))
        yaw_deg = (math.degrees(yaw) + 360.0) % 360.0
        return {"roll": roll, "pitch": pitch, "yaw": yaw, "yaw_deg": yaw_deg}


# ---------------------------------------------------------------------------
# GPS via MAVLink2Rest (BlueOS)
# ---------------------------------------------------------------------------
class GPS:
    def __init__(self, mav: MavlinkLink, debug=False):
        """
        GPS via mavlink2rest (compatible test_mavlink.py)
        """
        self.mav = mav
        self.debug = debug
        self.gps_position = None  # (lat, lon)
        self.x = None
        self.y = None
        self.gps_history = []

    def get_gps(self):
        """
        Compatible avec test_mavlink.py get_gps()
        Essaie GLOBAL_POSITION_INT puis GPS_RAW_INT
        """
        # Essayer GLOBAL_POSITION_INT d'abord
        msg_data = self.mav.get_message("GLOBAL_POSITION_INT")
        if msg_data and "message" in msg_data and msg_data["message"].get("lat") not in (None, 0):
            m = msg_data["message"]
            lat = m["lat"] / 1e7
            lon = m["lon"] / 1e7
        else:
            # Fallback vers GPS_RAW_INT
            msg_data = self.mav.get_message("GPS_RAW_INT")
            if not msg_data or "message" not in msg_data or msg_data["message"].get("lat") in (None, 0):
                return self.gps_position  # renvoie la dernière si on n'a rien
            m = msg_data["message"]
            lat = m["lat"] / 1e7
            lon = m["lon"] / 1e7

        if self.debug:
            print(f"[GPS] lat={lat:.7f}, lon={lon:.7f}")
        
        if lat != 0 and lon != 0:
            timestamp = datetime.datetime.now()
            self.gps_position = (lat, lon)
            self.gps_history.append((lat, lon, timestamp))
        
        return self.gps_position

    def get_gps_dict(self):
        """
        Version étendue retournant un dictionnaire comme test_mavlink.py
        """
        # Essayer GLOBAL_POSITION_INT d'abord
        msg_data = self.mav.get_message("GLOBAL_POSITION_INT")
        if msg_data and "message" in msg_data and msg_data["message"].get("lat") not in (None, 0):
            m = msg_data["message"]
            return {
                "lat": m["lat"] / 1e7,
                "lon": m["lon"] / 1e7,
                "alt_m": m.get("alt", 0) / 1000.0,
                "hdg_deg": (m.get("hdg", 0) / 100.0) if m.get("hdg") is not None else None
            }
        
        # Fallback vers GPS_RAW_INT
        msg_data = self.mav.get_message("GPS_RAW_INT")
        if msg_data and "message" in msg_data and msg_data["message"].get("lat") not in (None, 0):
            m = msg_data["message"]
            return {
                "lat": m["lat"] / 1e7,
                "lon": m["lon"] / 1e7,
                "alt_m": m.get("alt", 0) / 1000.0,
                "hdg_deg": (m.get("cog", 0) / 100.0) if m.get("cog") is not None else None
            }
        
        return None

    def get_coords(self):
        """Renvoie les coordonnées cartésiennes (x,y) du bateau."""
        point = self.get_gps()
        if point is not None:
            self.x, self.y = geo.conversion_spherique_cartesien(point)
        return self.x, self.y

    def export_gpx(self, filename="log/output.gpx"):
        """
        Exporte l'historique GPS au format GPX avec timestamps.
        """
        gpx_header = """<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="GPS Python Class" xmlns="http://www.topografix.com/GPX/1/1">
  <trk>
    <name>GPS Track</name>
    <trkseg>
"""
        gpx_footer = """    </trkseg>
  </trk>
</gpx>
"""
        with open(filename, "w", encoding="utf-8") as f:
            f.write(gpx_header)
            for lat, lon, dt in self.gps_history:
                if isinstance(dt, datetime.datetime):
                    time_str = dt.isoformat() + "Z"
                else:
                    time_str = str(dt)
                f.write(f'      <trkpt lat="{lat}" lon="{lon}">\n')
                f.write(f'        <time>{time_str}</time>\n')
                f.write('      </trkpt>\n')
            f.write(gpx_footer)


# ---------------------------------------------------------------------------
# Navigation via MAVLink2Rest (BlueOS)
# ---------------------------------------------------------------------------
class Navigation:
    def __init__(self, imu: IMU, gps: GPS, motor_driver: MotorDriver, Kp=3.0, max_cmd=250, base_speed_multiplier=0.5):
        """
        Navigation utilisant les drivers mavlink2rest
        """
        self.imu = imu
        self.dt = imu.dt
        self.gps = gps
        self.motor_driver = motor_driver
        self.Kp = Kp
        self.max_cmd = max_cmd
        self.base_speed = max_cmd*base_speed_multiplier
        self.history = []

    def get_current_heading(self):
        """Cap actuel (yaw) en degrés depuis l'IMU REST"""
        _, _, yaw = self.imu.get_euler_angles()
        if yaw is None:
            return None
        return (math.degrees(yaw) + 360.0) % 360.0

    def follow_heading(self, target_heading_deg, duration_s):
        """
        Suivre un cap en degrés pendant duration_s.
        """
        t0 = time.time()
        while time.time() - t0 < duration_s:
            current_heading = self.get_current_heading()
            if current_heading is None:
                time.sleep(self.dt)
                continue

            # erreur de cap dans [-180, 180]
            error = target_heading_deg - current_heading
            error = (error + 180) % 360 - 180

            correction = self.Kp * error

            left_motor = self.base_speed + correction
            right_motor = self.base_speed - correction
            #print(left_motor,right_motor)
            left_motor = np.clip(left_motor, -self.max_cmd, self.max_cmd)
            right_motor = np.clip(right_motor, -self.max_cmd, self.max_cmd)

            self.motor_driver.send_cmd_motor(left_motor, right_motor)
            print(f"Target:{target_heading_deg:.1f}  Current:{current_heading:.1f}  Error:{error:.1f}", end="\r")

            time.sleep(self.dt)

        self.motor_driver.send_cmd_motor(0, 0)
        print("\nNavigation complete. Motors stopped.")



    def go_to_gps(self, target_coords, cartesian=True, distance=5.0):
        """
        Aller vers une position (cartésienne ou GPS) et s'arrêter à 'distance' mètres.
        """
        # Si coordonnées GPS -> conversion en cartesien
        if not cartesian:
            target_coords = geo.conversion_spherique_cartesien(target_coords)
        target_coords = np.array(target_coords, dtype=float)

        distance_target = float("inf")
        while distance_target > distance:
            current_coords = np.array(self.gps.get_coords(), dtype=object)
            if current_coords[0] is not None and current_coords[1] is not None:
                delta = target_coords - current_coords.astype(float)
                target_heading = (math.degrees(math.atan2(delta[1], delta[0])) + 360.0) % 360.0
                distance_target = float(np.linalg.norm(delta))

                current_heading = self.get_current_heading()
                if current_heading is None:
                    time.sleep(self.dt)
                    continue

                error = target_heading - current_heading
                error = (error + 180) % 360 - 180
                correction = self.Kp * error

                reference_distance = distance
                distance_correction = math.tanh(distance_target / reference_distance)

                left_motor = distance_correction * self.base_speed + correction
                right_motor = distance_correction * self.base_speed - correction

                left_motor = np.clip(left_motor, -self.max_cmd, self.max_cmd)
                right_motor = np.clip(right_motor, -self.max_cmd, self.max_cmd)
                #print(left_motor,right_motor)

                self.motor_driver.send_cmd_motor(left_motor, right_motor)
                print(f"Vm:{distance_correction*self.base_speed:6.2f}  D_Corr:{distance_correction:4.2f}  Err:{error:6.2f}  Dist:{distance_target:6.2f}", end="\r")
                time.sleep(self.dt)

        self.motor_driver.send_cmd_motor(0, 0)

    def return_home(self):
        # Exemples de points GPS (à adapter)
        self.go_to_gps((48.1990856, -3.0155828), cartesian=False, distance=6)
        self.go_to_gps((48.1990483, -3.014815), cartesian=False, distance=10)

    def stay_at(self, point, cartesien=False):
        """
        Maintenir la position autour d'un point (simple logique de rappel).
        """
        if not cartesien:
            point = geo.conversion_spherique_cartesien(point)

        while True:
            current_position = np.array(self.gps.get_coords(), dtype=object)
            if current_position[0] is not None and current_position[1] is not None:
                if np.linalg.norm(current_position.astype(float) - np.array(point, dtype=float)) > 5.0:
                    self.go_to_gps(point, cartesian=True)
            time.sleep(0.1)


# ---------------------------------------------------------------------------
# Fonction utilitaire pour initialisation complète
# ---------------------------------------------------------------------------

def init_blueboat(host="192.168.2.202", port=6040, sysid=2, compid=1, max_cmd=MAX_CMD, dt=DT ,Kp=KP ,base_speed_multiplier=BASE_SPEED_MULTIPLIER):
    """
    Initialise et retourne tous les composants nécessaires pour le BlueBoat.
    
    :param host: adresse IP du BlueOS
    :param port: port mavlink2rest 
    :param sysid: system ID
    :param compid: component ID
    :return: (mav_link, imu, gps, motor_driver, navigation)
    """
    # Connexion MAVLink
    mav = MavlinkLink(host=host, port=port, sysid=sysid, compid=compid)
    
    # Instances des composants
    imu = IMU(mav, dt=dt)
    gps = GPS(mav)
    motor_driver = MotorDriver(mav)
    navigation = Navigation(imu, gps, motor_driver, max_cmd=max_cmd, Kp=Kp, base_speed_multiplier=base_speed_multiplier)
    
    return mav, imu, gps, motor_driver, navigation


# ---------------------------------------------------------------------------
# Exemple d'utilisation
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    # Initialisation complète
    mav, imu, gps, motors, nav = init_blueboat(host="192.168.2.202")
    
    # Test lecture cap/yaw
    r, p, y = imu.get_euler_angles()
    if y is not None:
        print("Yaw (deg):", (math.degrees(y) + 360.0) % 360.0)
    else:
        print("ATTITUDE non disponible pour le moment.")
    
    # Test GPS
    pos = gps.get_gps()
    if pos:
        print(f"Position GPS: {pos}")
    
    # Test moteurs (attention: le bateau bougera!)
    # motors.send_cmd_motor(100, 100)  # Avancer doucement
    # time.sleep(2)
    # motors.stop_motors()
    
    print("Tests terminés.")