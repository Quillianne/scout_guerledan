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
import os
import requests


import utils.geo_conversion as geo

from utils.settings import DT, KP, MAX_CMD, BASE_SPEED_MULTIPLIER


# ---------------------------------------------------------------------------
# Lien MAVLink2Rest (BlueOS)
# ---------------------------------------------------------------------------
class MavlinkLink:
    def __init__(self, host="192.168.2.202", port=6040, sysid=2, compid=1, timeout=3.0, silent_errors=False):
        """
        Connexion via mavlink2rest API pour BlueOS
        """
        self.base = f"http://{host}:{port}"
        self.post_url = f"{self.base}/mavlink"
        self.sysid = int(sysid)
        self.compid = int(compid)
        self.timeout = float(timeout)
        self.silent_errors = silent_errors
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
            if not self.silent_errors:
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
    
    def set_flight_mode(self, mode_id=None, mode_name=None):
        """Set flight mode by id or name (Rover modes)."""
        rover_modes = {
            0: "MANUAL",
            1: "ACRO",
            3: "STEERING",
            4: "HOLD",
            5: "LOITER",
            6: "FOLLOW",
            7: "SIMPLE",
            8: "AUTO",
            9: "RTL",
            10: "SMART_RTL",
            11: "GUIDED",
            12: "INITIALISING",
            15: "AUTO_ZIGZAG",
            16: "AUTOTUNE",
            17: "AUTO_RTGS",
            18: "BRAKE",
        }

        if mode_id is None and mode_name is None:
            raise ValueError("mode_id ou mode_name requis")

        if mode_id is None:
            name = str(mode_name).strip().upper()
            inv = {v: k for k, v in rover_modes.items()}
            if name not in inv:
                raise ValueError(f"Mode inconnu: {mode_name}")
            mode_id = inv[name]

        body = {
            "type": "COMMAND_LONG",
            "command": {"type": "MAV_CMD_DO_SET_MODE"},
            "param1": 1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            "param2": int(mode_id),
            "param3": 0, "param4": 0, "param5": 0, "param6": 0, "param7": 0,
            "target_system": self.sysid, "target_component": self.compid, "confirmation": 0
        }
        return self.post_message(body)

    def get_flight_mode(self):
        """Récupère le mode de vol actuel du véhicule"""
        j = self.get_message("HEARTBEAT")
        if not j or "message" not in j:
            return None
        
        msg = j["message"]
        custom_mode = msg.get("custom_mode", None)
        base_mode = msg.get("base_mode", {})
        
        if custom_mode is None:
            return None
            
        # Modes ArduRover (custom_mode values)
        rover_modes = {
            0: "MANUAL",
            1: "ACRO", 
            3: "STEERING",
            4: "HOLD",
            5: "LOITER",
            6: "FOLLOW",
            7: "SIMPLE",
            10: "AUTO",
            11: "RTL",
            12: "SMART_RTL",
            15: "GUIDED"
        }
        
        mode_name = rover_modes.get(custom_mode, f"UNKNOWN({custom_mode})")
        
        # Vérifier si armé
        armed = bool(base_mode.get("bits", 0) & 128)  # MAV_MODE_FLAG_SAFETY_ARMED
        
        return {
            "mode": mode_name,
            "custom_mode": custom_mode,
            "armed": armed
        }

    def get_battery_status(self):
        """
        Retourne un dict avec tension (V), courant (A) et pourcentage restant (%) si dispo.
        - Essaie BATTERY_STATUS (cellules en mV, courant en cA = 0.01 A)
        - Complète avec SYS_STATUS (voltage_battery en mV, current_battery en cA, battery_remaining en %)
        """
        info = {}

        # 1) BATTERY_STATUS (prioritaire si présent)
        j = self.get_message("BATTERY_STATUS")
        if j and "message" in j:
            m = j["message"]

            # Tensions (mV) : 'voltages' est un tableau par cellule ; valeur 65535 = inconnue
            voltages = m.get("voltages", [])
            valid = [v for v in voltages if v is not None and v != 65535]
            if valid:
                pack_mV = sum(valid)  # si plusieurs cellules valides, somme ≈ tension pack
                info["voltage_V"] = pack_mV / 1000.0
                info["cells_V"] = [v / 1000.0 for v in valid]

            # Courant (BATTERY_STATUS.current_battery) en cA (0.01 A) ; 32767 / -1 = inconnu
            cur = m.get("current_battery", None)
            if cur is not None and cur not in (-1, 32767):
                info["current_A"] = cur / 100.0

            # Pourcentage restant
            rem = m.get("battery_remaining", None)
            if rem is not None and rem != -1:
                info["remaining_percent"] = rem

        # 2) SYS_STATUS (fallback / compléments)
        j2 = self.get_message("SYS_STATUS")
        if j2 and "message" in j2:
            m2 = j2["message"]
            vb = m2.get("voltage_battery", None)  # mV
            if vb is not None and vb != 0 and "voltage_V" not in info:
                info["voltage_V"] = vb / 1000.0

            cur2 = m2.get("current_battery", None)  # cA
            if cur2 is not None and cur2 != -1 and "current_A" not in info:
                info["current_A"] = cur2 / 100.0

            rem2 = m2.get("battery_remaining", None)
            if rem2 is not None and rem2 != -1 and "remaining_percent" not in info:
                info["remaining_percent"] = rem2

        return info if info else None


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

            left_motor = self.base_speed - correction
            right_motor = self.base_speed + correction
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

                left_motor = distance_correction * self.base_speed - correction
                right_motor = distance_correction * self.base_speed + correction

                left_motor = np.clip(left_motor, -self.max_cmd, self.max_cmd)
                right_motor = np.clip(right_motor, -self.max_cmd, self.max_cmd)
                #print(left_motor,right_motor)

                self.motor_driver.send_cmd_motor(left_motor, right_motor)
                print(f"Vm:{distance_correction*self.base_speed:6.2f}  D_Corr:{distance_correction:4.2f}  Err:{error:6.2f}  Dist:{distance_target:6.2f}", end="\r")
                time.sleep(self.dt)

        self.motor_driver.send_cmd_motor(0, 0)

    def go_to_position(self, target):
        """
        Commande 1 shot pour diriger le bateau vers une position cible (x,y).

        Parameters:
        target : np.array([[x], [y]]) - position cible en coordonnées cartésiennes
        """

        current_coords = np.array(self.gps.get_coords(), dtype=float).reshape(2, 1)
        if current_coords[0,0] is None or current_coords[1,0] is None:
            print("Position GPS non disponible.")
            return

        delta = target - current_coords.astype(float)
        distance_target = float(np.linalg.norm(delta))
        if distance_target < 2.0:
            print("Déjà proche de la position cible.")
            return

        target_heading = (math.degrees(math.atan2(delta[1], delta[0])) + 360.0) % 360.0
        current_heading = self.get_current_heading()
        if current_heading is None:
            print("Cap actuel non disponible.")
            return

        error = target_heading - current_heading
        error = (error + 180) % 360 - 180
        correction = self.Kp * error

        reference_distance = 2.0
        distance_correction = math.tanh(distance_target / reference_distance)

        left_motor = distance_correction * self.base_speed - correction
        right_motor = distance_correction * self.base_speed + correction

        left_motor = np.clip(left_motor, -self.max_cmd, self.max_cmd)
        right_motor = np.clip(right_motor, -self.max_cmd, self.max_cmd)

        self.motor_driver.send_cmd_motor(left_motor, right_motor)
        print(f"Commande envoyée: Vm={distance_correction*self.base_speed:6.2f}, D_Corr={distance_correction:4.2f}, Err={error:6.2f}, Dist={distance_target:6.2f}", end="\r")

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
# Classe utilitaire pour config + init depuis fichier
# ---------------------------------------------------------------------------

class BlueBoatConfig:
    """Charge une configuration et initialise un BlueBoat à partir d'un boat_id."""

    def __init__(self, config_path="boat_control_config.json"):
        self.config_path = config_path
        self.boats = self._load_config()

    def _default_config(self):
        return [
            {"boat_id": 1, "host": "192.168.2.201", "port": 6040, "sysid": 1, "compid": 1},
            {"boat_id": 2, "host": "192.168.2.202", "port": 6040, "sysid": 2, "compid": 1},
            {"boat_id": 3, "host": "192.168.2.203", "port": 6040, "sysid": 3, "compid": 1},
        ]

    def _load_config(self):
        if not os.path.exists(self.config_path):
            return self._default_config()
        try:
            with open(self.config_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, list):
                return self._default_config()
            cleaned = []
            for item in data:
                if not isinstance(item, dict) or "boat_id" not in item or "host" not in item:
                    continue
                cleaned.append({
                    "boat_id": int(item["boat_id"]),
                    "host": str(item["host"]),
                    "port": int(item.get("port", 6040)),
                    "sysid": int(item.get("sysid", item["boat_id"])),
                    "compid": int(item.get("compid", 1)),
                })
            return cleaned if cleaned else self._default_config()
        except Exception:
            return self._default_config()

    def get_boat(self, boat_id: int):
        for item in self.boats:
            if int(item.get("boat_id")) == int(boat_id):
                return item
        return None

    def init_from_config(self, boat_id: int, **overrides):
        """Retourne (mav, imu, gps, motor_driver, navigation) à partir du boat_id."""
        item = self.get_boat(boat_id)
        if item is None:
            raise ValueError(f"Boat id {boat_id} introuvable dans la config")

        cfg = {**item, **overrides}
        return init_blueboat(
            host=cfg.get("host"),
            port=cfg.get("port", 6040),
            sysid=cfg.get("sysid", boat_id),
            compid=cfg.get("compid", 1),
            max_cmd=cfg.get("max_cmd", MAX_CMD),
            dt=cfg.get("dt", DT),
            Kp=cfg.get("Kp", KP),
            base_speed_multiplier=cfg.get("base_speed_multiplier", BASE_SPEED_MULTIPLIER),
        )

    def init_all(self, **overrides):
        """Retourne un dict {boat_id: (mav, imu, gps, motor_driver, navigation)}."""
        result = {}
        for item in self.boats:
            boat_id = int(item.get("boat_id"))
            result[boat_id] = self.init_from_config(boat_id, **overrides)
        return result


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