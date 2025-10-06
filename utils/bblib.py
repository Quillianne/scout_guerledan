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

from pymavlink import mavutil
import utils.geo_conversion as geo


# ---------------------------------------------------------------------------
# Lien MAVLink
# ---------------------------------------------------------------------------
class MavlinkLink:
    def __init__(self, conn_str="udp:127.0.0.1:14550", timeout_heartbeat=15):
        """
        conn_str exemples:
          - "udp:127.0.0.1:14550" (à bord, BlueOS)
          - "udp:<IP_Bateau>:14550" (depuis un PC)
          - "serial:/dev/ttyAMA0:115200"
        """
        self.master = mavutil.mavlink_connection(conn_str, autoreconnect=True)
        self.master.wait_heartbeat(timeout=timeout_heartbeat)

    def recv(self, typ, timeout=0.2):
        """Renvoie le dernier message MAVLink du type demandé (ou None)."""
        return self.master.recv_match(type=typ, blocking=True, timeout=timeout)

    def request_rate(self, msg_id, rate_hz):
        """Demande une fréquence d'émission spécifique (si supporté)."""
        # rate_hz -> intervalle en microsecondes
        interval_us = int(1e6 / max(0.1, float(rate_hz)))
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0, 0, 0, 0, 0
        )

    def send_rc_override(self, channels):
        """
        Envoi des commandes RC override pour contrôler les moteurs.
        channels: liste de 8 valeurs PWM (1000-2000), None pour ignorer un canal.
        """
        # Remplacer les None par 65535 (valeur ignore)
        rc_channels = []
        for i in range(8):
            if i < len(channels) and channels[i] is not None:
                rc_channels.append(channels[i])
            else:
                rc_channels.append(65535)  # 65535 = ignore ce canal
        
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_channels
        )

    def set_mode(self, mode):
        """Change le mode de vol (ex: MANUAL, GUIDED, etc.)"""
        mode_id = self.master.mode_mapping().get(mode.upper())
        if mode_id is None:
            print(f"Mode '{mode}' non reconnu")
            return False
        
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        return True

    def arm_disarm(self, arm=True):
        """Arme ou désarme le véhicule"""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if arm else 0,  # 1=arm, 0=disarm
            0, 0, 0, 0, 0, 0
        )


# ---------------------------------------------------------------------------
# Moteurs via MAVLink (BlueBoat)
# ---------------------------------------------------------------------------
class MotorDriver:
    def __init__(self, mav: MavlinkLink):
        """
        Contrôle moteurs BlueBoat via MAVLink RC override.
        
        Pour BlueBoat:
        - Canal 1 (throttle)
        - Canal 3 (rudder)
        
        Valeurs PWM: 1000-2000 (1500 = neutre)
        """
        self.mav = mav
        self.throttle_channel = 0   # Canal 1 (index 0)
        self.steering_channel = 2   # Canal 3 (index 2) 
        self.pwm_neutral = 1500
        self.pwm_min = 1000
        self.pwm_max = 2000
        
    def send_cmd_motor(self, left_motor, right_motor):
        """
        Interface compatible avec l'ancienne API arduino.
        Convertit les commandes left/right en throttle/steering.
        
        :param left_motor: vitesse moteur gauche (-max_speed à +max_speed)
        :param right_motor: vitesse moteur droite (-max_speed à +max_speed)
        """
        # Conversion différentiel vers throttle/steering
        throttle = (left_motor + right_motor) / 2.0  # Vitesse moyenne
        steering = (right_motor - left_motor) / 2.0  # Différence pour la direction
        
        self.send_throttle_steering(throttle, steering)
    
    def send_throttle_steering(self, throttle, steering):
        """
        Envoi des commandes throttle/steering directement.
        
        :param throttle: commande de vitesse (-250 à +250 par exemple)
        :param steering: commande de direction (-250 à +250 par exemple)
        """
        # Normalisation vers PWM 1000-2000
        throttle_pwm = self._normalize_to_pwm(throttle, 250)
        steering_pwm = self._normalize_to_pwm(steering, 250)
        
        # Préparation des 8 canaux RC (None = pas de changement)
        channels = [None] * 8
        channels[self.throttle_channel] = throttle_pwm
        channels[self.steering_channel] = steering_pwm
        
        # Envoi des commandes
        self.mav.send_rc_override(channels)
    
    def _normalize_to_pwm(self, value, max_value):
        """Convertit une valeur [-max_value, +max_value] vers PWM [1000, 2000]"""
        if max_value == 0:
            return self.pwm_neutral
        
        normalized = np.clip(value / max_value, -1.0, 1.0)
        pwm = self.pwm_neutral + normalized * (self.pwm_max - self.pwm_neutral) / 2
        return int(np.clip(pwm, self.pwm_min, self.pwm_max))
    
    def stop_motors(self):
        """Arrête tous les moteurs (position neutre)"""
        channels = [None] * 8
        channels[self.throttle_channel] = self.pwm_neutral
        channels[self.steering_channel] = self.pwm_neutral
        self.mav.send_rc_override(channels)


# ---------------------------------------------------------------------------
# IMU via MAVLink
# ---------------------------------------------------------------------------
class IMU:
    def __init__(self, mav: MavlinkLink, dt=0.01):
        """
        IMU via MAVLink (ArduPilot).
        - roll/pitch/yaw pris dans le message ATTITUDE (EKF, filtré), en radians.
        """
        self.mav = mav
        self.dt = dt
        self._last_euler = (None, None, None)

    def get_euler_angles(self):
        """
        Retourne (roll, pitch, yaw) en radians depuis MAVLink/ATTITUDE.
        ATTITUDE est dans le repère NED, yaw=0 vers le Nord, positif vers l'Est.
        Si aucun message n'est dispo au moment de l'appel, renvoie le dernier connu,
        sinon (None, None, None).
        """
        msg = self.mav.recv("ATTITUDE", timeout=0.2)
        if msg:
            r = float(msg.roll)
            p = float(msg.pitch)
            y = float(msg.yaw)
            self._last_euler = (r, p, y)
            return r, p, y
        return self._last_euler


# ---------------------------------------------------------------------------
# GPS via MAVLink
# ---------------------------------------------------------------------------
class GPS:
    def __init__(self, mav: MavlinkLink, debug=False):
        self.mav = mav
        self.debug = debug
        self.gps_position = None  # (lat, lon)
        self.x = None
        self.y = None
        self.gps_history = []

    def get_gps(self):
        """
        Lit GLOBAL_POSITION_INT pour récupérer (lat, lon) en degrés décimaux.
        Fallback: GPS_RAW_INT si besoin.
        """
        msg = self.mav.recv("GLOBAL_POSITION_INT", timeout=0.2)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
        else:
            msg = self.mav.recv("GPS_RAW_INT", timeout=0.2)
            if not msg:
                return self.gps_position  # renvoie la dernière si on n'a rien
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7

        if self.debug:
            print(f"[GPS] lat={lat:.7f}, lon={lon:.7f}")
        if lat != 0 and lon != 0:
            timestamp = datetime.datetime.now()
            self.gps_position = (lat, lon)
            self.gps_history.append((lat, lon, timestamp))
        return self.gps_position

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
# Navigation (mise à jour pour utiliser MotorDriver MAVLink)
# ---------------------------------------------------------------------------
class Navigation:
    def __init__(self, imu: IMU, gps: GPS, motor_driver: MotorDriver, Kp=1.0, max_speed=250):
        """
        :param imu: IMU (MAVLink) pour l'orientation.
        :param gps: GPS (MAVLink) pour la position.
        :param motor_driver: MotorDriver (MAVLink) pour contrôle moteurs.
        :param Kp: gain proportionnel sur l'erreur de cap.
        :param max_speed: saturation moteur.
        """
        self.imu = imu
        self.dt = imu.dt
        self.gps = gps
        self.motor_driver = motor_driver
        self.Kp = Kp
        self.max_speed = max_speed
        self.history = []

    def get_current_heading(self):
        """Cap actuel (yaw) en degrés depuis l'IMU (ATTITUDE)."""
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

            base_speed = self.max_speed * 0.5
            left_motor = base_speed - correction
            right_motor = base_speed + correction

            left_motor = np.clip(left_motor, -self.max_speed, self.max_speed)
            right_motor = np.clip(right_motor, -self.max_speed, self.max_speed)

            self.motor_driver.send_cmd_motor(left_motor, right_motor)
            print(f"Target:{target_heading_deg:.1f}  Current:{current_heading:.1f}  Error:{error:.1f}", end="\r")

            time.sleep(self.dt)

        self.motor_driver.send_cmd_motor(0, 0)
        print("\nNavigation complete. Motors stopped.")

    def follow_trajectory(self, f, fdot, duration=500, stop_motor=True):
        """
        Suivre une trajectoire paramétrée f(t) (pos) et fdot(t) (vit), pendant 'duration' s.
        """
        t0 = time.time()
        time_elapsed = 0.0
        while time_elapsed < duration:
            t = time.time()
            time_elapsed = t - t0
            x, y = f(t)          # position cible
            vx, vy = fdot(t)     # vitesse cible (non utilisée ici)
            px, py = self.gps.get_coords()  # position bateau
            self.history.append((np.array((x, y)), np.array((px, py))))

            if px is not None and py is not None:
                # cap vers la cible
                vector_to_target = np.array([x, y]) - np.array([px, py])
                distance = np.linalg.norm(vector_to_target)
                heading_to_follow = (-math.degrees(math.atan2(vector_to_target[1], vector_to_target[0])) + 360.0) % 360.0

                current_heading = self.get_current_heading()
                if current_heading is None:
                    time.sleep(self.dt)
                    continue

                error = current_heading - heading_to_follow
                if error > 180: error -= 360
                elif error < -180: error += 360
                correction = self.Kp * error

                # correction proportionnelle à la distance
                reference_distance = 5.0
                distance_correction = math.tanh(distance / reference_distance)

                base_speed = self.max_speed * 0.9
                left_motor = distance_correction * base_speed + correction
                right_motor = distance_correction * base_speed - correction

                left_motor = np.clip(left_motor, -self.max_speed, self.max_speed)
                right_motor = np.clip(right_motor, -self.max_speed, self.max_speed)

                self.motor_driver.send_cmd_motor(left_motor, right_motor)
                print(f"Vm:{distance_correction*base_speed:6.2f}  D_Corr:{distance_correction:4.2f}  Err:{error:6.2f}  Dist:{distance:6.2f}", end="\r")
                time.sleep(self.dt)

        if stop_motor:
            self.motor_driver.send_cmd_motor(0, 0)
        np.savez("log/trajectory.npz", history=self.history)
        self.history = []

    def follow_gps(self, target_coords, cartesian=True, distance=5.0):
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
                target_heading = (-math.degrees(math.atan2(delta[1], delta[0])) + 360.0) % 360.0
                distance_target = float(np.linalg.norm(delta))

                current_heading = self.get_current_heading()
                if current_heading is None:
                    time.sleep(self.dt)
                    continue

                error = current_heading - target_heading
                if error > 180: error -= 360
                elif error < -180: error += 360
                correction = self.Kp * error

                reference_distance = distance
                distance_correction = math.tanh(distance_target / reference_distance)

                base_speed = self.max_speed * 0.9
                left_motor = distance_correction * base_speed + correction
                right_motor = distance_correction * base_speed - correction

                left_motor = np.clip(left_motor, -self.max_speed, self.max_speed)
                right_motor = np.clip(right_motor, -self.max_speed, self.max_speed)

                self.motor_driver.send_cmd_motor(left_motor, right_motor)
                print(f"Vm:{distance_correction*base_speed:6.2f}  D_Corr:{distance_correction:4.2f}  Err:{error:6.2f}  Dist:{distance_target:6.2f}", end="\r")
                time.sleep(self.dt)

        self.motor_driver.send_cmd_motor(0, 0)

    def return_home(self):
        # Exemples de points GPS (à adapter)
        self.follow_gps((48.1990856, -3.0155828), cartesian=False, distance=6)
        self.follow_gps((48.19904833333333, -3.0148149999999996), cartesian=False, distance=6.5)

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
                    self.follow_gps(point, cartesian=True)
            time.sleep(0.1)


# ---------------------------------------------------------------------------
# Fonction utilitaire pour initialisation complète
# ---------------------------------------------------------------------------
def init_blueboat(conn_str="udp:127.0.0.1:14550"):
    """
    Initialise et retourne tous les composants nécessaires pour le BlueBoat.
    
    :param conn_str: chaîne de connexion MAVLink
    :return: (mav_link, imu, gps, motor_driver, navigation)
    """
    # Connexion MAVLink
    mav = MavlinkLink(conn_str)
    
    # (Optionnel) demander des fréquences d'émission confortables
    try:
        mav.request_rate(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)             # ~20 Hz
        mav.request_rate(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)  # ~10 Hz
    except Exception as e:
        print(f"Erreur lors de la configuration des fréquences: {e}")
    
    # Instances des composants
    imu = IMU(mav)
    gps = GPS(mav)
    motor_driver = MotorDriver(mav)
    navigation = Navigation(imu, gps, motor_driver)
    
    return mav, imu, gps, motor_driver, navigation


# ---------------------------------------------------------------------------
# Exemple d'utilisation
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    # Initialisation complète
    mav, imu, gps, motors, nav = init_blueboat("udp:127.0.0.1:14550")
    
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