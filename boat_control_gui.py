#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interface graphique de contrôle des bateaux Scout
================================================
Interface pour surveiller et contrôler les bateaux 2 et 3 :
- GPS (latitude, longitude)
- Batterie (tension, pourcentage)
- Heading (cap en degrés)
- Commandes : Armer, Désarmer, Retour Maison

L'interface est non-bloquante et gère les connexions indisponibles.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import math
import signal
from datetime import datetime
import json
import os
from utils.bblib import MavlinkLink, IMU, GPS, MotorDriver, Navigation

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "boat_control_config.json")


class HeartbeatManager:
    """Gestionnaire de heartbeat pour maintenir les connexions"""
    
    def __init__(self, targets, hz=1.0, port=6040, compid=1, timeout=2.0):
        self.targets = targets  # Liste de (ip, sysid) ou (ip, sysid, port)
        self.hz = hz
        self.port = port
        self.compid = compid
        self.timeout = timeout
        
        self.links = []
        self.running = False
        self.thread = None
        
        # Créer les liens MAVLink
        for target in targets:
            if len(target) == 3:
                ip, sysid, tgt_port = target
            else:
                ip, sysid = target
                tgt_port = port
            link = MavlinkLink(host=ip, port=tgt_port, sysid=sysid, compid=compid, timeout=timeout, silent_errors=True)
            self.links.append(link)
    
    def build_heartbeat_message(self):
        """Construction du message HEARTBEAT"""
        return {
            "type": "HEARTBEAT",
            "custom_mode": 0,
            "mavtype": {"type": "MAV_TYPE_GCS"},
            "autopilot": {"type": "MAV_AUTOPILOT_INVALID"},
            "base_mode": {"bits": 0},
            "system_status": {"type": "MAV_STATE_ACTIVE"},
            "mavlink_version": 2
        }
    
    def start(self):
        """Démarre l'envoi de heartbeats"""
        if self.running:
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.thread.start()
        print(f"[HEARTBEAT] Démarrage heartbeat à {self.hz:.1f} Hz vers {len(self.targets)} cibles")
    
    def stop(self):
        """Arrête l'envoi de heartbeats"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        print("[HEARTBEAT] Arrêt du heartbeat")
    
    def _heartbeat_loop(self):
        """Boucle d'envoi des heartbeats"""
        period = 1.0 / max(0.2, self.hz)
        hb = self.build_heartbeat_message()
        
        count = 0
        while self.running:
            t0 = time.time()
            
            # Envoyer à toutes les cibles
            for link in self.links:
                if not self.running:
                    break
                try:
                    link.post_message(hb)
                except Exception as e:
                    # Ne pas spammer les erreurs
                    if count % 10 == 0:
                        print(f"[HEARTBEAT] Erreur vers {link.base}: {e}")
            
            count += 1
            
            # Maintenir la cadence
            dt = time.time() - t0
            sleep_time = max(0.0, period - dt)
            if sleep_time > 0:
                time.sleep(sleep_time)


class BoatMonitor:
    """Classe pour surveiller un bateau individuel"""
    
    def __init__(self, boat_id, host, sysid=None, port=6040):
        self.boat_id = boat_id
        self.host = host
        self.sysid = sysid if sysid else boat_id
        self.port = port
        
        # État de connexion
        self.connected = False
        self.last_update = None
        
        # Données du bateau
        self.gps_data = {"lat": None, "lon": None, "alt": None, "hdg": None}
        self.battery_data = {"voltage": None, "percentage": None, "current": None}
        self.heading = None
        self.mode_data = {"mode": None, "armed": None}
        
        # Objets MAVLink
        self.mav = None
        self.gps = None
        self.imu = None
        self.motors = None
        self.navigation = None
        
        # Thread de monitoring
        self.monitoring = False
        self.monitor_thread = None
        
        # Thread de retour maison
        self.home_active = False
        self.home_thread = None
        self.stop_home = False
    
    def calculate_battery_percentage_4s(self, voltage):
        """
        Calcule le pourcentage de batterie pour une batterie LiPo 4S
        basé sur la tension totale
        
        Tension 4S LiPo:
        - Pleine charge: 16.8V (4.2V par cellule)
        - Nominale: 14.8V (3.7V par cellule)  
        - Décharge sécuritaire: 14.0V (3.5V par cellule)
        - Critique: 13.2V (3.3V par cellule)
        """
        if voltage is None:
            return None
            
        # Tensions de référence pour batterie 4S
        V_MAX = 16.8  # Tension maximale (4.2V × 4)
        V_NOM = 14.8  # Tension nominale (3.7V × 4)  
        V_MIN = 13.2  # Tension minimale sécuritaire (3.3V × 4)
        
        # Limiter la tension dans la plage valide
        voltage = max(min(voltage, V_MAX), V_MIN)
        
        # Calcul du pourcentage avec courbe non-linéaire approximative
        if voltage >= V_NOM:
            # Entre nominal et max (courbe plus douce dans cette zone)
            percentage = 50 + 50 * (voltage - V_NOM) / (V_MAX - V_NOM)
        else:
            # Entre min et nominal (courbe plus raide)
            percentage = 50 * (voltage - V_MIN) / (V_NOM - V_MIN)
        
        return max(0, min(100, percentage))
    
    def connect(self):
        """Tente de se connecter au bateau"""
        try:
            self.mav = MavlinkLink(host=self.host, port=self.port, sysid=self.sysid, timeout=2.0)
            self.gps = GPS(self.mav)
            self.imu = IMU(self.mav)
            self.motors = MotorDriver(self.mav)
            self.navigation = Navigation(self.imu, self.gps, self.motors)
            self.nav = Navigation(self.imu, self.gps, self.motors)
            
            # Test de connexion
            test_msg = self.mav.get_message("SYS_STATUS")
            if test_msg:
                self.connected = True
                print(f"Bateau {self.boat_id}: Connexion établie ({self.host}:{self.sysid})")
                return True
            else:
                self.connected = False
                return False
                
        except Exception as e:
            self.connected = False
            print(f"Bateau {self.boat_id}: Erreur de connexion - {e}")
            return False
    
    def disconnect(self):
        """Déconnecte le bateau"""
        self.connected = False
        self.stop_home_navigation()  # Arrêter le retour maison si en cours
        self.mav = None
        self.gps = None
        self.imu = None
        self.motors = None
        self.navigation = None
    
    def update_data(self):
        """Met à jour les données du bateau"""
        if not self.connected or not self.mav:
            return False
            
        try:
            # GPS
            gps_info = self.gps.get_gps_dict()
            if gps_info:
                self.gps_data.update(gps_info)
            
            # Batterie
            battery_info = self.mav.get_battery_status()
            if battery_info:
                voltage = battery_info.get("voltage_V")
                self.battery_data["voltage"] = voltage
                # Calculer le pourcentage basé sur la tension 4S au lieu d'utiliser celui retourné
                self.battery_data["percentage"] = self.calculate_battery_percentage_4s(voltage)
                self.battery_data["current"] = battery_info.get("current_A")
            
            # Heading depuis IMU
            attitude = self.imu.get_attitude_dict()
            if attitude:
                self.heading = attitude["yaw_deg"]
            
            # Mode de vol et état d'armement
            mode_info = self.mav.get_flight_mode()
            if mode_info:
                self.mode_data["mode"] = mode_info["mode"]
                self.mode_data["armed"] = mode_info["armed"]
            
            self.last_update = datetime.now()
            return True
            
        except Exception as e:
            print(f"Bateau {self.boat_id}: Erreur lors de la mise à jour - {e}")
            self.connected = False
            return False
    
    def arm(self):
        """Arme le bateau"""
        if self.connected and self.mav:
            try:
                result = self.mav.arm_disarm(arm=True)
                return result is not None
            except Exception as e:
                print(f"Bateau {self.boat_id}: Erreur arm - {e}")
                return False
        return False
    
    def disarm(self):
        """Désarme le bateau"""
        if self.connected and self.mav:
            try:
                result = self.mav.arm_disarm(arm=False)
                return result is not None
            except Exception as e:
                print(f"Bateau {self.boat_id}: Erreur disarm - {e}")
                return False
        return False
    
    def return_to_launch(self):
        """Lance le retour maison avec la navigation personnalisée"""
        if not self.connected or not self.navigation:
            return False
            
        if self.home_active:
            print(f"Bateau {self.boat_id}: Retour maison déjà en cours")
            return False
            
        try:
            self.home_active = True
            self.stop_home = False
            self.home_thread = threading.Thread(target=self._home_worker, daemon=True)
            self.home_thread.start()
            return True
        except Exception as e:
            print(f"Bateau {self.boat_id}: Erreur lancement retour maison - {e}")
            self.home_active = False
            return False
    
    def _home_worker(self):
        """Thread worker pour le retour maison (utilise Navigation, pas RTL MAVLink)"""
        try:
            print(f"Bateau {self.boat_id}: Début du retour maison via Navigation")
            
            # Armer le bateau (comme dans test_bblib)
            if not self.stop_home and self.mav:
                print(f"Bateau {self.boat_id}: Armement...")
                result = self.mav.arm_disarm(True)
                print(f"Bateau {self.boat_id}: Armement result: {result}")
                time.sleep(1)
            
            # Exécuter le retour maison interruptible
            if not self.stop_home and self.navigation:
                print(f"Bateau {self.boat_id}: Lancement retour maison interruptible...")
                self.interruptible_return_home()
                
        except Exception as e:
            print(f"Bateau {self.boat_id}: Erreur durant retour maison - {e}")
        finally:
            self.home_active = False
            self.stop_home = False
            if self.motors:
                self.motors.stop_motors()  # S'assurer que les moteurs s'arrêtent
            print(f"Bateau {self.boat_id}: Retour maison terminé")
    
    def interruptible_return_home(self):
        """
        Version interruptible de Navigation.return_home()
        Copie modifiée qui vérifie self.stop_home régulièrement
        """
        # Points de retour maison (même que dans Navigation.return_home())
        waypoints = [
            (48.1990856, -3.0155828, 6),   # (lat, lon, distance_seuil)
            (48.1990483, -3.014815, 10)
        ]
        
        for i, (lat, lon, distance_threshold) in enumerate(waypoints):
            if self.stop_home:
                print(f"Bateau {self.boat_id}: Retour maison interrompu au waypoint {i+1}")
                break
                
            print(f"Bateau {self.boat_id}: Waypoint {i+1}/{len(waypoints)} -> ({lat:.7f}, {lon:.7f})")
            
            # Aller vers le waypoint avec vérification d'interruption
            success = self.interruptible_go_to_gps((lat, lon), distance_threshold)
            
            if not success:
                print(f"Bateau {self.boat_id}: Échec ou interruption waypoint {i+1}")
                break
                
        if not self.stop_home:
            print(f"Bateau {self.boat_id}: Retour maison terminé avec succès")
    
    def interruptible_go_to_gps(self, target_gps, distance_threshold=5.0):
        """
        Version interruptible de Navigation.go_to_gps()
        Retourne True si arrivé à destination, False si interrompu
        """
        import utils.geo_conversion as geo
        import numpy as np
        import math
        
        # Convertir la cible GPS en coordonnées cartésiennes
        target_cartesian = geo.conversion_spherique_cartesien(target_gps)
        target = np.array(target_cartesian, dtype=float).reshape(2, 1)
        
        print(f"Bateau {self.boat_id}: Navigation vers {target_gps} (seuil: {distance_threshold}m)")
        
        max_iterations = 1000  # Éviter les boucles infinies
        iteration = 0
        
        while not self.stop_home and iteration < max_iterations:
            iteration += 1
            
            # Vérifier la position actuelle
            current_coords = np.array(self.gps.get_coords(), dtype=float).reshape(2, 1)
            
            if current_coords[0,0] is None or current_coords[1,0] is None:
                print(f"Bateau {self.boat_id}: Position GPS non disponible")
                time.sleep(0.5)
                continue
                
            # Calculer la distance à la cible
            delta = target - current_coords
            distance_target = float(np.linalg.norm(delta))
            
            # Vérifier si on est arrivé
            if distance_target < distance_threshold:
                print(f"Bateau {self.boat_id}: Waypoint atteint (distance: {distance_target:.1f}m)")
                return True
                
            # Calculer le cap cible
            target_heading = (math.degrees(math.atan2(delta[1], delta[0])) + 360.0) % 360.0
            
            # Obtenir le cap actuel
            current_heading = self.navigation.get_current_heading()
            if current_heading is None:
                print(f"Bateau {self.boat_id}: Cap actuel non disponible")
                time.sleep(0.5)
                continue
                
            # Calculer l'erreur de cap
            error = target_heading - current_heading
            error = (error + 180) % 360 - 180
            correction = self.navigation.Kp * error
            
            # Calculer la correction de distance
            reference_distance = 2.0
            distance_correction = math.tanh(distance_target / reference_distance)
            
            # Calculer les commandes moteur
            left_motor = distance_correction * self.navigation.base_speed + correction
            right_motor = distance_correction * self.navigation.base_speed - correction
            
            # Limiter les commandes
            left_motor = np.clip(left_motor, -self.navigation.max_cmd, self.navigation.max_cmd)
            right_motor = np.clip(right_motor, -self.navigation.max_cmd, self.navigation.max_cmd)
            
            # Envoyer les commandes moteur
            if not self.stop_home:
                self.motors.send_cmd_motor(left_motor, right_motor)
                
            # Affichage de debug (moins fréquent pour éviter le spam)
            if iteration % 10 == 0:
                print(f"Bateau {self.boat_id}: Dist={distance_target:.1f}m, Cap={current_heading:.1f}°→{target_heading:.1f}°, Err={error:.1f}°")
            
            # Petite pause pour ne pas surcharger
            time.sleep(0.1)
            
        # Si on arrive ici, soit stop_home=True, soit max_iterations atteint
        if self.stop_home:
            print(f"Bateau {self.boat_id}: Navigation interrompue par demande d'arrêt")
            return False
        else:
            print(f"Bateau {self.boat_id}: Navigation interrompue (timeout)")
            return False
    
    def stop_home_navigation(self):
        """Arrête le retour maison en cours"""
        if self.home_active:
            print(f"Bateau {self.boat_id}: Arrêt du retour maison demandé")
            self.stop_home = True
            if self.motors:
                self.motors.stop_motors()
            # Attendre que le thread se termine
            if self.home_thread and self.home_thread.is_alive():
                self.home_thread.join(timeout=2)
            self.home_active = False


class BoatControlGUI:
    """Interface graphique principale"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Contrôle des Bateaux Scout")
        self.root.geometry("1000x650")
        self.root.configure(bg="#f0f0f0")
        
        # Configuration des bateaux (à adapter selon votre réseau)
        self.config = self.load_config()
        self.boats = {
            item["boat_id"]: BoatMonitor(
                item["boat_id"],
                item["host"],
                item.get("sysid", item["boat_id"]),
                item.get("port", 6040),
            )
            for item in self.config
        }
        
        # Variables pour l'interface
        self.monitoring_active = False
        self.update_thread = None
        
        # Gestionnaire de heartbeat
        targets = [(boat.host, boat.sysid, boat.port) for boat in self.boats.values()]
        self.heartbeat_manager = HeartbeatManager(targets, hz=1.0)
        
        self.setup_gui()
        self.start_monitoring()
        self.start_heartbeat()
    
    def start_heartbeat(self):
        """Démarre le heartbeat"""
        self.heartbeat_manager.start()
        self.heartbeat_status.config(text="Heartbeat: ON", fg="#27ae60")
        
    def setup_gui(self):
        """Configure l'interface graphique"""
        
        # Titre principal
        title_frame = tk.Frame(self.root, bg="#2c3e50", height=60)
        title_frame.pack(fill="x", padx=5, pady=5)
        title_frame.pack_propagate(False)
        
        title_label = tk.Label(title_frame, text="Contrôle des Bateaux Scout", 
                              font=("Arial", 18, "bold"), fg="white", bg="#2c3e50")
        title_label.pack(expand=True)
        
        # Frame principal pour les bateaux
        boats_frame = tk.Frame(self.root, bg="#f0f0f0")
        boats_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Créer une interface pour chaque bateau
        for i, (boat_id, boat) in enumerate(self.boats.items()):
            self.create_boat_panel(boats_frame, boat, i)
        
        # Frame pour les boutons généraux
        control_frame = tk.Frame(self.root, bg="#f0f0f0")
        control_frame.pack(fill="x", padx=10, pady=5)
        
        refresh_btn = tk.Button(control_frame, text="Actualiser", 
                               command=self.refresh_connections,
                               font=("Arial", 12), bg="#3498db", fg="black",
                               padx=20, pady=5)
        refresh_btn.pack(side="left", padx=5)

        config_btn = tk.Button(control_frame, text="Modifier config",
                      command=self.open_config_dialog,
                      font=("Arial", 12), bg="#95a5a6", fg="black",
                      padx=20, pady=5)
        config_btn.pack(side="left", padx=5)
        
        # Bouton d'arrêt d'urgence
        emergency_btn = tk.Button(control_frame, text="ARRÊT URGENCE", 
                                 command=self.emergency_stop,
                                 font=("Arial", 12, "bold"), bg="#c0392b", fg="black",
                                 padx=20, pady=5)
        emergency_btn.pack(side="left", padx=10)
        
        # Status heartbeat
        self.heartbeat_status = tk.Label(control_frame, text="Heartbeat: OFF", 
                                        font=("Arial", 10), fg="#e74c3c")
        self.heartbeat_status.pack(side="left", padx=10)
        
        quit_btn = tk.Button(control_frame, text="Quitter", 
                           command=self.quit_app,
                           font=("Arial", 12), bg="#e74c3c", fg="black",
                           padx=20, pady=5)
        quit_btn.pack(side="right", padx=5)
        
    def create_boat_panel(self, parent, boat, column):
        """Crée le panneau d'un bateau"""
        
        # Frame principal du bateau
        boat_frame = tk.LabelFrame(parent, text=f"Bateau {boat.boat_id}", 
                                  font=("Arial", 14, "bold"), 
                                  bg="#ecf0f1", fg="#2c3e50", 
                                  relief="groove", bd=2)
        boat_frame.grid(row=0, column=column, padx=10, pady=5, sticky="nsew")
        parent.grid_columnconfigure(column, weight=1)
        
        # Indicateur de connexion
        conn_frame = tk.Frame(boat_frame, bg="#ecf0f1")
        conn_frame.pack(fill="x", padx=5, pady=5)
        
        boat.status_label = tk.Label(conn_frame, text="🔴 Déconnecté", 
                                    font=("Arial", 12, "bold"), 
                                    fg="#e74c3c", bg="#ecf0f1")
        boat.status_label.pack()
        
        # Informations GPS
        gps_frame = tk.LabelFrame(boat_frame, text="Position GPS", 
                                 font=("Arial", 10, "bold"), bg="#ecf0f1")
        gps_frame.pack(fill="x", padx=5, pady=5)
        
        boat.lat_label = tk.Label(gps_frame, text="Lat: --", 
                                 font=("Arial", 10), bg="#ecf0f1")
        boat.lat_label.pack(anchor="w")
        
        boat.lon_label = tk.Label(gps_frame, text="Lon: --", 
                                 font=("Arial", 10), bg="#ecf0f1")
        boat.lon_label.pack(anchor="w")
        
        boat.alt_label = tk.Label(gps_frame, text="Alt: --", 
                                 font=("Arial", 10), bg="#ecf0f1")
        boat.alt_label.pack(anchor="w")
        
        # Informations batterie
        battery_frame = tk.LabelFrame(boat_frame, text="Batterie", 
                                     font=("Arial", 10, "bold"), bg="#ecf0f1")
        battery_frame.pack(fill="x", padx=5, pady=5)
        
        boat.voltage_label = tk.Label(battery_frame, text="Tension: --", 
                                     font=("Arial", 10), bg="#ecf0f1")
        boat.voltage_label.pack(anchor="w")
        
        boat.percentage_label = tk.Label(battery_frame, text="Niveau: --", 
                                        font=("Arial", 10), bg="#ecf0f1")
        boat.percentage_label.pack(anchor="w")
        
        # État du véhicule (mode et armement)
        status_frame = tk.LabelFrame(boat_frame, text="État", 
                                    font=("Arial", 10, "bold"), bg="#ecf0f1")
        status_frame.pack(fill="x", padx=5, pady=5)
        
        boat.mode_label = tk.Label(status_frame, text="Mode: --", 
                                  font=("Arial", 10), bg="#ecf0f1")
        boat.mode_label.pack(anchor="w")
        
        boat.armed_label = tk.Label(status_frame, text="Armé: --", 
                                   font=("Arial", 10), bg="#ecf0f1")
        boat.armed_label.pack(anchor="w")
        
        # Heading
        heading_frame = tk.LabelFrame(boat_frame, text="Cap", 
                                     font=("Arial", 10, "bold"), bg="#ecf0f1")
        heading_frame.pack(fill="x", padx=5, pady=5)
        
        boat.heading_label = tk.Label(heading_frame, text="Cap: --", 
                                     font=("Arial", 10), bg="#ecf0f1")
        boat.heading_label.pack(anchor="w")
        
        # Boutons de contrôle
        controls_frame = tk.Frame(boat_frame, bg="#ecf0f1")
        controls_frame.pack(fill="x", padx=5, pady=10)

        row1 = tk.Frame(controls_frame, bg="#ecf0f1")
        row1.pack(fill="x", pady=(0, 4))

        row2 = tk.Frame(controls_frame, bg="#ecf0f1")
        row2.pack(fill="x")

        row3 = tk.Frame(controls_frame, bg="#ecf0f1")
        row3.pack(fill="x", pady=(4, 0))

        row4 = tk.Frame(controls_frame, bg="#ecf0f1")
        row4.pack(fill="x", pady=(4, 0))

        arm_btn = tk.Button(row1, text="Armer", 
                   command=lambda: self.arm_boat(boat.boat_id),
                   font=("Arial", 9), bg="#27ae60", fg="black")
        arm_btn.pack(side="left", padx=2, fill="x", expand=True)

        disarm_btn = tk.Button(row1, text="Désarmer", 
                      command=lambda: self.disarm_boat(boat.boat_id),
                      font=("Arial", 9), bg="#e67e22", fg="black")
        disarm_btn.pack(side="left", padx=2, fill="x", expand=True)

        home_btn = tk.Button(row2, text="Retour Maison", 
                     command=lambda: self.return_home_boat(boat.boat_id),
                     font=("Arial", 8), bg="#9b59b6", fg="black")
        home_btn.pack(side="left", padx=2, fill="x", expand=True)

        stop_btn = tk.Button(row2, text="Stop Retour Maison", 
                   command=lambda: self.stop_boat(boat.boat_id),
                   font=("Arial", 9), bg="#e74c3c", fg="black")
        stop_btn.pack(side="left", padx=2, fill="x", expand=True)

        manual_btn = tk.Button(row3, text="Manual", 
                 command=lambda: self.set_mode_boat(boat.boat_id, "MANUAL"),
                 font=("Arial", 9), bg="#7f8c8d", fg="black")
        manual_btn.pack(side="left", padx=2, fill="x", expand=True)

        hold_btn = tk.Button(row3, text="Hold", 
               command=lambda: self.set_mode_boat(boat.boat_id, "HOLD"),
               font=("Arial", 9), bg="#95a5a6", fg="black")
        hold_btn.pack(side="left", padx=2, fill="x", expand=True)

        test_btn = tk.Button(row4, text="Test motors", 
            command=lambda: self.open_motor_test_window(boat.boat_id),
            font=("Arial", 9), bg="#bdc3c7", fg="black")
        test_btn.pack(side="left", padx=2, fill="x", expand=True)
        
        # Indicateur d'état retour maison
        boat.home_status_label = tk.Label(controls_frame, text="", 
                                         font=("Arial", 8), bg="#ecf0f1", fg="#e67e22")
        boat.home_status_label.pack(side="bottom", pady=2)
        
    def start_monitoring(self):
        """Démarre le monitoring des bateaux"""
        self.monitoring_active = True
        self.update_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        self.update_thread.start()
        
        # Planifier les mises à jour de l'interface
        self.schedule_gui_update()
    
    def monitor_loop(self):
        """Boucle de monitoring des bateaux"""
        while self.monitoring_active:
            for boat in self.boats.values():
                if not boat.connected:
                    # Tenter de se reconnecter
                    boat.connect()
                
                if boat.connected:
                    # Mettre à jour les données
                    success = boat.update_data()
                    if not success:
                        boat.disconnect()
                
            time.sleep(2)  # Mise à jour toutes les 2 secondes
    
    def schedule_gui_update(self):
        """Planifie la mise à jour de l'interface"""
        if self.monitoring_active:
            self.update_gui()
            self.root.after(1100, self.schedule_gui_update)  # Toutes les secondes
    
    def update_gui(self):
        """Met à jour l'affichage de l'interface"""
        for boat in self.boats.values():
            # Statut de connexion
            if boat.connected:
                boat.status_label.config(text="🟢 Connecté", fg="#27ae60")
            else:
                boat.status_label.config(text="🔴 Déconnecté", fg="#e74c3c")
            
            # GPS
            if boat.gps_data["lat"] is not None:
                boat.lat_label.config(text=f"Lat: {boat.gps_data['lat']:.6f}°")
            else:
                boat.lat_label.config(text="Lat: --")
            
            if boat.gps_data["lon"] is not None:
                boat.lon_label.config(text=f"Lon: {boat.gps_data['lon']:.6f}°")
            else:
                boat.lon_label.config(text="Lon: --")
            
            if boat.gps_data["alt"] is not None:
                boat.alt_label.config(text=f"Alt: {boat.gps_data['alt']:.1f} m")
            else:
                boat.alt_label.config(text="Alt: --")
            
            # Batterie
            if boat.battery_data["voltage"] is not None:
                boat.voltage_label.config(text=f"Tension: {boat.battery_data['voltage']:.2f} V")
            else:
                boat.voltage_label.config(text="Tension: --")
            
            if boat.battery_data["percentage"] is not None:
                percentage = boat.battery_data['percentage']
                voltage = boat.battery_data['voltage']
                
                # Déterminer la couleur selon le niveau
                if percentage >= 60:
                    color = "#27ae5f"  # Vert
                elif percentage >= 30:
                    color = "#f39c12"  # Orange
                else:
                    color = "#e74c3c"  # Rouge
                
                # Affichage avec tension et pourcentage calculé (4S)
                boat.percentage_label.config(
                    text=f"Niveau: {percentage:.0f}% (4S: {voltage:.2f}V)", 
                    fg=color
                )
            else:
                boat.percentage_label.config(text="Niveau: --", fg="#7f8c8d")
            
            # État du véhicule (mode et armement)
            if boat.mode_data["mode"] is not None:
                boat.mode_label.config(text=f"Mode: {boat.mode_data['mode']}")
            else:
                boat.mode_label.config(text="Mode: --")
            
            if boat.mode_data["armed"] is not None:
                armed_status = "OUI" if boat.mode_data["armed"] else "NON"
                color = "#27ae60" if boat.mode_data["armed"] else "#e74c3c"
                boat.armed_label.config(text=f"Armé: {armed_status}", fg=color)
            else:
                boat.armed_label.config(text="Armé: --", fg="#7f8c8d")
            
            # Heading
            if boat.heading is not None:
                boat.heading_label.config(text=f"Cap: {boat.heading:.1f}°")
            else:
                boat.heading_label.config(text="Cap: --")
            
            # Status retour maison
            if boat.home_active:
                boat.home_status_label.config(text="Retour maison...", fg="#f39c12")
            else:
                boat.home_status_label.config(text="", fg="#2c3e50")
    
    def arm_boat(self, boat_id):
        """Arme un bateau"""
        boat = self.boats.get(boat_id)
        if boat and boat.connected:
            success = boat.arm()
            if success:
                messagebox.showinfo("Succès", f"Bateau {boat_id} armé")
            else:
                messagebox.showerror("Erreur", f"Impossible d'armer le bateau {boat_id}")
        else:
            messagebox.showerror("Erreur", f"Bateau {boat_id} non connecté")
    
    def disarm_boat(self, boat_id):
        """Désarme un bateau"""
        boat = self.boats.get(boat_id)
        if boat and boat.connected:
            success = boat.disarm()
            if success:
                messagebox.showinfo("Succès", f"Bateau {boat_id} désarmé")
            else:
                messagebox.showerror("Erreur", f"Impossible de désarmer le bateau {boat_id}")
        else:
            messagebox.showerror("Erreur", f"Bateau {boat_id} non connecté")
    
    def return_home_boat(self, boat_id):
        """Commande retour maison pour un bateau (via Navigation, pas RTL MAVLink)"""
        boat = self.boats.get(boat_id)
        if boat and boat.connected:
            confirm = messagebox.askyesno("Confirmation", 
                                        f"Lancer le retour maison du bateau {boat_id} ?\n\n"
                                        f"Note: Utilise Navigation.return_home(), pas le mode RTL MAVLink")
            if confirm:
                success = boat.return_to_launch()
                if success:
                    messagebox.showinfo("Succès", f"Retour maison lancé pour le bateau {boat_id}")
                else:
                    messagebox.showerror("Erreur", f"Impossible de lancer le retour maison du bateau {boat_id}")
        else:
            messagebox.showerror("Erreur", f"Bateau {boat_id} non connecté")
    
    def stop_boat(self, boat_id):
        """Arrête toutes les actions du bateau"""
        boat = self.boats.get(boat_id)
        if boat:
            # Arrêter le retour maison si en cours
            boat.stop_home_navigation()
            
            # Arrêter les moteurs
            if boat.motors:
                boat.motors.stop_motors()
                
            messagebox.showinfo("Info", f"Bateau {boat_id} arrêté")
        else:
            messagebox.showerror("Erreur", f"Bateau {boat_id} non trouvé")

    def set_mode_boat(self, boat_id, mode_name):
        """Change le mode de vol d'un bateau"""
        boat = self.boats.get(boat_id)
        if boat and boat.connected and boat.mav:
            result = boat.mav.set_flight_mode(mode_name=mode_name)
            if result is None:
                messagebox.showerror("Erreur", f"Impossible de changer le mode du bateau {boat_id}")
            else:
                messagebox.showinfo("Info", f"Mode {mode_name} demandé pour le bateau {boat_id}")
        else:
            messagebox.showerror("Erreur", f"Bateau {boat_id} non connecté")

    def open_motor_test_window(self, boat_id):
        """Fenêtre de test moteurs avec arm/disarm et sliders."""
        boat = self.boats.get(boat_id)
        if not boat or not boat.connected:
            messagebox.showerror("Erreur", f"Bateau {boat_id} non connecté")
            return

        dialog = tk.Toplevel(self.root)
        dialog.title(f"Test moteurs - Bateau {boat_id}")
        dialog.geometry("360x260")
        dialog.resizable(False, False)

        armed_var = tk.BooleanVar(value=bool(boat.mode_data.get("armed")))

        def sync_armed():
            if not dialog.winfo_exists():
                return
            if boat.mode_data.get("armed") is not None:
                armed_var.set(bool(boat.mode_data.get("armed")))
            dialog.after(500, sync_armed)

        def toggle_arm():
            if armed_var.get():
                boat.arm()
            else:
                boat.disarm()

        armed_chk = tk.Checkbutton(dialog, text="Armé", variable=armed_var, command=toggle_arm)
        armed_chk.pack(anchor="w", padx=10, pady=10)

        left_scale = tk.Scale(dialog, from_=-250, to=250, orient="horizontal", label="Moteur gauche")
        right_scale = tk.Scale(dialog, from_=-250, to=250, orient="horizontal", label="Moteur droit")
        left_scale.pack(fill="x", padx=10)
        right_scale.pack(fill="x", padx=10)

        auto_running = {"active": True}

        def send_cmd():
            if not boat.motors:
                return
            left = float(left_scale.get())
            right = float(right_scale.get())

            def worker():
                boat.motors.drive_lr(left, right, seconds=0.2, rate_hz=10.0)

            threading.Thread(target=worker, daemon=True).start()

        def auto_loop():
            if not dialog.winfo_exists() or not auto_running["active"]:
                return
            if armed_var.get():
                send_cmd()
            dialog.after(1000, auto_loop)

        auto_loop()

        def on_close():
            auto_running["active"] = False
            if boat.motors:
                boat.motors.stop_motors()
            dialog.destroy()

        dialog.protocol("WM_DELETE_WINDOW", on_close)
        sync_armed()
    
    def emergency_stop(self):
        """Arrêt d'urgence de tous les bateaux"""
        confirm = messagebox.askyesno("ARRÊT D'URGENCE", 
                        "Arrêter TOUS les bateaux immédiatement ?",
                                    icon="warning")
        if confirm:
            for boat_id in self.boats.keys():
                self.stop_boat(boat_id)
            messagebox.showinfo("ARRÊT D'URGENCE", "Tous les bateaux ont été arrêtés")
    
    def refresh_connections(self):
        """Force la reconnexion aux bateaux"""
        for boat in self.boats.values():
            boat.disconnect()
        messagebox.showinfo("Info", "Reconnexion en cours...")

    def default_config(self):
        return [
            {"boat_id": 1, "host": "192.168.2.201", "port": 6040, "sysid": 1},
            {"boat_id": 2, "host": "192.168.2.202", "port": 6040, "sysid": 2},
            {"boat_id": 3, "host": "192.168.2.203", "port": 6040, "sysid": 3},
        ]

    def load_config(self):
        if not os.path.exists(CONFIG_PATH):
            return self.default_config()
        try:
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, list) or not data:
                return self.default_config()
            cleaned = []
            for item in data:
                if not isinstance(item, dict) or "boat_id" not in item or "host" not in item:
                    continue
                cleaned.append({
                    "boat_id": int(item["boat_id"]),
                    "host": str(item["host"]),
                    "port": int(item.get("port", 6040)),
                    "sysid": int(item.get("sysid", item["boat_id"])),
                })
            return cleaned if cleaned else self.default_config()
        except Exception:
            return self.default_config()

    def save_config(self):
        data = []
        for boat_id, boat in self.boats.items():
            data.append({
                "boat_id": boat_id,
                "host": boat.host,
                "port": boat.port,
                "sysid": boat.sysid,
            })
        try:
            with open(CONFIG_PATH, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            messagebox.showerror("Erreur", f"Impossible d'enregistrer la config: {e}")

    def open_config_dialog(self):
        """Ouvre une fenêtre pour modifier IP/port"""
        dialog = tk.Toplevel(self.root)
        dialog.title("Modifier la configuration")
        dialog.geometry("600x260")
        dialog.resizable(False, False)

        entries = {}
        test_buttons = {}

        header = tk.Label(dialog, text="Configuration des bateaux", font=("Arial", 12, "bold"))
        header.grid(row=0, column=0, columnspan=3, pady=(10, 10))

        tk.Label(dialog, text="Bateau").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        tk.Label(dialog, text="IP").grid(row=1, column=1, padx=5, pady=5, sticky="w")
        tk.Label(dialog, text="Port").grid(row=1, column=2, padx=5, pady=5, sticky="w")
        tk.Label(dialog, text="SysID").grid(row=1, column=3, padx=5, pady=5, sticky="w")
        tk.Label(dialog, text="Test").grid(row=1, column=4, padx=5, pady=5, sticky="w")

        for i, boat_id in enumerate(sorted(self.boats.keys()), start=2):
            boat = self.boats[boat_id]
            tk.Label(dialog, text=f"{boat_id}").grid(row=i, column=0, padx=5, pady=5, sticky="w")

            ip_var = tk.StringVar(value=boat.host)
            port_var = tk.StringVar(value=str(boat.port))
            sysid_var = tk.StringVar(value=str(boat.sysid))

            ip_entry = tk.Entry(dialog, textvariable=ip_var, width=20)
            port_entry = tk.Entry(dialog, textvariable=port_var, width=8)
            sysid_entry = tk.Entry(dialog, textvariable=sysid_var, width=8)

            ip_entry.grid(row=i, column=1, padx=5, pady=5, sticky="w")
            port_entry.grid(row=i, column=2, padx=5, pady=5, sticky="w")
            sysid_entry.grid(row=i, column=3, padx=5, pady=5, sticky="w")

            entries[boat_id] = (ip_var, port_var, sysid_var)

            test_btn = tk.Button(dialog, text="Test", width=6, bg="#95a5a6", fg="black")
            test_btn.grid(row=i, column=4, padx=5, pady=5, sticky="w")
            test_buttons[boat_id] = test_btn

        def _test_connection(ip, port, sysid):
            try:
                link = MavlinkLink(host=ip, port=port, sysid=sysid, timeout=1.5)
                msg = link.get_message("SYS_STATUS")
                return msg is not None
            except Exception:
                return False

        def _trigger_test(boat_id):
            ip_var, port_var, sysid_var = entries[boat_id]
            btn = test_buttons[boat_id]
            try:
                ip = ip_var.get().strip()
                port = int(port_var.get().strip())
                sysid = int(sysid_var.get().strip())
            except Exception:
                btn.config(bg="#e74c3c", text="KO")
                return

            btn.config(state="disabled", text="...")

            def worker():
                ok = _test_connection(ip, port, sysid)

                def update_btn():
                    btn.config(state="normal")
                    if ok:
                        btn.config(bg="#27ae60", text="OK")
                    else:
                        btn.config(bg="#e74c3c", text="KO")

                dialog.after(0, update_btn)

            threading.Thread(target=worker, daemon=True).start()

        for boat_id, btn in test_buttons.items():
            btn.configure(command=lambda bid=boat_id: _trigger_test(bid))

        def apply_changes():
            try:
                for boat_id, (ip_var, port_var, sysid_var) in entries.items():
                    ip = ip_var.get().strip()
                    port = int(port_var.get().strip())
                    sysid = int(sysid_var.get().strip())
                    if not ip:
                        raise ValueError(f"IP vide pour le bateau {boat_id}")
                    if port <= 0 or port > 65535:
                        raise ValueError(f"Port invalide pour le bateau {boat_id}")
                    if sysid <= 0 or sysid > 255:
                        raise ValueError(f"SysID invalide pour le bateau {boat_id}")

                    boat = self.boats[boat_id]
                    boat.host = ip
                    boat.port = port
                    boat.sysid = sysid
                    boat.disconnect()

                self.save_config()

                # Recréer le heartbeat avec la nouvelle configuration
                self.heartbeat_manager.stop()
                targets = [(boat.host, boat.sysid, boat.port) for boat in self.boats.values()]
                self.heartbeat_manager = HeartbeatManager(targets, hz=1.0)
                self.heartbeat_manager.start()

                messagebox.showinfo("Info", "Configuration mise à jour")
                dialog.destroy()
            except Exception as e:
                messagebox.showerror("Erreur", f"Configuration invalide: {e}")

        btn_frame = tk.Frame(dialog)
        btn_frame.grid(row=6, column=0, columnspan=5, pady=(15, 10))

        tk.Button(btn_frame, text="Annuler", command=dialog.destroy).pack(side="right", padx=5)
        tk.Button(btn_frame, text="Appliquer", command=apply_changes).pack(side="right", padx=5)
    
    def quit_app(self):
        """Quitte l'application"""
        self.monitoring_active = False
        
        # Arrêter le heartbeat
        self.heartbeat_manager.stop()
        
        # Arrêter tous les retours maison en cours et déconnecter les bateaux
        for boat in self.boats.values():
            boat.stop_home_navigation()
            boat.disconnect()
        
        self.root.quit()
        self.root.destroy()
    
    def run(self):
        """Lance l'interface"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.quit_app()


if __name__ == "__main__":
    print("Démarrage de l'interface de contrôle des bateaux Scout...")
    
    # Lancer l'interface
    app = BoatControlGUI()
    app.run()