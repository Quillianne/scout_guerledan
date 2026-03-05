#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_multiple_boat.py

Script de test pour vérifier le fonctionnement simultané de deux BlueBoat
sur des réseaux séparés (192.168.2.202 et 192.168.2.203).

- Le bateau 1 (sysid=2) suit un cap fixe pendant 5 s.
- Le bateau 2 (sysid=3) poursuit le bateau 1 en temps réel,
  sans utiliser la méthode follow_gps de Navigation.

Les deux actions sont exécutées dans des threads afin d’obtenir une
suivi simultané. Au bout de l’exécution, les moteurs sont arrêtés pour éviter toute poursuite accidentelle.
"""

import math
import time
import threading

import numpy as np

# Import du module principal (bblib.py)
from utils.bblib import BlueBoatConfig


def main():
    # ------------------------------------------------------------------
    # Initialisation des deux bateaux
    # ------------------------------------------------------------------
    cfg = BlueBoatConfig()
    mav2, imu2, gps2, motors2, nav2 = cfg.init_from_config(2)
    mav3, imu3, gps3, motors3, nav3 = cfg.init_from_config(3)

    # ------------------------------------------------------------------
    # 1) Bateau 1 : suivi de cap fixe (90°) pendant 5 s
    # ------------------------------------------------------------------
    def boat2_heading_task():
        print("[Boat2] Démarrage du suivi de cap à 90° pendant 5 s.")
        nav2.follow_heading(90.0, duration_s=5.0)
        print("[Boat2] Suivi terminé, moteurs arrêtés.")

    # ------------------------------------------------------------------
    # 2) Bateau 3 : poursuite du bateau 1 en temps réel
    # ------------------------------------------------------------------
    def boat3_follow_task():
        """
        Boucle de suivi du bateau 2.
        Elle s'exécute pendant une durée fixe (20 s).
        """
        print("[Boat3] Démarrage de la poursuite en temps réel.")
        start_time = time.time()
        duration = 20.0  # secondes

        while True:
            # Récupération des positions GPS (cartésiennes)
            tgt_coords = gps2.get_coords()   # coordonnées du bateau 2
            cur_coords = gps3.get_coords()   # coordonnées de ce bateau

            if (
                tgt_coords[0] is None
                or tgt_coords[1] is None
                or cur_coords[0] is None
                or cur_coords[1] is None
            ):
                time.sleep(nav3.dt)
                continue

            dx = tgt_coords[0] - cur_coords[0]
            dy = tgt_coords[1] - cur_coords[1]
            distance = np.hypot(dx, dy)

            # Cap vers le bateau 2 (en degrés, dans [0,360))
            desired_heading = (-math.degrees(math.atan2(dy, dx)) + 360.0) % 360.0

            current_heading = nav3.get_current_heading()
            if current_heading is None:
                time.sleep(nav3.dt)
                continue

            # Erreur de cap ([-180, 180])
            error = current_heading - desired_heading
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360

            correction = nav3.Kp * error

            # Vitesse proportionnelle à la distance (limite à 5 m pour éviter les oscillations)
            base_speed = (
                min(distance / 5.0, 1.0) * nav3.max_speed * 0.9
            )  # max ~225

            left_motor = base_speed + correction
            right_motor = base_speed - correction

            motors3.send_cmd_motor(left_motor, right_motor)

            print(
                f"[Boat3] Dist={distance:6.2f}m  Err={error:5.1f}°  Spd={base_speed:6.2f}",
                end="\r",
            )

            # Vérification de la durée
            if time.time() - start_time > duration:
                break

            time.sleep(nav3.dt)

        motors3.stop_motors()
        print("\n[Boat3] Poursuite terminée, moteurs arrêtés.")

    # ------------------------------------------------------------------
    # Lancement des deux tâches en parallèle
    # ------------------------------------------------------------------
    thread2 = threading.Thread(target=boat2_heading_task)
    thread3 = threading.Thread(target=boat3_follow_task)

    try:
        thread2.start()
        # On attend un peu que le bateau 1 commence à avancer avant de lancer la poursuite
        time.sleep(0.5)
        thread3.start()

        # Attente de fin des deux threads
        thread2.join()
        thread3.join()

    except KeyboardInterrupt:
        print("\n[Main] Interruption utilisateur, arrêt immédiat.")
    finally:
        # S'assurer que les moteurs sont arrêtés
        motors2.stop_motors()
        motors3.stop_motors()


if __name__ == "__main__":
    main()
