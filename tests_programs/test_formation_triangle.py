"""
Script de test custom pour la formation en triangle, avec toutes les infos connues (GPS, cap etc.)
"""

import math
import time
import threading

import numpy as np

# Import du module principal (bblib.py)
from utils.bblib import BlueBoatConfig



def main():
    # ------------------------------------------------------------------
    # Initialisation des trois bateaux
    # ------------------------------------------------------------------
    config = BlueBoatConfig()
    mav1, imu1, gps1, motors1, nav1 = config.init_from_config(boat_id=1)
    mav2, imu2, gps2, motors2, nav2 = config.init_from_config(boat_id=2)
    mav3, imu3, gps3, motors3, nav3 = config.init_from_config(boat_id=3)

    # ------------------------------------------------------------------
    # fonction de calcul des points cibles en temps réel
    # ------------------------------------------------------------------
    def compute_targets():
        """
        Compute target points for scouts A and B based on mothership position.

        The scouts are positioned to form an equilateral triangle with the mothership:
        - Mothership is at the rear vertex of the triangle
        - Scouts A and B are at the two front vertices
        - All three sides have equal length (equilateral triangle)
        - The triangle points in the mothership's heading direction
        
        Parameters:
        -----------
        mothership_state : list or tuple
            [x, y, theta, vx, vy, omega] - mothership state
        distance : float
            Length of each side of the equilateral triangle (meters)
        
        Returns:
        --------
        tuple : (target_A, target_B)
            target_A : np.array([[x], [y]]) - target position for scout A (left front vertex)
            target_B : np.array([[x], [y]]) - target position for scout B (right front vertex)
        """
        # Extract mothership position and heading
        ms_pos = np.array([[gps1.get_coords()[0]], [gps1.get_coords()[1]]])
        ms_theta = math.radians(nav1.get_current_heading())

        #print(ms_pos)

        # For an equilateral triangle with mothership at rear vertex:
        # - Each scout is at distance = side_length from mothership
        # - Each scout is at ±30° from the heading direction
        # - This ensures all three sides have equal length
        
        side = 20.0  # meters
        
        # Angle offset for equilateral triangle: 30 degrees = pi/6 radians
        angle_offset = np.pi / 6.0
        
        # Boat A target: at angle (heading + 30°), distance = side_length
        angle_A = ms_theta + angle_offset
        target_A = ms_pos + side * np.array([[np.cos(angle_A)], [np.sin(angle_A)]])
        
        # Boat B target: at angle (heading - 30°), distance = side_length
        angle_B = ms_theta - angle_offset
        target_B = ms_pos + side * np.array([[np.cos(angle_B)], [np.sin(angle_B)]])
        
        return target_A, target_B
    
    # ------------------------------------------------------------------
    # Boucle principale de suivi des points cibles
    # ------------------------------------------------------------------
    print("[Boat1] Démarrage du suivi GPS.")
    print("[Boat2] Démarrage du suivi GPS.")
    print("[Boat3] Démarrage du suivi GPS.")
    print("[Boat2] Armement.")
    print("[Boat3] Armement.")
    mav2.arm_disarm(True)
    mav3.arm_disarm(True)
    start_time = time.time()
    duration = 1000.0  # secondes

    coordsA = []
    coordsB = []

    while True:
        # Récupération des positions GPS (cartésiennes)
        tgt_coords_A, tgt_coords_B = compute_targets()
        cur_coords_A = gps2.get_coords()   # coordonnées du bateau 2
        cur_coords_B = gps3.get_coords()   # coordonnées du bateau 3


        if (
            tgt_coords_A[0] is None
            or tgt_coords_A[1] is None
            or cur_coords_A[0] is None
            or cur_coords_A[1] is None
            or tgt_coords_B[0] is None
            or tgt_coords_B[1] is None
            or cur_coords_B[0] is None
            or cur_coords_B[1] is None
        ):
            time.sleep(nav3.dt)
            continue

        # Commande de suivi pour le bateau 2 (scout A)
        nav2.go_to_position(tgt_coords_A)

        # Commande de suivi pour le bateau 3 (scout B)
        nav3.go_to_position(tgt_coords_B)

        coordsA.append(cur_coords_A)
        coordsB.append(cur_coords_B)

        
        #print(cur_coords_B, tgt_coords_B)

        if time.time() - start_time > duration:
            break

        time.sleep(nav3.dt)

    motors2.stop_motors()
    motors3.stop_motors()
    time.sleep(3.0)  # temps pour s'arrêter
    print("[Boat2] Suivi GPS terminé, moteurs arrêtés, retour au ponton.")
    print("[Boat3] Suivi GPS terminé, moteurs arrêtés, retour au ponton.")


    #np.save("coordsA.npy", np.array(coordsA))
    #np.save("coordsB.npy", np.array(coordsB))
    # retour lobby
    #nav2.return_home()
    #nav3.return_home()

    # safety
    motors2.stop_motors()
    motors3.stop_motors()



if __name__ == "__main__":
    main()
