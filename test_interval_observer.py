"""
Test de l'observateur par intervalles avec 3 bateaux

Ce script initialise 3 bateaux et les laisse à l'arrêt.
Ils seront contrôlés à la manette via QGroundControl (MavLink).
On utilise ensuite vibes_display.py pour estimer leur positions avec des intervalles.
"""

import time
import math
import logging
from xxlimited import new
from codac import Interval, IntervalVector

from utils.bblib import init_blueboat

from utils.prediction import equivalent_contractor
from utils.vibes_display import VibesDisplay


# ----------------------------------------------------------------------------
# Constant uncertainties 
# ----------------------------------------------------------------------------

INIT_UNCERTAINTY = 2.5
GPS_UNCERTAINTY = 2.5
DIST_UNCERTAINTY = 2.5

# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------

def true_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def make_init_box(x, y):
    return IntervalVector([x, y]).inflate(INIT_UNCERTAINTY)


def box_size_str(box: IntervalVector) -> str:
    return f"dx={box[0].diam():.3f}, dy={box[1].diam():.3f}"

# -------------------------------------------------------------------------------
# Logging setup
# -------------------------------------------------------------------------------
log_file = "test_observer.log"

logging.basicConfig(
    filename=log_file,
    filemode='w',
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# -------------------------------------------------------------------------------
# Main
# -------------------------------------------------------------------------------
def main():
    """
    Initialise 3 bateaux et les laisse à l'arrêt.
    """
    print("=" * 60)
    print("Initialisation des 3 bateaux...")
    print("=" * 60)

    # ------------------------------------------------------------------
    # Initialisation des trois bateaux
    # ------------------------------------------------------------------

    # Bateau 1 (sysid=1)
    print("\n[Bateau 1] Initialisation sur 127.0.0.1...")
    mav1, imu1, gps1, motors1, nav1 = init_blueboat(
        host="127.0.0.1", 
        port=5763,
        sysid=1
    )
    motors1.stop_motors()
    print("[Bateau 1] ✓ Initialisé")

    # Bateau 2 (sysid=2)
    print("\n[Bateau 2] Initialisation sur 192.168.2.202...")
    mav2, imu2, gps2, motors2, nav2 = init_blueboat(
        host="192.168.2.202", 
        port=6040,
        sysid=2
    )
    motors2.stop_motors()
    print("[Bateau 2] ✓ Initialisé")

    # Bateau 3 (sysid=3)
    print("\n[Bateau 3] Initialisation sur 192.168.2.203...")
    mav3, imu3, gps3, motors3, nav3 = init_blueboat(
        host="192.168.2.203", 
        port=6040,
        sysid=3
    )
    motors3.stop_motors()
    print("[Bateau 3] ✓ Initialisé")

    print("\n" + "=" * 60)
    print("Les 3 bateaux sont initialisés et à l'arrêt.")
    print("Appuyez sur Ctrl+C pour quitter.")
    print("=" * 60)
    
    # ------------------------------------------------------------------
    # Initialisation des contractors et de l'affichage VIBes
    # ------------------------------------------------------------------
    box1 = make_init_box(0.0, 0.0)
    box2 = make_init_box(10.0, 0.0)
    box3 = make_init_box(0.0, 10.0)
    c1 = equivalent_contractor(box1)
    c2 = equivalent_contractor(box2)
    c3 = equivalent_contractor(box3)
    display = VibesDisplay(c1, c2, c3, precision=1.0, margin=10.0)
    coords1 = [0.0, 0.0]
    coords2 = [10.0, 0.0]
    coords3 = [0.0, 10.0]

    # ------------------------------------------------------------------
    # Boucle principale d'affichage
    # ------------------------------------------------------------------
    try:
        while True:
            # Récupération des données en coordonnées cartésiennes
            new_coords1 = gps1.get_coords()
            new_coords2 = gps2.get_coords()
            new_coords3 = gps3.get_coords()

            # Handle None case for Boat 1
            if new_coords1[0] is not None and new_coords1[1] is not None:
                coords1 = new_coords1
            else:
                print("[Warning] No data received from Boat 1. Using last known coordinates.")

            # Handle None case for Boat 2
            if new_coords2[0] is not None and new_coords2[1] is not None:
                coords2 = new_coords2
            else:
                print("[Warning] No data received from Boat 2. Using last known coordinates.")

            # Handle None case for Boat 3
            if new_coords3[0] is not None and new_coords3[1] is not None:
                coords3 = new_coords3
            else:
                print("[Warning] No data received from Boat 3. Using last known coordinates.")

            # Log the data
            logging.info(f"Boat 1: {coords1}, Boat 2: {coords2}, Boat 3: {coords3}")

            # Initialisation des boîtes pour les bateaux
            box1 = IntervalVector([coords1[0], coords1[1]]).inflate(GPS_UNCERTAINTY)
            box2 = IntervalVector([coords2[0], coords2[1]]).inflate(GPS_UNCERTAINTY)
            box3 = IntervalVector([coords3[0], coords3[1]]).inflate(GPS_UNCERTAINTY)

            # Mise à jour des contraintes de distance
            d12 = Interval(true_distance(coords1, coords2)).inflate(DIST_UNCERTAINTY)
            d13 = Interval(true_distance(coords1, coords3)).inflate(DIST_UNCERTAINTY)
            d23 = Interval(true_distance(coords2, coords3)).inflate(DIST_UNCERTAINTY)

            c1 = equivalent_contractor(box1)
            c2 = equivalent_contractor(box2)
            c3 = equivalent_contractor(box3)

            c1.add_distance_condition(d12, c2.get_box())
            c1.add_distance_condition(d13, c3.get_box())

            c2.add_distance_condition(d12, c1.get_box())
            c2.add_distance_condition(d23, c3.get_box())

            c3.add_distance_condition(d13, c1.get_box())
            c3.add_distance_condition(d23, c2.get_box())

            # Affichage des pavages avec VIBes
            display.draw()

            # Attente avant la prochaine mise à jour
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n[Main] Interruption utilisateur détectée. Données sauvegardées dans 'boat_data.log'.")

    finally:
        # S'assurer que tous les moteurs sont arrêtés
        print("\n[Main] Arrêt des moteurs...")
        motors1.stop_motors()
        motors2.stop_motors()
        motors3.stop_motors()
        print("[Main] ✓ Tous les moteurs sont arrêtés.")

if __name__ == "__main__":
    main()