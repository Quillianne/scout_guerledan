"""
Mission avec un trajet prédéfini,
pour collecter des logs afin de valider l'observateur par intervalles seul,
sans loi de commande.

Le mothership suit une droite,
un scout est dirigé par Télécommande RC pour effectuer les mouvements voulus,
et le 3ème scout reste au ponton.
"""

import logging
import time

from utils.bblib import BlueBoatConfig
from utils.prediction import equivalent_contractor
from utils.vibes_display import VibesDisplay


def main():
    # -------------------------------------------------------------------------------
    # Logging setup
    # -------------------------------------------------------------------------------
    log_file = "test_go_to_gps.log"

    logging.basicConfig(
        filename=log_file,
        filemode='w',
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )


    # -------------------------------------------------------------------------------
    # Waypoints
    # -------------------------------------------------------------------------------
    # beginning and end of the line to follow for the mothership
    start = (48.199776862150934, -3.015781017195938)
    end = (48.19880233222624, -3.0154935087317236)


    # -------------------------------------------------------------------------------
    # Main
    # -------------------------------------------------------------------------------
    """
    Initialize the robot.
    """
    print("=" * 60)
    print("Initialisation du robot...")
    print("=" * 60)

    # Load configuration for the robots
    config = BlueBoatConfig()

    # Initialize Robot 1
    print("\n[Robot 1] Initialisation via configuration...")
    mav1, imu1, gps1, motors1, nav1 = config.init_from_config(boat_id=1)
    motors1.stop_motors()
    print("[Robot 1] ✓ Initialisé")


    # ------------------------------------------------------------------------------
    # Mission Loop
    # ------------------------------------------------------------------------------

    # enter a key to launch the mission
    input("\nAppuyez sur Entrée pour envoyer le Motership à sa position de départ...")
    # go to start position
    nav1.go_to_gps(start)

    input("\nAppuyez sur Entrée pour démarrer la mission...")
    # go to end position
    nav1.go_to_gps(end)

    motors1.stop_motors()
    print("\n[Main] Mission terminée. Les robots sont à l'arrêt.")


if __name__ == "__main__":
    main()
