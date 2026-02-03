"""
Initialisation de 3 bateaux et mise en attente.

Ce fichier est inutile.
"""

import time
from utils.bblib import init_blueboat


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
    print("\n[Bateau 1] Initialisation sur 192.168.1.1...")
    mav1, imu1, gps1, motors1, nav1 = init_blueboat(
        host="192.168.1.1", 
        sysid=1,
        port=6040
    )
    motors1.stop_motors()
    print("[Bateau 1] ✓ Initialisé")
    
    # Bateau 2 (sysid=2)
    print("\n[Bateau 2] Initialisation sur 192.168.2.202...")
    mav2, imu2, gps2, motors2, nav2 = init_blueboat(
        host="192.168.2.202", 
        sysid=2,
        port=6040
    )
    motors2.stop_motors()
    print("[Bateau 2] ✓ Initialisé")
    
    # Bateau 3 (sysid=3)
    print("\n[Bateau 3] Initialisation sur 192.168.2.203...")
    mav3, imu3, gps3, motors3, nav3 = init_blueboat(
        host="192.168.2.203", 
        sysid=3,
        port=6040
    )
    motors3.stop_motors()
    print("[Bateau 3] ✓ Initialisé")
    
    print("\n" + "=" * 60)
    print("Les 3 bateaux sont initialisés et à l'arrêt.")
    print("Appuyez sur Ctrl+C pour quitter.")
    print("=" * 60)
    
    # ------------------------------------------------------------------
    # Boucle d'attente principale
    # ------------------------------------------------------------------
    try:
        while True:
            # Récupération des données et affichage
            print(f"\n[Bateau 1] Position GPS : {gps1.get_coords()}")
            print(f"[Bateau 2] Position GPS : {gps2.get_coords()}")
            print(f"[Bateau 3] Position GPS : {gps3.get_coords()}")
            
            # Attente avant la prochaine mise à jour
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n\n[Main] Interruption utilisateur détectée.")
    
    finally:
        # S'assurer que tous les moteurs sont arrêtés
        print("\n[Main] Arrêt des moteurs...")
        motors1.stop_motors()
        motors2.stop_motors()
        motors3.stop_motors()
        print("[Main] ✓ Tous les moteurs sont arrêtés.")


if __name__ == "__main__":
    main()
