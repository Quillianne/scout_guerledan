#!/usr/bin/env python3
"""
Exemple d'utilisation de bblib.py avec contrôle moteurs MAVLink pour BlueBoat
===========================================================================

Ce script montre comment utiliser la nouvelle API MAVLink pour contrôler
les moteurs du BlueBoat via RC override.
"""

import time
import math
from utils.bblib import init_blueboat

conn_str = "udp:127.0.0.1:14550"

def test_motor_control():
    """Test des commandes moteurs via MAVLink"""
    print("Initialisation de la connexion MAVLink...")
    try:
        mav, imu, gps, motors, nav = init_blueboat(conn_str)
        print("✓ Connexion MAVLink établie")
    except Exception as e:
        print(f"✗ Erreur de connexion: {e}")
        print("Vérifiez que BlueOS fonctionne et que le port 14550 est accessible")
        return

    # Test 1: Arrêt des moteurs
    print("\n1. Test arrêt moteurs...")
    motors.stop_motors()
    time.sleep(2)

    # Test 2: Commandes directes throttle/steering
    print("2. Test commandes directes (throttle=50, steering=0)...")
    motors.send_throttle_steering(50, 0)  # Avancer doucement
    time.sleep(3)
    motors.stop_motors()
    time.sleep(1)

    # Test 3: Commandes différentielles (compatible ancienne API)
    print("3. Test commandes différentielles (gauche=100, droite=100)...")
    motors.send_arduino_cmd_motor(100, 100)  # Avancer
    time.sleep(3)
    
    print("4. Test virage à droite (gauche=100, droite=50)...")
    motors.send_arduino_cmd_motor(100, 50)   # Virage droite
    time.sleep(2)
    
    print("5. Test marche arrière (gauche=-80, droite=-80)...")
    motors.send_arduino_cmd_motor(-80, -80)  # Reculer
    time.sleep(2)
    
    motors.stop_motors()
    print("✓ Tests moteurs terminés")

def test_navigation():
    """Test de navigation automatique"""
    print("\nTest de navigation...")
    try:
        mav, imu, gps, motors, nav = init_blueboat(conn_str)
        
        # Vérifier la lecture des capteurs
        heading = nav.get_current_heading()
        if heading is not None:
            print(f"Cap actuel: {heading:.1f}°")
        else:
            print("⚠ Pas de données IMU disponibles")
            
        pos = gps.get_gps()
        if pos:
            print(f"Position GPS: {pos[0]:.6f}, {pos[1]:.6f}")
        else:
            print("⚠ Pas de données GPS disponibles")
            
        # Test de suivi de cap (attention: le bateau bougera!)
        print("\nTest suivi de cap 90° pendant 10s (décommentez pour activer)...")
        # nav.follow_heading(90.0, 10.0)  # Cap Est pendant 10 secondes
        
    except Exception as e:
        print(f"Erreur lors du test de navigation: {e}")

def demo_arming():
    """Démonstration d'armement/désarmement"""
    print("\nDémonstration armement/désarmement...")
    try:
        mav, imu, gps, motors, nav = init_blueboat(conn_str)
        
        print("Mode MANUAL...")
        mav.set_mode("MANUAL")
        time.sleep(1)
        
        print("Armement du véhicule...")
        mav.arm_disarm(arm=True)
        time.sleep(2)
        
        print("Désarmement du véhicule...")
        mav.arm_disarm(arm=False)
        
    except Exception as e:
        print(f"Erreur: {e}")

if __name__ == "__main__":
    print("=== Test BlueBoat MAVLink Motor Control ===\n")
    
    # Choisir le test à exécuter
    choice = input("Choisir le test:\n1. Contrôle moteurs\n2. Navigation\n3. Armement/désarmement\n4. Tout\nChoix (1-4): ")
    
    if choice == "1":
        test_motor_control()
    elif choice == "2":
        test_navigation()
    elif choice == "3":
        demo_arming()
    elif choice == "4":
        test_motor_control()
        test_navigation()
        demo_arming()
    else:
        print("Choix invalide")