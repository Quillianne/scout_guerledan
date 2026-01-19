#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simulation de flotte de bateaux avec visualisation
Bateau mère avec GPS et deux scouts éclaireurs
"""

from codac import *
from boat import Boat
from fleet import Fleet
import numpy as np
import time as time_module


def main():
    """
    Simulation principale
    """
    # Paramètres de simulation
    t_max = 50.0  # Durée totale (s)
    
    # Créer la flotte
    fleet = Fleet(paving_precision=1.0)
    
    # === Configuration des bateaux ===
    
    # Bateau mère au centre avec GPS
    mother = Boat("mother", [0.0, 0.0], initial_uncertainty=0.5, has_gps=True)
    mother.set_motion(v=4.0, psi=0.0)  # 3 m/s vers l'est
    fleet.add_boat(mother)
    
    # Scout 1 - part vers le nord-est (devant à droite)
    scout1 = Boat("scout1", [10.0, 10.0], initial_uncertainty=1.0)
    scout1.set_motion(v=4.0, psi=0.0)  # 4 m/s, 30 degrés
    fleet.add_boat(scout1)
    
    # Scout 2 - part vers le sud-est (devant à gauche)
    scout2 = Boat("scout2", [10.0, -10.0], initial_uncertainty=1.0)
    scout2.set_motion(v=4.0, psi=0.0)  # 4 m/s, -30 degrés
    fleet.add_boat(scout2)
    
    # === Visualisation VIBes ===
    beginDrawing()
    fig_map = VIBesFigMap("Fleet Navigation")
    fig_map.set_properties(100, 100, 1000, 800)
    fig_map.axis_limits(-10, 80, -30, 30)
    
    # Couleurs pour chaque bateau
    colors = {
        "mother": "blue[lightblue]",
        "scout1": "red[orange]",
        "scout2": "green[lightgreen]"
    }
    
    print("=== Simulation de flotte de bateaux ===")
    print("Bateau mère: GPS actif")
    print("2 Scouts éclaireurs: dead reckoning + mesures de distance")
    print(f"Durée: {t_max}s\n")
    
    fleet.print_status()
    
    # === Boucle de simulation ===
    try:
        iteration = 0
        while fleet.time < t_max:
            # Prédiction pour tous les bateaux (10 Hz)
            fleet.predict_all()
            
            # Mise à jour avec mesures de distance (1 Hz)
            fleet.update_with_distance_measurements()
            
            # Affichage et visualisation
            if iteration % 1 == 0:
                # Effacer et redessiner
                fig_map.clear()
                
                # Dessiner chaque bateau
                for boat in fleet.get_all_boats():
                    # Pour les scouts : dessiner les boîtes pavées
                    if not boat.has_gps:
                        paved_boxes = boat.get_paved_boxes()
                        # Limiter le nombre de boîtes pour la performance
                        max_boxes = 1000
                        for i, box in enumerate(paved_boxes):
                            if i >= max_boxes:
                                break
                            fig_map.draw_box(box, colors[boat.id])
                    else:
                        # Pour le bateau mère : dessiner la box englobante
                        fig_map.draw_box(boat.get_position_box(), colors[boat.id])
                    
                    # Position vraie (point)
                    true_pos = boat.get_true_position()
                    fig_map.draw_vehicle(true_pos, 0.5)
                
                fig_map.show()
            
            # Affichage du statut (1 Hz)
            if iteration % 10 == 0:
                fleet.print_status()
            
            iteration += 1
            time_module.sleep(0.05)  # Ralentir pour visualisation
    
    except KeyboardInterrupt:
        print("\n\nSimulation interrompue par l'utilisateur")
    
    # === Résumé final ===
    print("\n" + "="*60)
    print("RÉSUMÉ FINAL")
    print("="*60)
    fleet.print_status()
    
    print("\n=== Distances entre bateaux ===")
    boats = fleet.get_all_boats()
    for i, boat1 in enumerate(boats):
        for boat2 in boats[i+1:]:
            dist = boat1.measure_distance_to(boat2)
            true_pos1 = boat1.get_true_position()
            true_pos2 = boat2.get_true_position()
            dist_true = np.sqrt((true_pos1[0] - true_pos2[0])**2 + 
                               (true_pos1[1] - true_pos2[1])**2)
            print(f"{boat1.id} <-> {boat2.id}: {dist} (vraie: {dist_true:.2f}m)")
    
    print("\n=== Incertitudes finales ===")
    for boat in boats:
        unc_x, unc_y = boat.get_uncertainty()
        print(f"{boat.id}: ±{unc_x:.2f}m (x), ±{unc_y:.2f}m (y)")
    
    print("\nAppuyez sur Ctrl+C pour fermer")
    try:
        time_module.sleep(100)
    except KeyboardInterrupt:
        print("Fermeture")


if __name__ == "__main__":
    main()
