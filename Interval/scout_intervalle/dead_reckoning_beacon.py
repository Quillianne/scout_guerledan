#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from codac import *
import numpy as np
import time as time_module
from paver import Paver

# Paramètres de simulation
dt = 0.1  # Pas de temps (s)
t_max = 10.0  # Durée totale (s)
observation_period = 1.0  # Période d'observation des beacons (s)
paving_precision = 1.0  # Précision du pavage (m)

# Vérité terrain
x_true = 0.0
y_true = 0.0
v_true = 5.0  # m/s
psi_true = np.pi / 4  # rad (45 degrés)

# Incertitudes
position_uncertainty = 1.0  # m
v_uncertainty = 0.5  # m/s
psi_uncertainty = 0.1  # rad
distance_uncertainty = 3.0  # m

# Beacons avec incertitude de position
beacon1_true = [10.0, 40.0]
beacon2_true = [40.0, 10.0]
beacon1_uncertainty = 0.5  # m
beacon2_uncertainty = 0.5  # m

# Initialisation VIBes
beginDrawing()
fig_map = VIBesFigMap("Dead Reckoning - Static")
fig_map.set_properties(100, 100, 800, 800)
fig_map.axis_limits(-5, 50, -5, 50)

# Dessiner les beacons
b1_box = IntervalVector([beacon1_true[0], beacon1_true[1]]).inflate(beacon1_uncertainty)
fig_map.add_beacon(b1_box)

b2_box = IntervalVector([beacon2_true[0], beacon2_true[1]]).inflate(beacon2_uncertainty)
fig_map.add_beacon(b2_box)

# Box de position initiale
x_box = IntervalVector([x_true, y_true]).inflate(position_uncertainty)

# Mesures de vitesse et cap avec incertitude
v_interval = Interval(v_true).inflate(v_uncertainty)
psi_interval = Interval(psi_true).inflate(psi_uncertainty)

# Boucle principale
t = 0.0
last_observation_time = 0.0

# Initialiser le paveur
paver = Paver(precision=paving_precision)

print(f"Position initiale: x={x_box[0].mid():.2f}m ±{x_box[0].diam()/2:.2f}, y={x_box[1].mid():.2f}m ±{x_box[1].diam()/2:.2f}")

try:
    while t < t_max:
        # 1. Translation du box de position (dead reckoning)
        # Calculer le déplacement
        vx_interval = v_interval * cos(psi_interval)
        vy_interval = v_interval * sin(psi_interval)
        
        dx = vx_interval * dt
        dy = vy_interval * dt
        
        # Translater le box
        x_box[0] += dx
        x_box[1] += dy
        
        # Mise à jour de la vérité terrain
        x_true += v_true * np.cos(psi_true) * dt
        y_true += v_true * np.sin(psi_true) * dt
        
        t += dt
        
        # 2. Observation des beacons toutes les secondes
        if t - last_observation_time >= observation_period:
            last_observation_time = t
            
            # Calculer les distances vraies
            d1_true = np.sqrt((x_true - beacon1_true[0])**2 + (y_true - beacon1_true[1])**2)
            d2_true = np.sqrt((x_true - beacon2_true[0])**2 + (y_true - beacon2_true[1])**2)
            
            # Mesures avec incertitude
            y1 = Interval(d1_true).inflate(distance_uncertainty)
            y2 = Interval(d2_true).inflate(distance_uncertainty)
            
            print(f"\nObservation à t={t:.1f}s:")
            print(f"  d1={d1_true:.1f}m ±{distance_uncertainty:.1f}, d2={d2_true:.1f}m ±{distance_uncertainty:.1f}")
            
            # Positions des beacons (IntervalVector)
            b1 = IntervalVector([beacon1_true[0], beacon1_true[1]]).inflate(beacon1_uncertainty)
            b2 = IntervalVector([beacon2_true[0], beacon2_true[1]]).inflate(beacon2_uncertainty)
            
            # Créer le contracteur de distance localement
            ctc_dist = CtcDist()
            
            def beacon_contractor(x_box: IntervalVector) -> IntervalVector:
                """Contracte le box avec les contraintes de distance aux beacons"""
                result = x_box.copy()
                
                # Contrainte beacon 1
                box5d_1 = IntervalVector(5)
                box5d_1[0] = result[0]
                box5d_1[1] = result[1]
                box5d_1[2] = b1[0]
                box5d_1[3] = b1[1]
                box5d_1[4] = y1
                ctc_dist.contract(box5d_1)
                result[0] = box5d_1[0]
                result[1] = box5d_1[1]
                
                if result.is_empty():
                    return result
                
                # Contrainte beacon 2
                box5d_2 = IntervalVector(5)
                box5d_2[0] = result[0]
                box5d_2[1] = result[1]
                box5d_2[2] = b2[0]
                box5d_2[3] = b2[1]
                box5d_2[4] = y2
                ctc_dist.contract(box5d_2)
                result[0] = box5d_2[0]
                result[1] = box5d_2[1]
                
                return result
            
            # Paver la position actuelle avec les contraintes de distance
            current_boxes = paver.pave(x_box, beacon_contractor)
            
            print(f"  Nombre de boîtes: {len(current_boxes)}")
            
            # Limiter le nombre de boîtes à dessiner
            max_boxes = 300
            boxes_to_draw = current_boxes[:max_boxes] if len(current_boxes) > max_boxes else current_boxes
            
            # Effacer la figure et redessiner tout
            fig_map.clear()
            fig_map.add_beacon(b1_box)
            fig_map.add_beacon(b2_box)
            
            # Dessiner les nouvelles boîtes
            for box in boxes_to_draw:
                fig_map.draw_box(box, "red[orange]")
            
            # Mettre à jour x_box avec la boîte englobante
            x_box = paver.get_bounding_box()
            print(f"Après contraction: x={x_box[0].mid():.2f}m ±{x_box[0].diam()/2:.2f}, y={x_box[1].mid():.2f}m ±{x_box[1].diam()/2:.2f}")
            
            fig_map.show()
        
        # Affichage périodique (sans observation)
        if int(t * 10) % 5 == 0 and t - last_observation_time < 0.05:
            print(f"t={t:.1f}s | x={x_box[0].mid():.2f}m ±{x_box[0].diam()/2:.2f} | y={x_box[1].mid():.2f}m ±{x_box[1].diam()/2:.2f}")
        
        # Pause pour animation
        time_module.sleep(0.05)

except KeyboardInterrupt:
    print("\nArrêt demandé par l'utilisateur")

print(f"\nPosition finale: x={x_box[0].mid():.2f}m ±{x_box[0].diam()/2:.2f}, y={x_box[1].mid():.2f}m ±{x_box[1].diam()/2:.2f}")
print(f"Position vraie: x={x_true:.2f}m, y={y_true:.2f}m")
print("\nAppuyez sur Ctrl+C pour arrêter")

# Keep the figure open
fig_map.show()
time_module.sleep(100)
