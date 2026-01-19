#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Flotte de bateaux - Gestion de plusieurs bateaux avec communication
"""

from codac import *
from boat import Boat
from paver import Paver
from typing import List, Dict
import numpy as np


class Fleet:
    """
    Classe gérant une flotte de bateaux avec communications et mesures inter-bateaux
    """
    
    def __init__(self, paving_precision: float = 5.0):
        """
        Initialise la flotte
        
        Args:
            paving_precision: Précision du pavage pour les mises à jour (m)
        """
        self.boats: Dict[str, Boat] = {}
        self.paver = Paver(precision=paving_precision)
        self.time = 0.0
        
        # Fréquences
        self.predict_dt = 0.1  # 10 Hz pour dead reckoning
        self.distance_measurement_period = 1.0  # 1 Hz pour mesures de distance
        self.last_distance_measurement_time = 0.0
    
    def add_boat(self, boat: Boat):
        """
        Ajoute un bateau à la flotte
        
        Args:
            boat: Instance de Boat à ajouter
        """
        self.boats[boat.id] = boat
    
    def predict_all(self):
        """
        Prédiction pour tous les bateaux (dead reckoning) - 10 Hz
        """
        for boat in self.boats.values():
            boat.predict(self.predict_dt)
        
        self.time += self.predict_dt
    
    def update_with_distance_measurements(self):
        """
        Mise à jour avec les mesures de distance entre bateaux - 1 Hz
        """
        if self.time - self.last_distance_measurement_time < self.distance_measurement_period:
            return
        
        self.last_distance_measurement_time = self.time
        
        # Le bateau mère se met à jour avec GPS
        for boat in self.boats.values():
            if boat.has_gps:
                boat.update_from_gps()
        
        # Chaque bateau scout mesure sa distance aux autres
        boat_list = list(self.boats.values())
        for i, boat in enumerate(boat_list):
            if boat.has_gps:
                continue  # Le bateau mère n'a pas besoin de se mettre à jour avec les distances
            
            # Mesurer les distances aux autres bateaux
            other_boats = []
            distances = []
            
            for j, other_boat in enumerate(boat_list):
                if i != j:
                    distance = boat.measure_distance_to(other_boat)
                    other_position = other_boat.get_position_box()
                    
                    other_boats.append(other_position)
                    distances.append(distance)
            
            # Créer le contracteur de distance
            if other_boats:
                paved_boxes = self._update_position_with_distances(
                    boat.x_box, 
                    other_boats, 
                    distances
                )
                # Stocker les boîtes pavées dans le bateau
                boat.paved_boxes = paved_boxes
                # Mettre à jour la position avec la bounding box
                if paved_boxes:
                    boat.x_box = self.paver.get_bounding_box()
    
    def _update_position_with_distances(self,
                                       position_box: IntervalVector,
                                       beacon_positions: List[IntervalVector],
                                       distances: List[Interval]) -> List[IntervalVector]:
        """
        Met à jour la position d'un bateau avec des mesures de distance
        
        Args:
            position_box: Position actuelle du bateau
            beacon_positions: Positions des autres bateaux
            distances: Distances mesurées
            
        Returns:
            List[IntervalVector]: Boîtes pavées
        """
        # Créer le contracteur de distance
        ctc_dist = CtcDist()
        
        def distance_contractor(x_box: IntervalVector) -> IntervalVector:
            """Contracte avec toutes les contraintes de distance"""
            result = x_box.copy()
            
            for beacon_pos, distance in zip(beacon_positions, distances):
                box5d = IntervalVector(5)
                box5d[0] = result[0]
                box5d[1] = result[1]
                box5d[2] = beacon_pos[0]
                box5d[3] = beacon_pos[1]
                box5d[4] = distance
                
                ctc_dist.contract(box5d)
                result[0] = box5d[0]
                result[1] = box5d[1]
                
                if result.is_empty():
                    return result
            
            return result
        
        # Paver avec le contracteur
        boxes = self.paver.pave(position_box, distance_contractor)
        
        # Retourner les boîtes pavées
        return boxes
    
    def get_boat(self, boat_id: str) -> Boat:
        """
        Récupère un bateau par son ID
        
        Args:
            boat_id: ID du bateau
            
        Returns:
            Boat: Instance du bateau
        """
        return self.boats.get(boat_id)
    
    def get_all_boats(self) -> List[Boat]:
        """
        Retourne tous les bateaux
        
        Returns:
            List[Boat]: Liste de tous les bateaux
        """
        return list(self.boats.values())
    
    def print_status(self):
        """
        Affiche le statut de tous les bateaux
        """
        print(f"\n=== Fleet Status at t={self.time:.2f}s ===")
        for boat in self.boats.values():
            print(boat)


if __name__ == "__main__":
    # Test de la flotte
    print("Test de la classe Fleet\n")
    
    # Créer la flotte
    fleet = Fleet(paving_precision=10)
    
    # Bateau mère au centre avec GPS
    mother = Boat("mother", [0.0, 0.0], initial_uncertainty=0.5, has_gps=True)
    mother.set_motion(v=3.0, psi=0.0)  
    fleet.add_boat(mother)
    
    scout1 = Boat("scout1", [0.0, 5.0], initial_uncertainty=1.0)
    scout1.set_motion(v=4.0, psi=np.pi/4) 
    fleet.add_boat(scout1)
    
    scout2 = Boat("scout2", [0.0, -5.0], initial_uncertainty=1.0)
    scout2.set_motion(v=4.0, psi=-np.pi/4) 
    fleet.add_boat(scout2)
    
    fleet.print_status()
    
    # Simuler 5 secondes
    for i in range(50):  # 50 itérations à 10 Hz = 5s
        fleet.predict_all()
        fleet.update_with_distance_measurements()
        
        if i % 10 == 9:  # Toutes les secondes
            fleet.print_status()
    
    print("\n=== Distances finales ===")
    boats = fleet.get_all_boats()
    for i, boat1 in enumerate(boats):
        for boat2 in boats[i+1:]:
            dist = boat1.measure_distance_to(boat2)
            print(f"{boat1.id} <-> {boat2.id}: {dist}")
