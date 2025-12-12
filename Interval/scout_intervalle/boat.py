#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Classe Boat - Représente un bateau avec dead reckoning et capacités de communication
"""

from codac import *
import numpy as np


class Boat:
    """
    Classe représentant un bateau autonome avec dead reckoning
    """
    
    def __init__(self, 
                 boat_id: str,
                 initial_position: list,
                 initial_uncertainty: float = 1.0,
                 has_gps: bool = False):
        """
        Initialise un bateau
        
        Args:
            boat_id: Identifiant unique du bateau
            initial_position: Position initiale [x, y]
            initial_uncertainty: Incertitude initiale sur la position (m)
            has_gps: Si True, le bateau a accès au GPS
        """
        self.id = boat_id
        self.has_gps = has_gps
        
        # Position vraie (simulation)
        self.x_true = initial_position[0]
        self.y_true = initial_position[1]
        
        # Position estimée (box)
        self.x_box = IntervalVector([self.x_true, self.y_true]).inflate(initial_uncertainty)
        
        # Paramètres de mouvement (à définir avec set_motion)
        self.v_true = 0.0
        self.psi_true = 0.0
        self.v_interval = Interval(0.0)
        self.psi_interval = Interval(0.0)
        
        # Incertitudes
        self.v_uncertainty = 0.5  # m/s
        self.psi_uncertainty = 0.1  # rad
        self.distance_uncertainty = 1.0  # m pour les mesures de distance
        self.gps_uncertainty = 0.5  # m pour le GPS
        
        # Boîtes du pavage (pour visualisation)
        self.paved_boxes = []
        
        # Temps
        self.time = 0.0
        self.last_distance_measurement_time = 0.0
    
    def set_motion(self, v: float, psi: float):
        """
        Définit le mouvement du bateau
        
        Args:
            v: Vitesse vraie (m/s)
            psi: Cap vrai (rad)
        """
        self.v_true = v
        self.psi_true = psi
        self.v_interval = Interval(v).inflate(self.v_uncertainty)
        self.psi_interval = Interval(psi).inflate(self.psi_uncertainty)
    
    def predict(self, dt: float):
        """
        Étape de prédiction (dead reckoning) - 10 Hz
        
        Args:
            dt: Pas de temps (s)
        """
        # Mise à jour de la position vraie
        self.x_true += self.v_true * np.cos(self.psi_true) * dt
        self.y_true += self.v_true * np.sin(self.psi_true) * dt
        
        # Propagation de l'incertitude (translation du box)
        vx_interval = self.v_interval * cos(self.psi_interval)
        vy_interval = self.v_interval * sin(self.psi_interval)
        
        dx = vx_interval * dt
        dy = vy_interval * dt
        
        self.x_box[0] += dx
        self.x_box[1] += dy
        
        self.time += dt
    
    def update_from_gps(self):
        """
        Mise à jour avec GPS (seulement pour le bateau mère) - 1 Hz
        """
        if not self.has_gps:
            return
        
        # Mesure GPS avec incertitude
        gps_box = IntervalVector([self.x_true, self.y_true]).inflate(self.gps_uncertainty)
        
        # Intersection avec la position estimée
        self.x_box = self.x_box & gps_box
    
    def measure_distance_to(self, other_boat) -> Interval:
        """
        Mesure la distance à un autre bateau
        
        Args:
            other_boat: Autre instance de Boat
            
        Returns:
            Interval: Distance mesurée avec incertitude
        """
        # Distance vraie
        d_true = np.sqrt((self.x_true - other_boat.x_true)**2 + 
                        (self.y_true - other_boat.y_true)**2)
        
        # Retourner avec incertitude
        return Interval(d_true).inflate(self.distance_uncertainty)
    
    def get_position_box(self) -> IntervalVector:
        """
        Retourne la boîte de position estimée
        
        Returns:
            IntervalVector: Boîte de position [x, y]
        """
        return self.x_box.copy()
    
    def get_paved_boxes(self) -> list:
        """
        Retourne les boîtes du pavage
        
        Returns:
            list: Liste des IntervalVector du pavage
        """
        return self.paved_boxes
    
    def get_true_position(self) -> list:
        """
        Retourne la position vraie (pour la simulation)
        
        Returns:
            list: [x_true, y_true]
        """
        return [self.x_true, self.y_true]
    
    def get_uncertainty(self) -> tuple:
        """
        Retourne l'incertitude actuelle
        
        Returns:
            tuple: (uncertainty_x, uncertainty_y) en mètres
        """
        return (self.x_box[0].diam() / 2, self.x_box[1].diam() / 2)
    
    def __str__(self):
        """Représentation textuelle du bateau"""
        unc_x, unc_y = self.get_uncertainty()
        return (f"Boat {self.id}: "
                f"pos=({self.x_box[0].mid():.2f}±{unc_x:.2f}, "
                f"{self.x_box[1].mid():.2f}±{unc_y:.2f}) "
                f"true=({self.x_true:.2f}, {self.y_true:.2f})")


if __name__ == "__main__":
    # Test de la classe Boat
    print("Test de la classe Boat\n")
    
    # Créer un bateau
    boat = Boat("scout1", [0.0, 0.0], initial_uncertainty=1.0)
    boat.set_motion(v=5.0, psi=np.pi/4)  # 5 m/s, 45 degrés
    
    print(f"Initial: {boat}")
    
    # Simuler quelques pas de temps
    for i in range(10):
        boat.predict(0.1)  # 10 Hz
    
    print(f"After 1s: {boat}")
    
    # Créer un deuxième bateau
    boat2 = Boat("mother", [0.0, 0.0], has_gps=True)
    boat2.set_motion(v=3.0, psi=0.0)  # 3 m/s, 0 degrés
    
    for i in range(10):
        boat2.predict(0.1)
        if i == 9:
            boat2.update_from_gps()
    
    print(f"Boat2 after 1s with GPS: {boat2}")
    
    # Mesurer la distance
    distance = boat.measure_distance_to(boat2)
    print(f"\nDistance entre les bateaux: {distance}")
