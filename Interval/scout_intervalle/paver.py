#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Paveur (Paver) pour le dead reckoning
Utilise un algorithme de pavage pour explorer l'espace des positions possibles
"""

from codac import *
from typing import List, Tuple


class Paver:
    """
    Classe pour paver l'espace des positions en utilisant des contractors
    """
    
    def __init__(self, precision: float = 0.1):
        """
        Initialise le paveur
        
        Args:
            precision: Précision du pavage (taille minimale des boîtes)
        """
        self.precision = precision
        self.boundary_boxes = []  # Boîtes frontières (solution possible)
    
    def pave(self, 
             initial_box: IntervalVector,
             contractor=None) -> List[IntervalVector]:
        """
        Pave l'espace avec un contracteur donné
        
        Args:
            initial_box: Box initial à paver (domaine de recherche)
            contractor: Fonction de contraction qui prend un IntervalVector et retourne le contracté
                       Si None, retourne simplement le box initial
            
        Returns:
            List[IntervalVector]: Liste des boîtes boundary (frontières)
        """
        self.boundary_boxes = []
        
        # Si pas de contracteur, retourner le box initial
        if contractor is None:
            return [initial_box]
        
        # Pile pour l'algorithme de pavage (SIVIA)
        stack = [initial_box.copy()]
        
        while stack:
            current_box = stack.pop()
            
            # Appliquer le contracteur
            contracted_box = current_box.copy()
            contracted_box = contractor(contracted_box)
            
            # Vérifier si la boîte est vide
            if contracted_box.is_empty():
                continue
            
            # Vérifier la taille de la boîte
            if current_box.max_diam() < self.precision:
                # Boîte assez petite, l'ajouter aux boundary
                self.boundary_boxes.append(contracted_box)
            else:
                # Bissecter la boîte et continuer
                # Bissecter sur la dimension la plus large
                largest_dim = 0
                max_diam = current_box[0].diam()
                for i in range(1, current_box.size()):
                    if current_box[i].diam() > max_diam:
                        max_diam = current_box[i].diam()
                        largest_dim = i
                
                # Vérifier que la dimension n'est pas dégénérée
                if current_box[largest_dim].diam() < 1e-10:
                    # Box trop petite, l'ajouter comme boundary
                    self.boundary_boxes.append(contracted_box)
                    continue
                
                bisected = current_box.bisect(largest_dim)
                stack.append(bisected[0])
                stack.append(bisected[1])
        
        return self.boundary_boxes
    
    def create_distance_contractor(self, 
                                   beacons: List[IntervalVector],
                                   distances: List[Interval]):
        """
        Crée un contracteur de distance pour les beacons donnés
        
        Args:
            beacons: Liste des positions des beacons
            distances: Liste des distances mesurées
            
        Returns:
            Fonction contracteur qui peut être utilisée avec pave()
        """
        ctc_dist = CtcDist()
        
        def contractor(x_box: IntervalVector) -> IntervalVector:
            """Contracte le box avec toutes les contraintes de distance"""
            result = x_box.copy()
            
            for beacon, distance in zip(beacons, distances):
                # Créer le box 5D pour la contrainte de distance
                box5d = IntervalVector(5)
                box5d[0] = result[0]  # x
                box5d[1] = result[1]  # y
                box5d[2] = beacon[0]  # beacon_x
                box5d[3] = beacon[1]  # beacon_y
                box5d[4] = distance  # distance mesurée
                
                # Contracter
                ctc_dist.contract(box5d)
                
                # Extraire la position contractée
                result[0] = box5d[0]
                result[1] = box5d[1]
                
                # Si vide, retourner immédiatement
                if result.is_empty():
                    return result
            
            return result
        
        return contractor
    
    def pave_with_motion(self,
                        initial_box: IntervalVector,
                        v_interval: Interval,
                        psi_interval: Interval,
                        dt: float,
                        contractor=None) -> List[IntervalVector]:
        """
        Pave l'espace après une étape de motion (translation)
        Optionnellement avec un contracteur
        
        Args:
            initial_box: Position initiale
            v_interval: Intervalle de vitesse
            psi_interval: Intervalle de cap
            dt: Pas de temps
            contractor: Contracteur optionnel à appliquer après motion
            
        Returns:
            List[IntervalVector]: Liste des boîtes boundary
        """
        # Calculer le box après translation
        vx_interval = v_interval * cos(psi_interval)
        vy_interval = v_interval * sin(psi_interval)
        
        dx = vx_interval * dt
        dy = vy_interval * dt
        
        # Créer le box après motion
        motion_box = initial_box.copy()
        motion_box[0] += dx
        motion_box[1] += dy
        
        # Paver avec le contracteur
        return self.pave(motion_box, contractor)
    
    def get_all_boxes(self) -> List[IntervalVector]:
        """
        Retourne toutes les boîtes
        """
        return self.boundary_boxes
    
    def get_bounding_box(self) -> IntervalVector:
        """
        Retourne la boîte englobante de toutes les solutions
        """
        all_boxes = self.get_all_boxes()
        if not all_boxes:
            return IntervalVector(2, Interval.EMPTY_SET)
        
        # Calculer l'union de toutes les boîtes
        bounding = all_boxes[0].copy()
        for box in all_boxes[1:]:
            bounding[0] |= box[0]
            bounding[1] |= box[1]
        
        return bounding


def pave_position(initial_box: IntervalVector,
                  beacons: List[IntervalVector],
                  distances: List[Interval],
                  precision: float = 0.1) -> List[IntervalVector]:
    """
    Fonction utilitaire pour paver rapidement une position avec des beacons
    
    Args:
        initial_box: Box de recherche initial
        beacons: Liste des positions des beacons
        distances: Liste des distances mesurées
        precision: Précision du pavage
        
    Returns:
        List[IntervalVector]: Liste des boîtes boundary
    """
    paver = Paver(precision=precision)
    contractor = paver.create_distance_contractor(beacons, distances)
    return paver.pave(initial_box, contractor)


if __name__ == "__main__":
    # Test du paveur
    print("Test du paveur\n")
    
    # Position initiale incertaine
    initial_box = IntervalVector([0.0, 0.0]).inflate(5.0)
    print(f"Box initial: {initial_box}")
    
    # Deux beacons
    beacon1 = IntervalVector([20.0, 20.0]).inflate(1.0)
    beacon2 = IntervalVector([40.0, 10.0]).inflate(1.0)
    
    # Distances mesurées
    d1 = Interval(23.3).inflate(1.0)  # Distance au beacon 1
    d2 = Interval(37.0).inflate(1.0)  # Distance au beacon 2
    
    print(f"Beacon 1: {beacon1}")
    print(f"Beacon 2: {beacon2}")
    print(f"Distance 1: {d1}")
    print(f"Distance 2: {d2}\n")
    
    # Paver
    paver = Paver(precision=0.5)
    contractor = paver.create_distance_contractor([beacon1, beacon2], [d1, d2])
    boundary = paver.pave(initial_box, contractor)
    
    print(f"Nombre de boîtes: {len(boundary)}")
    
    # Boîte englobante
    bounding = paver.get_bounding_box()
    print(f"\nBoîte englobante: {bounding}")
    print(f"  x: {bounding[0].mid():.2f} ± {bounding[0].diam()/2:.2f}")
    print(f"  y: {bounding[1].mid():.2f} ± {bounding[1].diam()/2:.2f}")
    
    # Visualisation avec VIBes
    print("\nVisualisation avec VIBes...")
    beginDrawing()
    fig = VIBesFigMap("Paver Test")
    fig.set_properties(100, 100, 800, 800)
    fig.axis_limits(-10, 50, -10, 50)
    
    # Dessiner les beacons
    fig.add_beacon(beacon1)
    fig.add_beacon(beacon2)
    
    # Dessiner les boîtes (limiter le nombre pour la performance)
    max_boxes_to_draw = 200
    for i, box in enumerate(boundary):
        if i >= max_boxes_to_draw:
            break
        fig.draw_box(box, "red[orange]")
    
    # Dessiner la boîte englobante
    fig.draw_box(bounding, "red[orange]")
    
    fig.show()
    
    print("Appuyez sur Ctrl+C pour fermer")
    import time
    try:
        time.sleep(100)
    except KeyboardInterrupt:
        print("\nFermeture")
