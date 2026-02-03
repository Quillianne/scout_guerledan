![LOGO](Logo_Scout.png)

# Scout Guerlédan - Projet de navigation autonome multi-bateaux

## Présentation du projet

Ce projet vise à développer un système de contrôle et de navigation pour une flotte de bateaux autonomes (BlueBoat) évoluant sur le lac de Guerlédan. Le système permet de coordonner plusieurs scouts en formation, avec un bateau principal (MotherShip) et des bateaux suiveurs (ScoutA, ScoutB) qui maintiennent une formation géométrique (triangle équilatéral).

**Objectifs principaux :**

- Estimation d'état par méthodes ensemblistes (intervalles)
- Localisation en swarm
- Contrôle autonome d'USVs en swarm

## Installation et utilisation

### Prérequis

- Python 3.8+
- Bibliothèques Python :
  - `numpy==2.3.3` - Calculs numériques et manipulation de tableaux
  - `pymavlink==2.4.49` - Communication protocole MAVLink avec les bateaux
  - `requests==2.32.5` - Requêtes HTTP pour l'API REST
  - `matplotlib==3.10.7` - Visualisation et tracé de trajectoires
  - `codac==2.0.0.dev23` - Bibliothèque pour les calculs par intervalles et méthodes ensemblistes

- (optionnel) **VIBes-viewer** pour l'affichage des boîtes en simulation d'intervalles (via Codac)

### Installation

1. Cloner le dépôt :
```bash
git clone https://github.com/Quillianne/scout_guerledan.git
cd scout_guerledan
```

2. Installer les dépendances :
```bash
pip install -r requirements.txt
```

3. (Optionnel) Installer VIBes-viewer si vous souhaitez l'affichage des boîtes :
```bash
# Releases : https://github.com/ENSTABretagneRobotics/VIBES/releases
```
La connexion au viewer se fait directement via Codac (pas de bibliothèque Python dédiée).

### Utilisation

#### 1. Interface graphique de contrôle
Lancer l'interface pour monitorer et contrôler les bateaux :
```bash
python boat_control_gui.py
```

L'interface permet de :
- Visualiser position GPS, batterie, cap
- Armer/désarmer les bateaux
- Déclencher le retour maison
- Modifier IP/port/sysid via “Modifier config”
- Maintenir les heartbeats pendant l'exécution

Le heartbeat évite le désarmement de sécurité : auparavant, si on lançait un script puis qu'on le fermait et relançait plus tard, les bateaux restaient désarmés car ils ne recevaient plus d'info. Le maintien du heartbeat prévient ce cas.

#### 2. Heartbeat (maintien de connexion)
```bash
python heartbeat.py --targets 192.168.2.201:1,192.168.2.202:2,192.168.2.203:3
```

#### 3. Tests de formation en triangle
Tester la formation géométrique avec bateaux réels :
```bash
python test_formation_triangle.py
```

#### 4. Simulation (ancienne, simpliste)
Tester des algorithmes de base en simulation (historiquement utilisés la 1ʳᵉ semaine) :
```bash
python Sim/Simulation.py
```

#### 5. Simulation avec intervalles + VIBes-viewer
Simulation de flotte + affichage VIBes-viewer (Codac) :
```bash
python test_display.py
```

#### 6. Visualisation de trajectoires
Analyse et GIF à partir des fichiers `testcoordsA.npy` / `testcoordsB.npy` :
```bash
python plot_trajectories_optimized.py
```


## Structure du projet

### Dossiers principaux

#### `Sim/`
**Ancien** : simulateur très simpliste utilisé la première semaine pour visualiser certains comportements.

- `Boat.py` : Modèle cinématique de bateau
- `Controller.py` : Contrôleurs (cap-vers-point, cap-constant, etc.)
- `Path_planner.py` : Planification des points de passage pour formations
- `Simulation.py` : Moteur de simulation avec visualisation matplotlib
- `README.md` : Détails des algorithmes de planification (compute_target_points*)



#### `utils/`
Modules utilitaires pour la communication et la gestion des bateaux.

- `bblib.py` : Bibliothèque principale pour communication MAVLink avec BlueBoat
- `geo_conversion.py` : Conversions géographiques (WGS84 ↔ NED)
- `interval.md` : Documentation des contracteurs équivalents / formules
- `settings.py` : Configuration globale du projet
- `prediction.py` : Algorithmes de prédiction et estimation de trajectoires
- `vibes_display.py` : Affichage et visualisation avec VIBes-viewer


### Scripts principaux

- `boat_control_gui.py` : Interface graphique de monitoring et contrôle des bateaux
- `heartbeat.py` : Gestion des heartbeats MAVLink pour maintenir les connexions
- `plot_trajectories_optimized.py` : Analyse et GIF des trajectoires à partir de `testcoordsA.npy` / `testcoordsB.npy`

Scripts de test pour différentes fonctionnalités :
  - `test_bblib.py` : Tests de la bibliothèque MAVLink
  - `test_formation_triangle.py` : Test de formation en triangle
  - `test_mavlinkrest.py` : Tests API REST MAVLink
  - `test_multiple_boat.py` : Tests multi-bateaux

### Fichiers d'analyse

- `trajectoires_analyse_complete.png` : Visualisation complète des trajectoires
- `box_sizes_escape.png`, `box_sizes_no_escape.png` : Analyses de la taille des boîtes englobantes
- `compute_times_reset.png`, `compute_times_no_reset.png` : Analyses des temps de calcul



## Ressources

- Vidéos de démonstration : https://drive.google.com/drive/folders/1pT9_0SwZHuyaDzCsY3-wl783lb_Jlr6z?usp=drive_link
- Documentation Codac : https://codac.io/
- Protocole MAVLink : https://mavlink.io/

## 👥 Contributeurs

- Kilian BARANTAL
- Ewen MELE
- Aurèle PLANCHARD
