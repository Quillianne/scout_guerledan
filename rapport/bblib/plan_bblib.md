# Plan — Section 3 : `bblib.tex`

> **Auteur :** Aurèle  
> **Objectif de la section :** Décrire les programmes développés pour permettre la communication avec les BlueBoat et offrir une interface de contrôle, ainsi que les tests de la première semaine de formation.

---

## 1. Contexte et motivation (1–1.5 pages)

### 1.1 Architecture matérielle et réseau

- 3 BlueBoat, chacun équipé d'un **Raspberry Pi embarqué** faisant tourner **BlueOS** (OS dédié de Blue Robotics).
- 1 station de base (laptop) connectée aux bateaux via un réseau **WiFi local** (192.168.2.x).
- Sur chaque bateau, un **Pixhawk** (contrôleur de vol) exécute le firmware **ArduRover** et communique avec le Raspberry Pi via USB/UART.

### 1.2 Qu'est-ce que MAVLink ?

- **MAVLink** (Micro Air Vehicle Link) est un protocole de communication léger, binaire et orienté messages, conçu pour les drones et robots autonomes.
- Il définit un ensemble de **messages standardisés** (ex. `ATTITUDE`, `GPS_RAW_INT`, `COMMAND_LONG`, `RC_CHANNELS_OVERRIDE`…) permettant à une station de contrôle d'interroger et de piloter un véhicule.
- Chaque message a un identifiant numérique, une structure binaire fixe, et est adressé par `system_id` / `component_id` pour identifier le véhicule et le composant cible.
- Dans l'usage classique, MAVLink est transmis sur un **socket UDP ou TCP** (port 14550 par défaut pour Mission Planner / QGroundControl) ou sur une liaison série.

### 1.3 Qu'est-ce que mavlink2rest ?

- **mavlink2rest** est un serveur léger (développé par Blue Robotics, embarqué dans BlueOS) qui fait le pont entre le flux MAVLink binaire du Pixhawk et une **API HTTP/REST**.
- Il écoute les messages MAVLink qui arrivent du Pixhawk et les stocke en mémoire (dernier message de chaque type).
- Il expose deux points d'entrée :
  - `GET /mavlink/vehicles/{sysid}/components/{compid}/messages/{MSG_NAME}` → retourne en **JSON** le dernier message reçu de ce type.
  - `POST /mavlink` avec un payload JSON → sérialise et injecte un message MAVLink dans le bus vers le Pixhawk.
- L'API est accessible depuis n'importe quel client HTTP sur le réseau local, sur le port 6040 de chaque bateau.

### 1.4 Pourquoi ne pas utiliser `pymavlink` directement ?

La bibliothèque Python officielle `pymavlink` permet d'ouvrir des connexions MAVLink bas niveau (UDP, TCP, série) et d'envoyer/recevoir des messages binaires. En théorie, c'est l'approche standard. Cependant, dans notre contexte, elle pose plusieurs problèmes :

| Critère | `pymavlink` (socket MAVLink brut) | `mavlink2rest` (HTTP/JSON) |
|---|---|---|
| **Couche réseau** | Socket UDP/TCP, gestion manuelle de la connexion, timeouts, reconnexion | Simple requête HTTP, sans état |
| **Accès simultané** | Un seul client à la fois (le socket est consommateur exclusif du flux) | Plusieurs clients simultanés (chaque GET lit le dernier message mis en cache) |
| **Pilotage depuis BlueOS** | Nécessite d'ouvrir un port UDP depuis BlueOS vers la station, configuration réseau non triviale avec BlueOS | Nativement exposé par BlueOS, aucune configuration supplémentaire |
| **Débogage** | Messages binaires, nécessite des outils spécialisés (Wireshark, logs pymavlink) | Requêtes testables directement dans un navigateur ou avec `curl` |
| **Dépendances** | `pymavlink` est une bibliothèque lourde avec des bindings C, plus complexe à installer | Seulement `requests`, bibliothèque standard Python |
| **Gestion multi-bateaux** | Nécessite N sockets distincts et N threads de réception | N instances indépendantes d'`MavlinkLink`, pas de threads réseau dédiés |

- En particulier, BlueOS **ne publie pas de socket UDP MAVLink par défaut** vers l'extérieur ; il est configuré pour n'exposer que l'API REST. Contourner cela aurait nécessité de modifier la configuration réseau de BlueOS sur chaque bateau.
- La contrainte de **multi-clients** est essentielle : pendant les tests, l'interface GUI, les scripts de mission et les logs tournent en parallèle sur la station de base, tous accédant aux données des bateaux.

### 1.5 Besoin d'une bibliothèque commune (`bblib`)

- Les scripts de mission, l'interface de contrôle et les tests partagent tous les mêmes opérations de base (lire le GPS, envoyer une commande moteur, etc.).
- Sans abstraction, chaque script réimplémenterait les requêtes HTTP, le parsing JSON MAVLink et la logique de conversion — source d'erreurs et de duplication.
- `bblib` centralise cette logique sous une **API orientée objet** cohérente, réutilisable et testable indépendamment des bateaux physiques (grâce au simulateur `fake_mavlink2rest.py`).

---

## 2. Architecture de `bblib.py` (2–3 pages)

### 2.1 Vue d'ensemble

Schéma de dépendances entre classes :

```
MavlinkLink
    ├── IMU
    ├── GPS ──── geo_conversion
    ├── MotorDriver
    └── Navigation (IMU + GPS + MotorDriver)

BlueBoatConfig ──► init_blueboat() ──► (MavlinkLink, IMU, GPS, MotorDriver, Navigation)
```

### 2.2 `MavlinkLink` — couche de communication REST

- Paramètres : `host`, `port` (6040), `sysid`, `compid`, `timeout`.
- **`get_message(msg_name)`** : requête `GET /mavlink/vehicles/{sysid}/components/{compid}/messages/{msg}` → JSON.
- **`post_message(message)`** : requête `POST /mavlink` avec le payload encapsulé dans un header MAVLink (system_id 255 = GCS).
- **`send_rc_override(ch1, ch3)`** : envoi d'une commande `RC_CHANNELS_OVERRIDE` pour piloter les moteurs via les canaux 1 (throttle) et 3 (steering).
- **`arm_disarm(arm)`** : envoi de `MAV_CMD_COMPONENT_ARM_DISARM`.
- **`set_flight_mode(mode_id/mode_name)`** : table de correspondance ArduRover (`MANUAL`, `GUIDED`, `AUTO`…).
- **`get_flight_mode()`** et **`get_battery_status()`** : lecture de `HEARTBEAT`, `BATTERY_STATUS`, `SYS_STATUS`.
- Gestion des erreurs silencieuse (option `silent_errors`) pour ne pas bloquer les threads de monitoring.

### 2.3 `IMU` — lecture d'attitude

- Source : message MAVLink **`ATTITUDE`** (filtre EKF embarqué).
- **`get_euler_angles()`** → `(roll, pitch, yaw)` en radians.
- **`get_attitude_dict()`** → dict enrichi avec `yaw_deg` normalisé dans [0°, 360°].
- Cache de la dernière valeur reçue (`_last_euler`) pour éviter un retour `None` lors des micro-coupures.

### 2.4 `GPS` — lecture de position

- Sources : **`GLOBAL_POSITION_INT`** (prioritaire) avec fallback sur **`GPS_RAW_INT`**.
- Conversion entiers MAVLink (×10⁻⁷) vers degrés décimaux.
- **`get_gps()`** : retourne `(lat, lon)` en degrés, historique horodaté dans `gps_history`.
- **`get_coords()`** : coordonnées **cartésiennes** locales `(x, y)` via `geo_conversion.conversion_spherique_cartesien()` (projection sphérique autour du `POINT_BASE` défini dans `settings.py`).
- **`export_gpx(filename)`** : export de la trace au format GPX pour visualisation sur SIG.

### 2.5 `MotorDriver` — contrôle des moteurs

- Conversion commande `(left, right)` ∈ [-max_cmd, max_cmd] → signaux PWM (1000–2000 µs) :
  - `throttle = (L + R) / 2`, `steering = (R − L) / 2`
  - CH1 (throttle) : 1500 − 500·t, CH3 (steering) : 1500 + 500·s
- **`send_cmd_motor(left, right)`** : envoi 1-shot (compatible ancienne API Arduino).
- **`drive_lr(left, right, seconds, rate_hz)`** : envoi répété pendant une durée, puis neutre automatique.
- **`stop_motors()`** : retour en position neutre (CH1=CH3=1500).
- Désarmement automatique dans `__del__` pour la sécurité.

### 2.6 `Navigation` — navigation autonome

- Régulateur proportionnel **P** sur l'erreur de cap (gain `Kp`, paramétrable dans `settings.py`).
- **`follow_heading(target_heading_deg, duration_s)`** : suivi de cap à vitesse constante.
- **`go_to_gps(target_coords, distance)`** : navigation vers un waypoint GPS avec loi de vitesse `tanh(d/d_ref)` (ralentissement à l'approche).
- **`go_to_position(target)`** : commande 1-shot vectorielle (intégration dans la boucle de l'algorithme de localisation).
- **`return_home()`** : séquence de waypoints prédéfinis vers la zone de départ de Guerlédan.
- **`stay_at(point)`** : maintien de position par rappel doux.

### 2.7 `BlueBoatConfig` et `init_blueboat()`

- **`boat_control_config.json`** : fichier de configuration extérieur (IP, port, sysid par bateau) pour ne pas coder les adresses en dur.
- **`BlueBoatConfig._load_config()`** : chargement avec valeurs par défaut (3 bateaux sur 192.168.2.201–203).
- **`init_from_config(boat_id)`** : instanciation complète d'un bateau (MavlinkLink, IMU, GPS, MotorDriver, Navigation) en une ligne.
- **`init_all()`** : initialise les N bateaux de la flotte et retourne un dict indexé par `boat_id`.

---

## 3. Interface graphique — `boat_control_gui.py` (1–1.5 pages)

### 3.1 Motivation

- Outil de supervision en temps réel pour l'équipe terrain pendant les tests.
- Construit entièrement sur `bblib` ; démontre que la bibliothèque couvre tous les cas d'usage.

### 3.2 `HeartbeatManager`

- Thread dédié envoyant un message `HEARTBEAT` (GCS) à 1 Hz vers tous les bateaux.
- Nécessaire : BlueOS désarme ou coupe la connexion si aucun GCS n'est détecté au bout de quelques secondes.

### 3.3 `BoatMonitor`

- Une instance par bateau : encapsule `MavlinkLink`, `GPS`, `IMU`, `MotorDriver`, `Navigation`.
- Thread de monitoring (polling ~2 Hz) : mise à jour de `gps_data`, `battery_data`, `heading`, `mode_data`.
- Calcul du pourcentage batterie à partir de la tension et du nombre de cellules détecté automatiquement (3S / 4S).

### 3.4 Interface Tkinter

- Panneau par bateau : GPS (lat/lon), cap (heading), batterie (tension + %), mode et état armé/désarmé.
- Boutons : **Armer**, **Désarmer**, **Retour Maison** (thread séparé via `Navigation.return_home()`).
- Chargement dynamique de la configuration depuis `boat_control_config.json` (IP modifiables sans recompiler).
- Gestion des connexions indisponibles : affichage dégradé sans blocage de l'interface.

---

## 4. Outil de simulation — `fake_mavlink2rest.py` (½ page)

- Serveur HTTP léger simulant l'API mavlink2rest pour **tests hors ligne**.
- Un serveur par port (= par bateau), simulant : GPS bruité, attitude, batterie, état armé/désarmé, mode.
- Permet de valider `bblib`, `boat_control_gui.py` et les scripts de mission **sans BlueBoat physique**.
- Utilisé lors de la semaine de formation pour tester rapidement les boucles de commande.

---

## 5. Tests de la première semaine de formation (1 page)

### 5.1 `test_bblib.py` — tests CLI de la bibliothèque

| Commande | Ce qui est testé |
|---|---|
| `imu` | Lecture `ATTITUDE` → roll/pitch/yaw |
| `gps` | Lecture `GLOBAL_POSITION_INT` / `GPS_RAW_INT` |
| `arm` / `disarm` | Armement via `COMMAND_LONG` |
| `battery` | Lecture tension / pourcentage |
| `neutral` | Envoi RC override neutre |
| `drive` | Commande moteurs pendant N secondes |
| `heading` | Suivi de cap P avec IMU |
| `goto` | Navigation GPS vers waypoint |
| `config` | Chargement de `boat_control_config.json` |

### 5.2 Autres scripts de test

- **`test_mavlinkrest.py`** : tests bas niveau du protocole REST (GET/POST) avec vérification des réponses HTTP.
- **`test_set_mode_manual.py`** : vérification de la commande de changement de mode.
- **`test_load_config.py`** : validation du parsing JSON de la configuration.
- **`test_multiple_boat.py`** : initialisation simultanée de plusieurs bateaux et polling en parallèle.
- **`test_formation_triangle.py`** : premier test de formation en triangle (validation de `go_to_position` sur 3 bateaux).

### 5.3 Résultats et observations

- *(À remplir après tests réels : latence REST observée, précision GPS, comportement du régulateur P, problèmes rencontrés et corrections apportées.)*
- Points à mentionner si pertinent :
  - Délai de la boucle REST (~50–100 ms par requête) → contrainte sur la fréquence de contrôle.
  - Dérive du cap par vent/courant → limites du simple régulateur P.
  - Robustesse aux coupures WiFi (reconnexion automatique, `silent_errors`).

---

## 6. Bilan (¼ page)

- La bibliothèque `bblib` fournit une **abstraction complète** de la communication MAVLink/REST.
- Elle a permis de développer rapidement l'interface GUI, les scripts de mission et l'intégration avec l'algorithme de localisation par intervalles.
- Le choix mavlink2rest s'est avéré judicieux : moins de complexité réseau, API testable directement dans un navigateur.

---

## Notes de rédaction

- **Figures à inclure :**
  - Schéma d'architecture (station ↔ WiFi ↔ BlueOS ↔ mavlink2rest ↔ bblib) — à créer.
  - Capture de `boat_control_gui.py` en fonctionnement (dossier `rapport/images/`).
  - Éventuellement un extrait de code commenté pour `MavlinkLink.get_message()` ou `Navigation.go_to_gps()`.
- **Longueur cible :** ~5–6 pages.
- **Style :** technique mais accessible ; ne pas détailler tout le code, privilégier les choix de conception.
