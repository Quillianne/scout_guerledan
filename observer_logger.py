"""
Test de l'observateur par intervalles avec 3 bateaux

Ce script initialise 3 bateaux et les laisse à l'arrêt.
Ils seront contrôlés à la télécommande RC.
On utilise ensuite vibes_display.py pour estimer leur positions avec des intervalles.
"""

import time
import math
import logging
import argparse
import ast
import re
import os
import matplotlib.pyplot as plt
from datetime import datetime
from codac import Interval
import threading

from utils.bblib import BlueBoatConfig
from utils.fleet_prediction import FleetPredictor


# ----------------------------------------------------------------------------
# Constant uncertainties 
# ----------------------------------------------------------------------------


INIT_UNCERTAINTY = 0.
GPS_UNCERTAINTY = 0.
DIST_UNCERTAINTY = 0.1
MOVE_UNCERTAINTY = 0.5

#INIT_UNCERTAINTY = 1.0
#GPS_UNCERTAINTY = 1.0
#DIST_UNCERTAINTY = 1.0
#MOVE_UNCERTAINTY = 1.0

# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------

def true_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def box_size_str(box: IntervalVector) -> str:
    return f"dx={box[0].diam():.3f}, dy={box[1].diam():.3f}"

def setup_logging():
    """Configure logging files for live mode only."""
    log_dir = "logs"
    log_latest = "test_observer.log"

    os.makedirs(log_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(log_dir, f"test_observer_{timestamp}.log")

    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")

    latest_handler = logging.FileHandler(log_latest, mode="w", encoding="utf-8")
    latest_handler.setFormatter(formatter)
    logger.addHandler(latest_handler)

    timestamped_handler = logging.FileHandler(log_path, mode="w", encoding="utf-8")
    timestamped_handler.setFormatter(formatter)
    logger.addHandler(timestamped_handler)

# -------------------------------------------------------------------------------
# Main
# -------------------------------------------------------------------------------
def parse_log_line(line: str):
    """Parse one log line into (timestamp, coords1, coords2, coords3)."""
    try:
        parts = line.strip().split(" - ", 2)
        if len(parts) < 3:
            return None
        ts = datetime.strptime(parts[0], "%Y-%m-%d %H:%M:%S,%f")
        msg = parts[2]
        msg = re.sub(r"Boat\s*\d+:", "", msg)
        nums = re.findall(r"np\.float64\(([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\)", msg)
        if len(nums) < 6:
            return None
        values = list(map(float, nums[:6]))
        c1 = [values[0], values[1]]
        c2 = [values[2], values[3]]
        c3 = [values[4], values[5]]
        return ts, c1, c2, c3
    except Exception:
        return None


def run_replay(log_path: str, speed: float, only_plot: bool = False, downsample: int = 1):
    entries = []
    with open(log_path, "r", encoding="utf-8") as f:
        for line in f:
            parsed = parse_log_line(line)
            if parsed:
                entries.append(parsed)

    if not entries:
        print("Aucune entrée valide dans le fichier de replay.")
        return

    if downsample and downsample > 1:
        entries = entries[::downsample]
        if not entries:
            print("Aucune entrée après downsample.")
            return

    # Initialisation du fleet predictor et de l'affichage VIBes
    _ts, coords1, coords2, coords3 = entries[0]
    fleet = FleetPredictor(
        initial_positions=[coords1, coords2, coords3],
        recursive=False,
        init_uncertainty=INIT_UNCERTAINTY,
        gps_uncertainty=GPS_UNCERTAINTY,
        dist_uncertainty=DIST_UNCERTAINTY,
        move_uncertainty=MOVE_UNCERTAINTY,
        precision=1.0,
        margin=10.0,
    )
    if only_plot:
        fleet.display = None

    prev_ts, prev_coords1, prev_coords2, prev_coords3 = entries[0]
    steps = []
    box1_sizes = []
    box2_sizes = []
    box3_sizes = []
    step_idx = 0

    try:
        for ts, coords1, coords2, coords3 in entries[1:]:
            # Dead reckoning (mouvement)
            dx1 = coords1[0] - prev_coords1[0]
            dy1 = coords1[1] - prev_coords1[1]
            dx2 = coords2[0] - prev_coords2[0]
            dy2 = coords2[1] - prev_coords2[1]
            dx3 = coords3[0] - prev_coords3[0]
            dy3 = coords3[1] - prev_coords3[1]

            d12 = true_distance(coords1, coords2)
            d13 = true_distance(coords1, coords3)
            d23 = true_distance(coords2, coords3)

            fleet.update(
                mothership_pos=coords1,
                distances=[d12, d23, d13],
                movements=[(dx1, dy1), (dx2, dy2), (dx3, dy3)],
            )

            fleet.draw([coords1, coords2, coords3])

            steps.append(step_idx)
            boxes = fleet.get_boxes()
            box1_sizes.append(max(boxes[0][0].diam(), boxes[0][1].diam()))
            box2_sizes.append(max(boxes[1][0].diam(), boxes[1][1].diam()))
            box3_sizes.append(max(boxes[2][0].diam(), boxes[2][1].diam()))
            step_idx += 1

            dt = (ts - prev_ts).total_seconds()
            if speed <= 0:
                speed = 1.0
            if fleet.display is not None:
                time.sleep(max(0.0, dt / speed))

            prev_ts = ts
            prev_coords1, prev_coords2, prev_coords3 = coords1, coords2, coords3

    except KeyboardInterrupt:
        print("\n[Replay] Interruption utilisateur détectée.")
    finally:
        if steps:
            plt.figure("Box sizes (replay)")
            plt.plot(steps, box1_sizes, label="Boat 1")
            plt.plot(steps, box2_sizes, label="Boat 2")
            plt.plot(steps, box3_sizes, label="Boat 3")
            plt.xlabel("Step")
            plt.ylabel("Box size (max diameter)")
            plt.legend()
            plt.tight_layout()
            plt.show()


def run_live():
    """
    Initialise 3 bateaux et les laisse à l'arrêt.
    """
    setup_logging()
    print("=" * 60)
    print("Initialisation des 3 bateaux...")
    print("=" * 60)

    # Load configuration for boats
    config = BlueBoatConfig()

    # Initialize boats using the configuration
    print("\n[Bateau 1] Initialisation via configuration...")
    mav1, imu1, gps1, motors1, nav1 = config.init_from_config(boat_id=1)
    print("[Bateau 1] ✓ Initialisé")

    print("\n[Bateau 2] Initialisation via configuration...")
    mav2, imu2, gps2, motors2, nav2 = config.init_from_config(boat_id=2)
    print("[Bateau 2] ✓ Initialisé")

    print("\n[Bateau 3] Initialisation via configuration...")
    mav3, imu3, gps3, motors3, nav3 = config.init_from_config(boat_id=3)
    print("[Bateau 3] ✓ Initialisé")

    print("\n" + "=" * 60)
    print("Les 3 bateaux sont initialisés et à l'arrêt.")
    print("Appuyez sur Ctrl+C pour quitter.")
    print("=" * 60)

    # ------------------------------------------------------------------
    # Attendre une première position GPS valide pour chaque bateau
    # ------------------------------------------------------------------
    print("\nAttente des positions GPS initiales...")
    while True:
        coords1 = gps1.get_coords()
        coords2 = gps2.get_coords()
        coords3 = gps3.get_coords()
        #print(coords2,coords3)

        if (
            coords1[0] is not None and coords1[1] is not None
            and coords2[0] is not None and coords2[1] is not None
            and coords3[0] is not None and coords3[1] is not None
        ):
            break
        time.sleep(0.5)
    
    # ------------------------------------------------------------------
    # Initialisation du fleet predictor et de l'affichage VIBes
    # ------------------------------------------------------------------
    fleet = FleetPredictor(
        initial_positions=[coords1, coords2, coords3],
        recursive=True,
        init_uncertainty=INIT_UNCERTAINTY,
        gps_uncertainty=GPS_UNCERTAINTY,
        dist_uncertainty=DIST_UNCERTAINTY,
        move_uncertainty=MOVE_UNCERTAINTY,
        precision=1.0,
        margin=10.0,
    )

    # ------------------------------------------------------------------
    # Boucle principale d'affichage
    # ------------------------------------------------------------------
    data_lock = threading.Lock()
    latest_coords = {"c1": coords1, "c2": coords2, "c3": coords3}

    def logging_thread():
        period = 1.0
        next_time = time.monotonic()
        while True:
            t0 = time.monotonic()
            new_coords1 = gps1.get_coords()
            new_coords2 = gps2.get_coords()
            new_coords3 = gps3.get_coords()

            with data_lock:
                if new_coords1[0] is not None and new_coords1[1] is not None:
                    latest_coords["c1"] = new_coords1
                if new_coords2[0] is not None and new_coords2[1] is not None:
                    latest_coords["c2"] = new_coords2
                if new_coords3[0] is not None and new_coords3[1] is not None:
                    latest_coords["c3"] = new_coords3

                c1 = latest_coords["c1"]
                c2 = latest_coords["c2"]
                c3 = latest_coords["c3"]

            logging.info(f"Boat 1: {c1}, Boat 2: {c2}, Boat 3: {c3}")

            next_time += period
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_time = time.monotonic()

    def display_thread():
        period = 5.0
        next_time = time.monotonic()
        with data_lock:
            prev_coords1 = latest_coords["c1"]
            prev_coords2 = latest_coords["c2"]
            prev_coords3 = latest_coords["c3"]
        while True:
            with data_lock:
                new_coords1 = latest_coords["c1"]
                new_coords2 = latest_coords["c2"]
                new_coords3 = latest_coords["c3"]

            coords1 = new_coords1 if new_coords1[0] is not None and new_coords1[1] is not None else prev_coords1
            coords2 = new_coords2 if new_coords2[0] is not None and new_coords2[1] is not None else prev_coords2
            coords3 = new_coords3 if new_coords3[0] is not None and new_coords3[1] is not None else prev_coords3

            # Dead reckoning (mouvement)
            dx1 = coords1[0] - prev_coords1[0]
            dy1 = coords1[1] - prev_coords1[1]
            dx2 = coords2[0] - prev_coords2[0]
            dy2 = coords2[1] - prev_coords2[1]
            dx3 = coords3[0] - prev_coords3[0]
            dy3 = coords3[1] - prev_coords3[1]

            d12 = true_distance(coords1, coords2)
            d13 = true_distance(coords1, coords3)
            d23 = true_distance(coords2, coords3)

            fleet.update(
                mothership_pos=coords1,
                distances=[d12, d23, d13],
                movements=[(dx1, dy1), (dx2, dy2), (dx3, dy3)],
            )

            # Affichage des pavages avec VIBes
            fleet.draw([coords1, coords2, coords3])

            prev_coords1, prev_coords2, prev_coords3 = coords1, coords2, coords3

            # Attente avant la prochaine mise à jour
            next_time += period
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_time = time.monotonic()

    # Start threads
    logging_thread = threading.Thread(target=logging_thread, daemon=True)
    display_thread = threading.Thread(target=display_thread, daemon=True)

    logging_thread.start()
    display_thread.start()

    # Keep the main thread alive
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[Main] Interruption utilisateur détectée. Données sauvegardées dans 'boat_data.log'.")

    finally:
        # S'assurer que tous les moteurs sont arrêtés
        print("\n[Main] Arrêt des moteurs...")
        motors1.stop_motors()
        motors2.stop_motors()
        motors3.stop_motors()
        print("[Main] ✓ Tous les moteurs sont arrêtés.")


def main():
    ap = argparse.ArgumentParser(description="Observateur par intervalles (live ou replay).")
    ap.add_argument("--replay", help="Fichier de log à rejouer")
    ap.add_argument("--speed", type=float, default=1.0, help="Vitesse de replay (ex: 2 = x2)")
    ap.add_argument("--only-plot", action="store_true", help="Mode replay sans VIBes, plot uniquement")
    ap.add_argument("--downsample", type=int, default=1, help="Downsample (ex: 2 = 1 point sur 2)")
    args = ap.parse_args()

    if args.replay:
        run_replay(args.replay, args.speed, only_plot=args.only_plot, downsample=args.downsample)
    else:
        run_live()


if __name__ == "__main__":
    main()