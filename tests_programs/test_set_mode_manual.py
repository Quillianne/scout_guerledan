#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Set flight mode for a boat loaded from config by --id.
"""

import argparse
from utils.bblib import BlueBoatConfig


def main():
    ap = argparse.ArgumentParser(description="Set flight mode for a boat from config.")
    ap.add_argument("--id", type=int, required=True, help="ID du bateau (config)")
    ap.add_argument("--mode-id", type=int, help="ID du mode (Rover)")
    ap.add_argument("--mode", help="Nom du mode (ex: MANUAL, GUIDED, RTL)")
    args = ap.parse_args()

    cfg = BlueBoatConfig()
    mav, _imu, _gps, _motors, _nav = cfg.init_from_config(args.id)

    result = mav.set_flight_mode(mode_id=args.mode_id, mode_name=args.mode)
    if result is None:
        print(f"Bateau {args.id}: échec set flight mode")
    else:
        print(f"Bateau {args.id}: mode demandé")


if __name__ == "__main__":
    main()