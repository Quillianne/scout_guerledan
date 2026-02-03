#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Set mode MANUAL for a boat loaded from config by --id.
"""

import argparse
from utils.bblib import BlueBoatConfig


def main():
    ap = argparse.ArgumentParser(description="Set MANUAL mode for a boat from config.")
    ap.add_argument("--id", type=int, required=True, help="ID du bateau (config)")
    args = ap.parse_args()

    cfg = BlueBoatConfig()
    mav, _imu, _gps, _motors, _nav = cfg.init_from_config(args.id)

    result = mav.set_mode_manual()
    if result is None:
        print(f"Bateau {args.id}: échec set mode MANUAL")
    else:
        print(f"Bateau {args.id}: mode MANUAL demandé")


if __name__ == "__main__":
    main()