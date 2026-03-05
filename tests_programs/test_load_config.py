#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Charge la configuration et lit une fois l'IMU de chaque bateau.
"""

import math
from utils.bblib import BlueBoatConfig


def main():
    cfg = BlueBoatConfig()
    boats = cfg.init_all()

    for boat_id, (_mav, imu, _gps, _motors, _nav) in boats.items():
        r, p, y = imu.get_euler_angles()
        if y is None:
            print(f"Bateau {boat_id}: IMU indisponible")
        else:
            yaw_deg = (math.degrees(y) + 360.0) % 360.0
            print(f"Bateau {boat_id}: roll={r:.3f} rad, pitch={p:.3f} rad, yaw={y:.3f} rad, yaw_deg={yaw_deg:.2f}°")


if __name__ == "__main__":
    main()