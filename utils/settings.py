# settings.py
import os

CALIBRATION_FILE = os.path.join(os.path.dirname(__file__),"imu_calibration.npz")
GYRO_CALIBRATION_FILE = os.path.join(os.path.dirname(__file__),"gyro_calibration.npz")
DT = 0.05
POINT_BOUEE = (48.20010, -3.01573)
POINT_BASE = (48.1996872, -3.0153766)
RHO = 6371000
FREQUENCE_CIRCLE = 1/120
RAYON_CIRCLE = 20

