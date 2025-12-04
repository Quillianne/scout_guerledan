#!/usr/bin/env python3
# Test runner for utils/bblib.py
import argparse, time, sys, math

from utils.settings import KP, DT, MAX_CMD, BASE_SPEED_MULTIPLIER

from utils.bblib import (
    MavlinkLink, IMU, GPS,
    MotorDriver, Navigation, init_blueboat
)


def cmd_imu(imu):
    r,p,y = imu.get_euler_angles()
    if y is None:
        print("ATTITUDE non disponible (pas encore vue).")
    else:
        print(f"roll={r:.4f} rad, pitch={p:.4f} rad, yaw={y:.4f} rad, yaw_deg={(math.degrees(y)+360)%360:.2f}°")

def cmd_gps(gps):
    info = gps.get_gps_dict()
    if not info:
        print("Pas de fix GNSS (lat/lon==0).")
    else:
        hdg = info['hdg_deg'] if info['hdg_deg'] is not None else 'n/a'
        print(f"lat={info['lat']:.7f}, lon={info['lon']:.7f}, alt={info['alt_m']:.2f} m, hdg={hdg}")

def cmd_arm(mav, state):
    print("Arm/Disarm:", mav.arm_disarm(bool(state)))

def cmd_neutral(motors):
    print("Neutral:", motors.stop_motors())

def cmd_drive(mav, motors, left, right, secs, rate):
    # sécurité minimale
    # print("Setting manual mode…"); print(mav.set_mode_manual())
    print("Arming…"); print(mav.arm_disarm(True))
    print(f"Drive LR: left={left}, right={right}, secs={secs}, rate={rate} Hz")
    info = motors.drive_lr(left, right, seconds=secs, rate_hz=rate)
    print("Neutral:", motors.stop_motors())
    print("Disarming…"); print(mav.arm_disarm(False))
    print(f"PWM sent: CH1(throttle)={info['ch1']}, CH3(steering)={info['ch3']}, "
          f"t={info['throttle_norm']:.2f}, s={info['steering_norm']:.2f}")

def cmd_head(mav, nav, heading, secs):
    # print("Setting manual mode…"); print(mav.set_mode_manual())
    print("Arming…"); print(mav.arm_disarm(True))
    print(f"Follow heading {heading}° for {secs}s (max_speed={nav.max_speed:.1f})")
    nav.follow_heading(target_heading_deg=heading, duration_s=secs)
    print("Neutral:", nav.motor_driver.stop_motors())
    print("Disarming…"); print(mav.arm_disarm(False))

def cmd_gotohome(mav, nav):
    # print("Setting manual mode…"); print(mav.set_mode_manual())
    print("Arming…"); print(mav.arm_disarm(True))
    print(f"Returning Home…")
    nav.return_home()

def main():
    ap = argparse.ArgumentParser(description="Tester utils/bblib.py (BlueOS mavlink2rest)")
    ap.add_argument("--host", default="192.168.2.202")
    ap.add_argument("--port", type=int, default=6040)
    ap.add_argument("--sysid", type=int, default=2)
    ap.add_argument("--compid", type=int, default=1)
    ap.add_argument("--maxcmd", type=float, default=MAX_CMD)
    ap.add_argument("--dt", type=float, default=DT, help="pas de boucle pour Navigation/IMU (s)")
    ap.add_argument("--speed", type=float, default=BASE_SPEED_MULTIPLIER, help="multiplicateur de vitesse pour Navigation (0.0-1.0)")
    ap.add_argument("--kp", type=float, default=KP, help="gain Kp pour navigation")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("imu")
    sub.add_parser("gps")

    p_arm = sub.add_parser("arm")
    p_arm.add_argument("state", type=int, choices=[0,1], help="1=arm, 0=disarm")

    sub.add_parser("neutral")

    p_drv = sub.add_parser("drive")
    p_drv.add_argument("--left", type=float, required=True, help="[-250..250]")
    p_drv.add_argument("--right", type=float, required=True, help="[-250..250]")
    p_drv.add_argument("--secs", type=float, default=3.0)
    p_drv.add_argument("--rate", type=float, default=10.0)

    p_head = sub.add_parser("head")
    p_head.add_argument("--heading", type=float, required=True, help="degrés [0..360)")
    p_head.add_argument("--secs", type=float, default=5.0)

    p_home = sub.add_parser("home")

    args = ap.parse_args()
    mav, imu, gps, motors, nav = init_blueboat(args.host, args.port, args.sysid, args.compid, args.maxcmd, args.dt, args.kp, args.speed)

    if args.cmd == "imu":
        cmd_imu(imu)
    elif args.cmd == "gps":
        cmd_gps(gps)
    elif args.cmd == "arm":
        cmd_arm(mav, args.state)
    elif args.cmd == "neutral":
        cmd_neutral(motors)
    elif args.cmd == "drive":
        cmd_drive(mav, motors, args.left, args.right, args.secs, args.rate)
    elif args.cmd == "head":
        cmd_head(mav, nav, args.heading, args.secs)
    elif args.cmd == "home":
        cmd_gotohome(mav, nav)

if __name__ == "__main__":
    # ATTENTION: les commandes 'drive' et 'head' font bouger le bateau.
    # Teste d'abord 'imu', 'gps', 'neutral', 'arm 0/1' sur berceau.
    main()