#!/usr/bin/env python3
# BlueOS mavlink2rest control + telemetry helper
# - CH1 = throttle (1000 fwd, 2000 rev, 1500 neutral)
# - CH3 = steering (1000 left, 2000 right, 1500 center)
# - Inputs: left/right in [-250..250], mixed to (throttle, steering)
# - Telemetry: ATTITUDE (roll/pitch/yaw), GLOBAL_POSITION_INT (lat/lon), fallback to GPS_RAW_INT

import time
import math
import json
import argparse
import requests

class M2R:
    def __init__(self, host="192.168.2.202", port=6040, sysid=2, compid=1, timeout=3.0):
        self.base = f"http://{host}:{port}"
        self.post_url = f"{self.base}/mavlink"         # confirmed: /mavlink (no /v1)
        self.sysid = int(sysid)
        self.compid = int(compid)
        self.timeout = float(timeout)
        self.s = requests.Session()
        self.s.trust_env = False  # ignore proxies
        self.hdrs = {"Content-Type": "application/json", "Accept": "application/json", "Connection": "close"}

    # ---------- low-level ----------
    def _payload(self, message: dict):
        return {"header": {"system_id": 255, "component_id": 0, "sequence": 0}, "message": message}

    def post(self, message: dict):
        r = self.s.post(self.post_url, data=json.dumps(self._payload(message)),
                        headers=self.hdrs, timeout=self.timeout)
        if not r.ok:
            raise RuntimeError(f"POST {message.get('type')} -> {r.status_code} {r.text}")
        return r.text.strip()

    def get_msg(self, name: str):
        url = f"{self.base}/mavlink/vehicles/{self.sysid}/components/{self.compid}/messages/{name}"
        r = self.s.get(url, timeout=self.timeout)
        if not r.ok:
            return None
        try:
            return r.json()
        except Exception:
            return None

    # ---------- commands ----------
    def arm(self, state: bool):
        return self.post({
            "type": "COMMAND_LONG",
            "command": {"type": "MAV_CMD_COMPONENT_ARM_DISARM"},
            "param1": 1 if state else 0,
            "param2": 0, "param3": 0, "param4": 0, "param5": 0, "param6": 0, "param7": 0,
            "target_system": self.sysid, "target_component": self.compid, "confirmation": 0
        })

    def rc_override(self, ch1=None, ch3=None):
        body = {
            "type": "RC_CHANNELS_OVERRIDE",
            "target_system": self.sysid, "target_component": self.compid,
            "chan1_raw": 0, "chan2_raw": 0, "chan3_raw": 0, "chan4_raw": 0,
            "chan5_raw": 0, "chan6_raw": 0, "chan7_raw": 0, "chan8_raw": 0
        }
        if ch1 is not None: body["chan1_raw"] = int(ch1)
        if ch3 is not None: body["chan3_raw"] = int(ch3)
        return self.post(body)

    # ---------- mixing & mapping ----------
    @staticmethod
    def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

    def lr_to_ts(self, left: float, right: float, max_cmd=250.0):
        """
        Convert left/right in [-max_cmd..max_cmd] to normalized throttle/steering in [-1..1]
        Then map to PWM:
          - throttle CH1: 1000 (fwd) .. 1500 (neutral) .. 2000 (rev)  => pwm = 1500 - 500*t
          - steering CH3: 1000 (left) .. 1500 .. 2000 (right)        => pwm = 1500 + 500*s
        """
        L = self.clamp(float(left) / max_cmd, -1.0, 1.0)
        R = self.clamp(float(right) / max_cmd, -1.0, 1.0)
        t = self.clamp((L + R) / 2.0, -1.0, 1.0)  # forward/back
        s = self.clamp((R - L) / 2.0, -1.0, 1.0)  # right/left (positive = turn right)

        ch1 = int(round(1500 - 500 * t))  # invert per your mapping: +t --> forward -> 1000
        ch3 = int(round(1500 + 500 * s))  # +s -> right
        ch1 = self.clamp(ch1, 1000, 2000)
        ch3 = self.clamp(ch3, 1000, 2000)
        return ch1, ch3, t, s

    # ---------- high-level helpers ----------
    def drive_lr(self, left: float, right: float, seconds=3.0, rate_hz=10.0):
        ch1, ch3, t, s = self.lr_to_ts(left, right)
        dt = 1.0 / self.clamp(rate_hz, 1.0, 50.0)
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.rc_override(ch1=ch1, ch3=ch3)
            time.sleep(dt)
        # send neutral once
        self.rc_override(ch1=1500, ch3=1500)
        return {"ch1": ch1, "ch3": ch3, "throttle_norm": t, "steering_norm": s}

    # ---------- telemetry ----------
    def get_attitude(self):
        """Returns dict with roll, pitch, yaw (rad) and yaw_deg (0..360) if available."""
        j = self.get_msg("ATTITUDE")
        if not j or "message" not in j: return None
        m = j["message"]
        roll = float(m.get("roll", 0.0))
        pitch = float(m.get("pitch", 0.0))
        yaw = float(m.get("yaw", 0.0))
        yaw_deg = (math.degrees(yaw) + 360.0) % 360.0
        return {"roll": roll, "pitch": pitch, "yaw": yaw, "yaw_deg": yaw_deg}

    def get_gps(self):
        """
        Returns dict with lat, lon (deg), alt (m), hdg_deg if available.
        Tries GLOBAL_POSITION_INT then GPS_RAW_INT.
        """
        j = self.get_msg("GLOBAL_POSITION_INT")
        if j and "message" in j and j["message"].get("lat") not in (None, 0):
            m = j["message"]
            return {
                "lat": m["lat"] / 1e7,
                "lon": m["lon"] / 1e7,
                "alt_m": m.get("alt", 0) / 1000.0,
                "hdg_deg": (m.get("hdg", 0) / 100.0) if m.get("hdg") is not None else None
            }
        j = self.get_msg("GPS_RAW_INT")
        if j and "message" in j and j["message"].get("lat") not in (None, 0):
            m = j["message"]
            return {
                "lat": m["lat"] / 1e7,
                "lon": m["lon"] / 1e7,
                "alt_m": m.get("alt", 0) / 1000.0,
                "hdg_deg": (m.get("cog", 0) / 100.0) if m.get("cog") is not None else None
            }
        return None

def main():
    ap = argparse.ArgumentParser(description="Control motors via mavlink2rest and read GPS/IMU")
    ap.add_argument("--host", default="192.168.2.202")
    ap.add_argument("--port", type=int, default=6040)
    ap.add_argument("--sysid", type=int, default=2)
    ap.add_argument("--compid", type=int, default=1)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_arm = sub.add_parser("arm");   p_arm.add_argument("--state", type=int, choices=[0,1], default=1)
    p_neu = sub.add_parser("neutral")
    p_drv = sub.add_parser("drive")
    p_drv.add_argument("--left", type=float, required=True, help="left motor in [-250..250]")
    p_drv.add_argument("--right", type=float, required=True, help="right motor in [-250..250]")
    p_drv.add_argument("--secs", type=float, default=3.0)
    p_drv.add_argument("--rate", type=float, default=10.0)

    sub.add_parser("imu")
    sub.add_parser("gps")

    args = ap.parse_args()
    m2r = M2R(host=args.host, port=args.port, sysid=args.sysid, compid=args.compid)

    if args.cmd == "arm":
        print(m2r.arm(bool(args.state)))

    elif args.cmd == "neutral":
        print(m2r.rc_override(ch1=1500, ch3=1500))

    elif args.cmd == "drive":
        info = m2r.drive_lr(args.left, args.right, seconds=args.secs, rate_hz=args.rate)
        print(f"Sent CH1={info['ch1']} (throttle={info['throttle_norm']:.2f}), "
              f"CH3={info['ch3']} (steering={info['steering_norm']:.2f})")

    elif args.cmd == "imu":
        att = m2r.get_attitude()
        if att:
            print(f"roll={att['roll']:.4f} rad  pitch={att['pitch']:.4f} rad  yaw={att['yaw']:.4f} rad  "
                  f"yaw_deg={att['yaw_deg']:.2f}°")
        else:
            print("ATTITUDE not available yet.")

    elif args.cmd == "gps":
        gps = m2r.get_gps()
        if gps:
            print(f"lat={gps['lat']:.7f}  lon={gps['lon']:.7f}  alt={gps['alt_m']:.2f} m  "
                  f"hdg={gps['hdg_deg'] if gps['hdg_deg'] is not None else 'n/a'}°")
        else:
            print("No GPS fix yet (lat/lon==0).")

if __name__ == "__main__":
    # Safety first: test on stands, props clear.
    main()