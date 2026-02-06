#!/usr/bin/env python3
"""
Fake mavlink2rest simulator for local testing.

Starts one HTTP server per port. Each server simulates a boat and responds to
mavlink2rest-like endpoints used by utils/bblib.py and test_mavlinkrest.py.
"""

import argparse
import json
import random
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse

try:
    from utils.settings import POINT_BASE
except Exception:
    POINT_BASE = (48.1990, -3.0155)


EARTH_RADIUS_M = 6371000.0


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class BoatSim:
    def __init__(
        self,
        sysid=1,
        compid=1,
        base_lat=POINT_BASE[0],
        base_lon=POINT_BASE[1],
        idx=0,
        gps_noise_m=0.5,
        heading_noise_deg=5.0,
        heading_bias_deg=1.0,
    ):
        self.sysid = int(sysid)
        self.compid = int(compid)
        self.lat = base_lat + 0.00005 * idx
        self.lon = base_lon + 0.00005 * idx
        self.alt_m = 0.0
        self.yaw = 0.0  # radians
        self.armed = False
        self.mode_id = 0  # MANUAL
        self.ch1 = 1500  # throttle
        self.ch3 = 1500  # steering
        self.last_update = time.time()
        self.batt_v = 16.0
        self.batt_percent = 100
        self.gps_noise_m = float(gps_noise_m)
        self.heading_noise_deg = float(heading_noise_deg)
        self.heading_bias_deg = float(heading_bias_deg)
        self.lock = threading.Lock()

    def set_rc(self, ch1=None, ch3=None):
        with self.lock:
            if ch1 is not None:
                self.ch1 = int(ch1)
            if ch3 is not None:
                self.ch3 = int(ch3)

    def set_arm(self, state: bool):
        with self.lock:
            self.armed = bool(state)

    def set_mode(self, mode_id: int):
        with self.lock:
            self.mode_id = int(mode_id)

    def step(self, dt):
        with self.lock:
            # Simple motion model from RC channels
            if self.armed:
                # throttle: 1000..2000 -> speed -1..1 (m/s * 2)
                # Convention du projet: 1000 = forward, 2000 = reverse
                t_norm = clamp((1500 - self.ch1) / 500.0, -1.0, 1.0)
                speed = t_norm * 2.0  # m/s
                # steering: 1000..2000 -> turn rate -1..1 (rad/s)
                # 1000 = left, 2000 = right
                s_norm = clamp((self.ch3 - 1500) / 500.0, -1.0, 1.0)
                turn_rate = s_norm * 0.6
            else:
                speed = 0.0
                turn_rate = 0.0

            self.yaw += turn_rate * dt
            # Move in local frame
            import math
            dx = speed * dt
            dy = 0.0
            # Convert local meters to lat/lon degrees
            dlat = (dx * math.cos(self.yaw) - dy * math.sin(self.yaw)) / EARTH_RADIUS_M
            dlon = (dx * math.sin(self.yaw) + dy * math.cos(self.yaw)) / (EARTH_RADIUS_M * max(0.1, math.cos(math.radians(self.lat))))
            self.lat += math.degrees(dlat)
            self.lon += math.degrees(dlon)

            # Battery drain
            self.batt_v = max(12.0, self.batt_v - 0.0001 * abs(speed))
            self.batt_percent = max(0, int((self.batt_v - 12.0) / 4.0 * 100))

    def get_message(self, name: str):
        with self.lock:
            import math
            # Noisy GPS/heading (sensor-level)
            lat = self.lat
            lon = self.lon
            if self.gps_noise_m > 0:
                ang = random.uniform(0.0, 2.0 * math.pi)
                dist = random.gauss(0.0, self.gps_noise_m)
                dlat = (dist * math.cos(ang)) / EARTH_RADIUS_M
                dlon = (dist * math.sin(ang)) / (EARTH_RADIUS_M * max(0.1, math.cos(math.radians(lat))))
                lat = lat + math.degrees(dlat)
                lon = lon + math.degrees(dlon)
            yaw_deg = (math.degrees(self.yaw) + self.heading_bias_deg) % 360.0
            if self.heading_noise_deg > 0:
                yaw_deg = (yaw_deg + random.gauss(0.0, self.heading_noise_deg)) % 360.0
            if name == "ATTITUDE":
                return {"message": {"roll": 0.0, "pitch": 0.0, "yaw": float(math.radians(yaw_deg))}}
            if name == "GLOBAL_POSITION_INT":
                return {
                    "message": {
                        "lat": int(lat * 1e7),
                        "lon": int(lon * 1e7),
                        "alt": int(self.alt_m * 1000),
                        "hdg": int(yaw_deg * 100),
                    }
                }
            if name == "GPS_RAW_INT":
                return {
                    "message": {
                        "lat": int(lat * 1e7),
                        "lon": int(lon * 1e7),
                        "alt": int(self.alt_m * 1000),
                        "cog": int(yaw_deg * 100),
                    }
                }
            if name == "HEARTBEAT":
                base_mode_bits = 128 if self.armed else 0
                return {"message": {"custom_mode": self.mode_id, "base_mode": {"bits": base_mode_bits}}}
            if name == "BATTERY_STATUS":
                # Fake 4S battery
                cell_v = self.batt_v / 4.0
                return {
                    "message": {
                        "voltages": [int(cell_v * 1000)] * 4,
                        "current_battery": 0,
                        "battery_remaining": -1,
                    }
                }
            if name == "SYS_STATUS":
                return {
                    "message": {
                        "voltage_battery": int(self.batt_v * 1000),
                        "current_battery": 0,
                        "battery_remaining": -1,
                    }
                }
            return {"message": {}}


class Handler(BaseHTTPRequestHandler):
    server_version = "FakeMavlink2Rest/0.1"

    def do_GET(self):
        parsed = urlparse(self.path)
        parts = parsed.path.strip("/").split("/")
        if len(parts) >= 7 and parts[0] == "mavlink" and parts[1] == "vehicles" and parts[3] == "components" and parts[5] == "messages":
            msg_name = parts[6]
            payload = self.server.sim.get_message(msg_name)
            self._json(200, payload)
            return
        self._json(404, {"error": "not found"})

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path.strip("/") == "mavlink":
            length = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(length).decode("utf-8")
            try:
                data = json.loads(body)
            except Exception:
                self._json(400, {"error": "invalid json"})
                return
            message = data.get("message", {})
            msg_type = message.get("type")
            if msg_type == "RC_CHANNELS_OVERRIDE":
                ch1 = message.get("chan1_raw")
                ch3 = message.get("chan3_raw")
                self.server.sim.set_rc(ch1=ch1, ch3=ch3)
            elif msg_type == "COMMAND_LONG":
                cmd = message.get("command", {}).get("type")
                if cmd == "MAV_CMD_COMPONENT_ARM_DISARM":
                    self.server.sim.set_arm(bool(message.get("param1", 0)))
                elif cmd == "MAV_CMD_DO_SET_MODE":
                    self.server.sim.set_mode(int(message.get("param2", 0)))
            self._json(200, {"ok": True})
            return
        self._json(404, {"error": "not found"})

    def log_message(self, _format, *_args):
        return

    def _json(self, code, payload):
        data = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def run_server(host, port, sim):
    httpd = HTTPServer((host, port), Handler)
    httpd.sim = sim
    httpd.serve_forever()


def main():
    ap = argparse.ArgumentParser(description="Fake mavlink2rest simulator")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--base-port", type=int, default=6040)
    ap.add_argument("--count", type=int, default=1, help="Nombre de ports à lancer")
    ap.add_argument("--sysid-base", type=int, default=1)
    ap.add_argument("--compid", type=int, default=1)
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--gps-noise-m", type=float, default=0.5, help="Bruit GPS (écart-type, mètres)")
    ap.add_argument("--heading-noise-deg", type=float, default=5.0, help="Bruit cap (écart-type, degrés)")
    ap.add_argument("--heading-bias-deg", type=float, default=1.0, help="Biais cap (degrés)")
    args = ap.parse_args()

    sims = []
    for i in range(args.count):
        port = args.base_port + i
        sysid = args.sysid_base + i
        sim = BoatSim(
            sysid=sysid,
            compid=args.compid,
            idx=i,
            gps_noise_m=args.gps_noise_m,
            heading_noise_deg=args.heading_noise_deg,
            heading_bias_deg=args.heading_bias_deg,
        )
        sims.append(sim)
        t = threading.Thread(target=run_server, args=(args.host, port, sim), daemon=True)
        t.start()
        print(f"[FakeMavlink2Rest] Port {port} (sysid={sysid})")

    period = 1.0 / max(1.0, args.rate)
    try:
        while True:
            t0 = time.time()
            for sim in sims:
                sim.step(period)
            dt = time.time() - t0
            time.sleep(max(0.0, period - dt))
    except KeyboardInterrupt:
        print("\nArrêt simulateur.")


if __name__ == "__main__":
    main()
