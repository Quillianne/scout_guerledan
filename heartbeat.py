# heartbeat.py
# Envoie des HEARTBEAT MAVLink (type GCS) via mavlink2rest à plusieurs cibles.
# Cibles par défaut :
#   - 192.168.2.202 (sysid=2)
#   - 192.168.2.203 (sysid=3)

import time
import argparse
import signal
import sys

from utils.bblib import MavlinkLink  # import direct, pas d'option --module

# ---------------------------------------------------------------------
# Construction du message HEARTBEAT (format mavlink2rest)
# ---------------------------------------------------------------------
def build_heartbeat_message():
    """
    Retourne le dict 'message' pour mavlink2rest :
      - message['type'] = 'HEARTBEAT' (nom du message)
      - le champ MAV_TYPE est exposé sous 'mavtype'
    """
    return {
        "type": "HEARTBEAT",
        "custom_mode": 0,
        "mavtype": {"type": "MAV_TYPE_GCS"},            # se présenter comme un GCS
        "autopilot": {"type": "MAV_AUTOPILOT_INVALID"},
        "base_mode": {"bits": 0},
        "system_status": {"type": "MAV_STATE_ACTIVE"},
        "mavlink_version": 2
    }

# ---------------------------------------------------------------------
# Parse de la liste des cibles "ip:sysid"
# ---------------------------------------------------------------------
def parse_targets(targets_arg: str):
    targets = []
    for item in targets_arg.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" not in item:
            raise ValueError(f"Cible invalide '{item}', attendu 'IP:SYSID' (ex: 192.168.2.202:2)")
        ip, sysid = item.split(":", 1)
        targets.append((ip.strip(), int(sysid)))
    return targets

# ---------------------------------------------------------------------
# Boucle principale d'envoi
# ---------------------------------------------------------------------
def run(targets, hz: float, port: int, compid: int, timeout: float):
    links = [MavlinkLink(host=ip, port=port, sysid=sysid, compid=compid, timeout=timeout)
             for (ip, sysid) in targets]

    period = 1.0 / max(0.2, hz)
    hb = build_heartbeat_message()

    print(f"[INFO] Envoi HEARTBEAT à {hz:.2f} Hz vers :")
    for ip, sysid in targets:
        print(f"       - {ip} (sysid={sysid})")

    stop = False
    def handle_sig(_sig, _frm):
        nonlocal stop
        stop = True
        print("\n[INFO] Arrêt demandé, fin propre…")

    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)

    while not stop:
        t0 = time.time()
        for link in links:
            try:
                link.post_message(hb)
            except Exception as e:
                print(f"[WARN] Échec envoi HEARTBEAT vers {link.base}: {e}")
        # maintenir la cadence
        dt = time.time() - t0
        time.sleep(max(0.0, period - dt))

    print("[OK] Heartbeat stoppé.")

# ---------------------------------------------------------------------
# Entrée CLI
# ---------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Envoi périodique de HEARTBEAT MAVLink (GCS) via mavlink2rest.")
    parser.add_argument("--targets", default="192.168.2.202:2,192.168.2.203:3",
                        help="Liste des cibles 'IP:SYSID' séparées par des virgules.")
    parser.add_argument("--hz", type=float, default=1.0, help="Fréquence des heartbeat (Hz).")
    parser.add_argument("--port", type=int, default=6040, help="Port mavlink2rest BlueOS (par défaut 6040).")
    parser.add_argument("--compid", type=int, default=1, help="COMPID de l'autopilote (souvent 1).")
    parser.add_argument("--timeout", type=float, default=3.0, help="Timeout HTTP (s).")
    args = parser.parse_args()

    try:
        targets = parse_targets(args.targets)
    except ValueError as e:
        print(f"[ERR] {e}")
        sys.exit(2)

    run(targets, args.hz, args.port, args.compid, args.timeout)