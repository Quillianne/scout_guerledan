import json
import re
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from django.http import JsonResponse, HttpResponseBadRequest
from django.shortcuts import render
from django.views.decorators.csrf import csrf_exempt

# Make repo root importable (for utils.*)
REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from utils.geo_conversion import conversion_cartesien_spherique
from utils.fleet_prediction import FleetPredictor
from utils.bblib import BlueBoatConfig, MavlinkLink, GPS
from codac import PavingOut

LOG_PATTERN = re.compile(r"np\.float64\(([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\)")
CONFIG_PATH = REPO_ROOT / "boat_control_config.json"
GPS_CACHE = {}
BOAT_CACHE = {}
HOME_STATE = {}
INTERVAL_STATE = {
    "fleet": None,
    "prev_coords": None,
    "settings": None,
    "log_enabled": False,
    "log_file": None,
}


def index(_request):
    return render(_request, "replay/home.html")


def logs_view(_request):
    return render(_request, "replay/index.html")


def manager_view(_request):
    return render(_request, "replay/manager.html")


def interval_view(_request):
    return render(_request, "replay/interval.html")


def list_log_files():
    log_dir = REPO_ROOT / "logs"
    if not log_dir.exists():
        return []
    logs = [p.name for p in log_dir.iterdir() if p.is_file() and p.name.endswith(".log")]
    return sorted(logs, reverse=True)


def api_logs(_request):
    return JsonResponse({"logs": list_log_files()})


@csrf_exempt
def api_logs_delete(request):
    if request.method != "POST":
        return HttpResponseBadRequest("POST required")
    try:
        body = json.loads(request.body.decode("utf-8"))
    except Exception:
        return HttpResponseBadRequest("Invalid JSON")
    name = body.get("log")
    if not name:
        return HttpResponseBadRequest("Missing log")
    log_path = REPO_ROOT / "logs" / name
    if not log_path.exists():
        return HttpResponseBadRequest("Log not found")
    log_path.unlink()
    return JsonResponse({"ok": True})


def parse_log_line(line: str):
    try:
        parts = line.strip().split(" - ", 2)
        if len(parts) < 3:
            return None
        ts = datetime.strptime(parts[0], "%Y-%m-%d %H:%M:%S,%f")
        msg = parts[2]
        msg = re.sub(r"Boat\s*\d+:", "", msg)
        nums = LOG_PATTERN.findall(msg)
        if len(nums) < 6:
            return None
        values = list(map(float, nums[:6]))
        c1 = [values[0], values[1]]
        c2 = [values[2], values[3]]
        c3 = [values[4], values[5]]
        return ts, c1, c2, c3
    except Exception:
        return None


def cartesian_to_gps(point_xy):
    lat, lon = conversion_cartesien_spherique(point_xy)
    return [float(lat), float(lon)]


def box_to_polygon_gps(box):
    xmin = float(box[0].lb())
    xmax = float(box[0].ub())
    ymin = float(box[1].lb())
    ymax = float(box[1].ub())
    corners = [
        (xmin, ymin),
        (xmax, ymin),
        (xmax, ymax),
        (xmin, ymax),
        (xmin, ymin),
    ]
    return [cartesian_to_gps(c) for c in corners]


def paving_to_polygons_gps(paving):
    if paving is None:
        return []
    try:
        boxes = paving.boxes(PavingOut.outer)
    except Exception:
        return []
    return [box_to_polygon_gps(b) for b in boxes]


def _load_config():
    cfg = BlueBoatConfig(str(CONFIG_PATH))
    return cfg.boats


def _save_config(boats):
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(boats, f, indent=2, ensure_ascii=False)


def _get_gps_client(item):
    key = (item.get("boat_id"), item.get("host"), item.get("port"), item.get("sysid"), item.get("compid", 1))
    if key in GPS_CACHE:
        return GPS_CACHE[key]
    mav = MavlinkLink(
        host=item.get("host"),
        port=item.get("port", 6040),
        sysid=item.get("sysid", item.get("boat_id")),
        compid=item.get("compid", 1),
    )
    gps = GPS(mav)
    GPS_CACHE[key] = gps
    return gps


def _get_boat_components(boat_id: int):
    if boat_id in BOAT_CACHE:
        return BOAT_CACHE[boat_id]
    cfg = BlueBoatConfig(str(CONFIG_PATH))
    mav, imu, gps, motors, nav = cfg.init_from_config(boat_id)
    BOAT_CACHE[boat_id] = (mav, imu, gps, motors, nav)
    return BOAT_CACHE[boat_id]


def _get_home_state(boat_id: int):
    if boat_id not in HOME_STATE:
        HOME_STATE[boat_id] = {"active": False, "stop": False, "target": None}
    return HOME_STATE[boat_id]


def _interruptible_go_to_gps(nav, gps, motors, target_gps, stop_flag, distance_threshold=5.0):
    import utils.geo_conversion as geo
    import numpy as np
    import math

    target_cartesian = geo.conversion_spherique_cartesien(target_gps)
    target_coords = np.array(target_cartesian, dtype=float)

    distance_target = float("inf")
    while distance_target > distance_threshold:
        if stop_flag["stop"]:
            return False

        current_coords = np.array(gps.get_coords(), dtype=object)
        if current_coords[0] is not None and current_coords[1] is not None:
            delta = target_coords - current_coords.astype(float)
            target_heading = (math.degrees(math.atan2(delta[1], delta[0])) + 360.0) % 360.0
            distance_target = float(np.linalg.norm(delta))

            current_heading = nav.get_current_heading()
            if current_heading is None:
                time.sleep(nav.dt)
                continue

            error = target_heading - current_heading
            error = (error + 180) % 360 - 180
            correction = nav.Kp * error

            reference_distance = distance_threshold
            distance_correction = math.tanh(distance_target / reference_distance)

            left_motor = distance_correction * nav.base_speed - correction
            right_motor = distance_correction * nav.base_speed + correction

            left_motor = np.clip(left_motor, -nav.max_cmd, nav.max_cmd)
            right_motor = np.clip(right_motor, -nav.max_cmd, nav.max_cmd)

            motors.send_cmd_motor(left_motor, right_motor)

        time.sleep(nav.dt)

    motors.stop_motors()
    return True


def _home_worker(boat_id: int):
    state = _get_home_state(boat_id)
    state["active"] = True
    state["stop"] = False

    mav, imu, gps, motors, nav = _get_boat_components(boat_id)

    waypoints = [
        (48.1990856, -3.0155828, 6),
        (48.1990483, -3.014815, 10),
    ]

    try:
        mav.arm_disarm(True)
        for lat, lon, distance_threshold in waypoints:
            if state["stop"]:
                break
            state["target"] = (lat, lon)
            ok = _interruptible_go_to_gps(nav, gps, motors, (lat, lon), state, distance_threshold)
            if not ok:
                break
    finally:
        state["active"] = False
        state["target"] = None
        state["stop"] = False
        motors.stop_motors()


def _goto_worker(boat_id: int, target: tuple[float, float], distance_threshold: float = 5.0):
    state = _get_home_state(boat_id)
    state["active"] = True
    state["stop"] = False

    mav, imu, gps, motors, nav = _get_boat_components(boat_id)

    try:
        mav.arm_disarm(True)
        state["target"] = target
        _interruptible_go_to_gps(nav, gps, motors, target, state, distance_threshold)
    finally:
        state["active"] = False
        state["target"] = None
        state["stop"] = False
        motors.stop_motors()


def _goto_worker(boat_id: int, target: tuple[float, float], distance_threshold: float = 5.0):
    state = _get_home_state(boat_id)
    state["active"] = True
    state["stop"] = False

    mav, imu, gps, motors, nav = _get_boat_components(boat_id)

    try:
        mav.arm_disarm(True)
        state["target"] = target
        _interruptible_go_to_gps(nav, gps, motors, target, state, distance_threshold)
    finally:
        state["active"] = False
        state["target"] = None
        state["stop"] = False
        motors.stop_motors()


def api_boats(_request):
    return JsonResponse({"boats": _load_config()})


@csrf_exempt
def api_boats_save(request):
    if request.method != "POST":
        return HttpResponseBadRequest("POST required")
    try:
        body = json.loads(request.body.decode("utf-8"))
        boats = body.get("boats")
    except Exception:
        return HttpResponseBadRequest("Invalid JSON")
    if not isinstance(boats, list):
        return HttpResponseBadRequest("Invalid boats list")
    cleaned = []
    for item in boats:
        if not isinstance(item, dict):
            continue
        if "boat_id" not in item or "host" not in item:
            continue
        cleaned.append({
            "boat_id": int(item["boat_id"]),
            "host": str(item["host"]),
            "port": int(item.get("port", 6040)),
            "sysid": int(item.get("sysid", item["boat_id"])),
            "compid": int(item.get("compid", 1)),
        })
    if not cleaned:
        return HttpResponseBadRequest("Empty boats")
    _save_config(cleaned)
    GPS_CACHE.clear()
    BOAT_CACHE.clear()
    return JsonResponse({"ok": True, "boats": cleaned})


def api_boats_gps(_request):
    boats = _load_config()
    results = []
    for item in boats:
        gps = _get_gps_client(item)
        gps_info = gps.get_gps_dict()
        heading = None
        if gps_info:
            heading = gps_info.get("hdg_deg")
        state = _get_home_state(int(item.get("boat_id")))
        results.append({
            "boat_id": item.get("boat_id"),
            "gps": [float(gps_info["lat"]), float(gps_info["lon"])] if gps_info else None,
            "heading": float(heading) if heading is not None else None,
            "target": list(state["target"]) if state.get("target") else None,
        })
    return JsonResponse({"boats": results})


@csrf_exempt
def api_boat_action(request):
    if request.method != "POST":
        return HttpResponseBadRequest("POST required")
    try:
        body = json.loads(request.body.decode("utf-8"))
    except Exception:
        return HttpResponseBadRequest("Invalid JSON")
    boat_id = body.get("boat_id")
    action = body.get("action")
    if boat_id is None or not action:
        return HttpResponseBadRequest("Missing boat_id/action")
    mav, imu, gps, motors, nav = _get_boat_components(int(boat_id))

    if action == "arm":
        state = bool(body.get("state", True))
        return JsonResponse({"ok": True, "result": mav.arm_disarm(state)})
    if action == "mode":
        mode_name = body.get("mode")
        return JsonResponse({"ok": True, "result": mav.set_flight_mode(mode_name=mode_name)})
    if action == "neutral":
        return JsonResponse({"ok": True, "result": motors.stop_motors()})
    if action == "rc":
        ch1 = int(body.get("ch1", 1500))
        ch3 = int(body.get("ch3", 1500))
        return JsonResponse({"ok": True, "result": mav.send_rc_override(ch1=ch1, ch3=ch3)})
    if action == "drive":
        left = float(body.get("left", 0))
        right = float(body.get("right", 0))
        return JsonResponse({"ok": True, "result": motors.send_cmd_motor(left, right)})
    if action == "return_home":
        state = _get_home_state(int(boat_id))
        if not state["active"]:
            threading.Thread(target=_home_worker, args=(int(boat_id),), daemon=True).start()
        return JsonResponse({"ok": True})
    if action == "goto":
        lat = body.get("lat")
        lon = body.get("lon")
        if lat is None or lon is None:
            return HttpResponseBadRequest("Missing lat/lon")
        distance = float(body.get("distance", 5.0))
        state = _get_home_state(int(boat_id))
        if not state["active"]:
            threading.Thread(
                target=_goto_worker,
                args=(int(boat_id), (float(lat), float(lon)), distance),
                daemon=True,
            ).start()
        return JsonResponse({"ok": True})
    if action == "goto":
        lat = body.get("lat")
        lon = body.get("lon")
        if lat is None or lon is None:
            return HttpResponseBadRequest("Missing lat/lon")
        distance = float(body.get("distance", 5.0))
        state = _get_home_state(int(boat_id))
        if not state["active"]:
            threading.Thread(
                target=_goto_worker,
                args=(int(boat_id), (float(lat), float(lon)), distance),
                daemon=True,
            ).start()
        return JsonResponse({"ok": True})
    if action == "stop_return_home":
        state = _get_home_state(int(boat_id))
        state["stop"] = True
        motors.stop_motors()
        return JsonResponse({"ok": True, "result": mav.set_flight_mode(mode_name="HOLD")})
    if action == "status":
        batt = mav.get_battery_status()
        hdg = nav.get_current_heading()
        gps_info = gps.get_gps_dict()
        mode = mav.get_flight_mode()
        return JsonResponse({"ok": True, "battery": batt, "heading": hdg, "gps": gps_info, "mode": mode})

    return HttpResponseBadRequest("Unknown action")


def api_replay(request):
    log_name = request.GET.get("log")
    if not log_name:
        return HttpResponseBadRequest("Missing log param")

    logs = list_log_files()
    if log_name not in logs:
        return HttpResponseBadRequest("Invalid log")

    try:
        downsample = int(request.GET.get("downsample", "1"))
    except ValueError:
        return HttpResponseBadRequest("Invalid downsample")

    downsample = max(1, downsample)

    def get_float_param(name, default):
        raw = request.GET.get(name)
        if raw is None or raw == "":
            return default
        try:
            return float(raw)
        except ValueError:
            return default

    recursive = request.GET.get("recursive", "0") in ("1", "true", "True")
    init_uncertainty = get_float_param("init_uncertainty", 0.0)
    gps_uncertainty = get_float_param("gps_uncertainty", 0.0)
    dist_uncertainty = get_float_param("dist_uncertainty", 0.1)
    move_uncertainty = get_float_param("move_uncertainty", 0.5)
    precision = get_float_param("precision", 1.0)
    margin = get_float_param("margin", 10.0)

    log_path = REPO_ROOT / "logs" / log_name
    entries = []
    with open(log_path, "r", encoding="utf-8") as f:
        for line in f:
            parsed = parse_log_line(line)
            if parsed:
                entries.append(parsed)

    if not entries:
        return HttpResponseBadRequest("No valid entries")

    t0 = entries[0][0]
    times = []
    boat1 = []
    boat2 = []
    boat3 = []

    for ts, c1, c2, c3 in entries:
        times.append((ts - t0).total_seconds())
        boat1.append({"xy": c1, "gps": cartesian_to_gps(c1)})
        boat2.append({"xy": c2, "gps": cartesian_to_gps(c2)})
        boat3.append({"xy": c3, "gps": cartesian_to_gps(c3)})

    # Prediction with downsample
    ds_entries = entries[::downsample]
    pred_indices = []
    pred_boxes = []
    pred_pavings = []

    if len(ds_entries) >= 2:
        _ts, c1, c2, c3 = ds_entries[0]
        fleet = FleetPredictor(
            initial_positions=[c1, c2, c3],
            recursive=recursive,
            init_uncertainty=init_uncertainty,
            gps_uncertainty=gps_uncertainty,
            dist_uncertainty=dist_uncertainty,
            move_uncertainty=move_uncertainty,
            precision=precision,
            margin=margin,
            use_paving_objects=False,
            enable_display=False,
        )
        prev_ts, prev_c1, prev_c2, prev_c3 = ds_entries[0]
        for i, (ts, c1, c2, c3) in enumerate(ds_entries[1:], start=1):
            dx1 = c1[0] - prev_c1[0]
            dy1 = c1[1] - prev_c1[1]
            dx2 = c2[0] - prev_c2[0]
            dy2 = c2[1] - prev_c2[1]
            dx3 = c3[0] - prev_c3[0]
            dy3 = c3[1] - prev_c3[1]

            d12 = ((c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2) ** 0.5
            d13 = ((c1[0] - c3[0]) ** 2 + (c1[1] - c3[1]) ** 2) ** 0.5
            d23 = ((c2[0] - c3[0]) ** 2 + (c2[1] - c3[1]) ** 2) ** 0.5

            fleet.update(
                mothership_pos=c1,
                distances=[d12, d23, d13],
                movements=[(dx1, dy1), (dx2, dy2), (dx3, dy3)],
            )

            boxes = fleet.get_boxes()
            pred_boxes.append([
                box_to_polygon_gps(boxes[0]),
                box_to_polygon_gps(boxes[1]),
                box_to_polygon_gps(boxes[2]),
            ])

            # Pavings for scouts (boat 2 & 3)
            pred_pavings.append([
                paving_to_polygons_gps(fleet.contractors[1].paving),
                paving_to_polygons_gps(fleet.contractors[2].paving),
            ])

            pred_indices.append(i * downsample)

            prev_ts, prev_c1, prev_c2, prev_c3 = ts, c1, c2, c3

    payload = {
        "times": times,
        "boats": [boat1, boat2, boat3],
        "prediction": {
            "indices": pred_indices,
            "boxes": pred_boxes,
            "pavings": pred_pavings,
        },
    }
    return JsonResponse(payload)


def api_interval_reset(_request):
    INTERVAL_STATE["fleet"] = None
    INTERVAL_STATE["prev_coords"] = None
    INTERVAL_STATE["settings"] = None
    return JsonResponse({"ok": True})


@csrf_exempt
def api_interval_log_start(request):
    log_dir = REPO_ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"web_observer_{timestamp}.log"
    INTERVAL_STATE["log_enabled"] = True
    INTERVAL_STATE["log_file"] = str(log_path)
    return JsonResponse({"ok": True, "log": log_path.name})


@csrf_exempt
def api_interval_log_stop(_request):
    INTERVAL_STATE["log_enabled"] = False
    return JsonResponse({"ok": True})


def api_interval_step(request):
    boats = _load_config()
    if len(boats) < 3:
        return HttpResponseBadRequest("Need at least 3 boats in config")

    def get_float_param(name, default):
        raw = request.GET.get(name)
        if raw is None or raw == "":
            return default
        try:
            return float(raw)
        except ValueError:
            return default

    recursive = request.GET.get("recursive", "0") in ("1", "true", "True")
    init_uncertainty = get_float_param("init_uncertainty", 0.0)
    gps_uncertainty = get_float_param("gps_uncertainty", 0.0)
    dist_uncertainty = get_float_param("dist_uncertainty", 0.1)
    move_uncertainty = get_float_param("move_uncertainty", 0.5)
    precision = get_float_param("precision", 1.0)
    margin = get_float_param("margin", 10.0)

    settings = (recursive, init_uncertainty, gps_uncertainty, dist_uncertainty, move_uncertainty, precision, margin)

    coords = []
    gps_positions = []
    for item in boats[:3]:
        gps = _get_gps_client(item)
        pos_gps = gps.get_gps()
        pos_xy = gps.get_coords()
        if pos_xy[0] is None or pos_xy[1] is None:
            return HttpResponseBadRequest("GPS not ready")
        coords.append([float(pos_xy[0]), float(pos_xy[1])])
        gps_positions.append([float(pos_gps[0]), float(pos_gps[1])] if pos_gps else None)

    if INTERVAL_STATE["fleet"] is None or INTERVAL_STATE["settings"] != settings:
        INTERVAL_STATE["fleet"] = FleetPredictor(
            initial_positions=coords,
            recursive=recursive,
            init_uncertainty=init_uncertainty,
            gps_uncertainty=gps_uncertainty,
            dist_uncertainty=dist_uncertainty,
            move_uncertainty=move_uncertainty,
            precision=precision,
            margin=margin,
            use_paving_objects=False,
            enable_display=False,
        )
        INTERVAL_STATE["prev_coords"] = coords
        INTERVAL_STATE["settings"] = settings

    prev = INTERVAL_STATE["prev_coords"]
    fleet = INTERVAL_STATE["fleet"]

    dx1 = coords[0][0] - prev[0][0]
    dy1 = coords[0][1] - prev[0][1]
    dx2 = coords[1][0] - prev[1][0]
    dy2 = coords[1][1] - prev[1][1]
    dx3 = coords[2][0] - prev[2][0]
    dy3 = coords[2][1] - prev[2][1]

    d12 = ((coords[0][0] - coords[1][0]) ** 2 + (coords[0][1] - coords[1][1]) ** 2) ** 0.5
    d13 = ((coords[0][0] - coords[2][0]) ** 2 + (coords[0][1] - coords[2][1]) ** 2) ** 0.5
    d23 = ((coords[1][0] - coords[2][0]) ** 2 + (coords[1][1] - coords[2][1]) ** 2) ** 0.5

    fleet.update(
        mothership_pos=coords[0],
        distances=[d12, d23, d13],
        movements=[(dx1, dy1), (dx2, dy2), (dx3, dy3)],
    )

    INTERVAL_STATE["prev_coords"] = coords

    if INTERVAL_STATE.get("log_enabled") and INTERVAL_STATE.get("log_file"):
        with open(INTERVAL_STATE["log_file"], "a", encoding="utf-8") as f:
            f.write(
                f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S,%f')} - INFO - "
                f"Boat 1: [np.float64({coords[0][0]}), np.float64({coords[0][1]})], "
                f"Boat 2: [np.float64({coords[1][0]}), np.float64({coords[1][1]})], "
                f"Boat 3: [np.float64({coords[2][0]}), np.float64({coords[2][1]})]\n"
            )

    boxes = fleet.get_boxes()
    payload = {
        "boats": gps_positions,
        "boxes": [
            box_to_polygon_gps(boxes[0]),
            box_to_polygon_gps(boxes[1]),
            box_to_polygon_gps(boxes[2]),
        ],
        "pavings": [
            paving_to_polygons_gps(fleet.contractors[1].paving),
            paving_to_polygons_gps(fleet.contractors[2].paving),
        ],
    }
    return JsonResponse(payload)
