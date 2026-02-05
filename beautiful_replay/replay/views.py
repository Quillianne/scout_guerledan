import json
import re
import sys
from datetime import datetime
from pathlib import Path

from django.http import JsonResponse, HttpResponseBadRequest
from django.shortcuts import render

# Make repo root importable (for utils.*)
REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from utils.geo_conversion import conversion_cartesien_spherique
from utils.fleet_prediction import FleetPredictor
from codac import PavingOut

LOG_PATTERN = re.compile(r"np\.float64\(([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\)")


def index(_request):
    return render(_request, "replay/index.html")


def list_log_files():
    log_dir = REPO_ROOT / "logs"
    if not log_dir.exists():
        return []
    logs = [p.name for p in log_dir.iterdir() if p.is_file() and p.name.endswith(".log")]
    return sorted(logs, reverse=True)


def api_logs(_request):
    return JsonResponse({"logs": list_log_files()})


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
