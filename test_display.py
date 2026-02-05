"""
Test simulation with three fake boats in triangle 
"""

import math
import time
import matplotlib.pyplot as plt
from codac import Interval
from utils.fleet_prediction import FleetPredictor

# ----------------------------------------------------------------------------
# Parameters
# ----------------------------------------------------------------------------
DT = 0.2
UPDATE_REAL_INTERVAL = 2.0
UPDATE_STEPS = int(UPDATE_REAL_INTERVAL / DT)
DO_ESCAPE = True
PLOT_BOX_SIZES = True
RECURSIVE_CONTRACTION = False

SPEED = 1.0
MOVE_UNCERTAINTY = 0.5
DIST_UNCERTAINTY = 0.1
GPS_UNCERTAINTY = 0.

INIT_UNCERTAINTY = 1.0

N_STEPS = 1050


# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------

def true_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def make_init_box(x, y):
    return IntervalVector([x, y]).inflate(INIT_UNCERTAINTY)


def box_size_str(box: IntervalVector) -> str:
    return f"dx={box[0].diam():.3f}, dy={box[1].diam():.3f}"


# ----------------------------------------------------------------------------
# Simulation
# ----------------------------------------------------------------------------

def main():
    # Triangle initial positions (x, y)
    p1 = [0.0, 0.0]
    p2 = [15.0, 15.0]
    p3 = [15.0, -15.0]

    old_p1 = list(p1)
    old_p2 = list(p2)
    old_p3 = list(p3)

    fleet = FleetPredictor(
        initial_positions=[p1, p2, p3],
        recursive=RECURSIVE_CONTRACTION,
        gps_uncertainty=GPS_UNCERTAINTY,
        dist_uncertainty=DIST_UNCERTAINTY,
        move_uncertainty=MOVE_UNCERTAINTY,
        precision=1.0,
        margin=10.0,
    )

    start_maneuver_step = -9999
    escape3_start_step = start_maneuver_step
    escape3_end_step = start_maneuver_step + 8
    escape2_start_step = start_maneuver_step + 8
    escape2_end_step = start_maneuver_step + 16
    maneuver_active = False

    steps = []
    box2_sizes = []
    box3_sizes = []
    update_steps = []
    update_compute_times = []
    update_draw_times = []
    global_spans = []  # list of (start, end)
    boat2_starts = []  # list of steps
    current_span_start = None


    for t in range(N_STEPS): 
        # Straight motion along +x
        dx = SPEED * DT
        dy = 0.0

        p1[0] += dx
        p2[0] += dx
        p3[0] += dx

        current_step = (t-1) // UPDATE_STEPS
        boxes = fleet.get_boxes()
        box2_size = max(boxes[1][0].diam(), boxes[1][1].diam())
        box3_size = max(boxes[2][0].diam(), boxes[2][1].diam())
        # print(f"Step {current_step}: box 2 size: {box2_size:.2f}, box 3 size: {box3_size:.2f}")

        if (
            (not maneuver_active)
            and DO_ESCAPE
            and ((box2_size > 20.0) or (box3_size > 20.0))
        ):
            start_maneuver_step = current_step + 1
            escape3_start_step = start_maneuver_step
            escape3_end_step = start_maneuver_step + 8
            escape2_start_step = start_maneuver_step + 8
            escape2_end_step = start_maneuver_step + 16
            maneuver_active = True

        steps.append(current_step)
        box2_sizes.append(box2_size)
        box3_sizes.append(box3_size)

        if DO_ESCAPE:
            # Escape pattern for boat 3 (8 steps)
            if current_step == start_maneuver_step:
                p3[0] += 3*dx
            elif current_step == start_maneuver_step + 1:
                p3[0] += 3*dx
            elif current_step == start_maneuver_step + 2:
                p3[1] -= 3*dx
            elif current_step == start_maneuver_step + 3:
                p3[1] -= 3*dx
            elif current_step == start_maneuver_step + 4:
                p3[0] -= 3*dx
            elif current_step == start_maneuver_step + 5:
                p3[0] -= 3*dx
            elif current_step == start_maneuver_step + 6:
                p3[1] += 3*dx
            elif current_step == start_maneuver_step + 7:
                p3[1] += 3*dx

            # Same escape pattern for boat 2 (steps 13-20)
            if current_step == start_maneuver_step + 8:
                p2[0] += 3*dx
            elif current_step == start_maneuver_step + 9:
                p2[0] += 3*dx
            elif current_step == start_maneuver_step + 10:
                p2[1] += 3*dx
            elif current_step == start_maneuver_step + 11:
                p2[1] += 3*dx
            elif current_step == start_maneuver_step + 12:
                p2[0] -= 3*dx
            elif current_step == start_maneuver_step + 13:
                p2[0] -= 3*dx
            elif current_step == start_maneuver_step + 14:
                p2[1] -= 3*dx
            elif current_step == start_maneuver_step + 15:
                p2[1] -= 3*dx

        if maneuver_active and current_step > escape2_end_step:
            maneuver_active = False

        is_update_step = (t % UPDATE_STEPS == 0)

        if is_update_step:
            update_start = time.perf_counter()
            # Movement update (dead reckoning)
            dx1 = p1[0] - old_p1[0]
            dy1 = p1[1] - old_p1[1]
            dx2 = p2[0] - old_p2[0]
            dy2 = p2[1] - old_p2[1]
            dx3 = p3[0] - old_p3[0]
            dy3 = p3[1] - old_p3[1]

            old_p1 = list(p1)
            old_p2 = list(p2)
            old_p3 = list(p3)

            # Distance constraints
            d12 = true_distance(p1, p2)
            d13 = true_distance(p1, p3)
            d23 = true_distance(p2, p3)

            fleet.update(
                mothership_pos=p1,
                distances=[d12, d23, d13],
                movements=[(dx1, dy1), (dx2, dy2), (dx3, dy3)],
            )


            if current_step == escape3_start_step:
                boxes = fleet.get_boxes()
                box2 = boxes[1]
                box3 = boxes[2]
                print(f"STEP {current_step} // Manoeuver start: box 2: {box_size_str(box2)}, box 3: {box_size_str(box3)}")
                current_span_start = current_step

            if current_step == escape2_start_step:
                boxes = fleet.get_boxes()
                box2 = boxes[1]
                box3 = boxes[2]
                # print(f"STEP {current_step} // Manoeuver start: box 2: {box_size_str(box2)}, box 3: {box_size_str(box3)}")
                boat2_starts.append(current_step)

            if current_step == escape2_end_step:
                boxes = fleet.get_boxes()
                box2 = boxes[1]
                box3 = boxes[2]
                print(f"STEP {current_step} // Manoeuver end: box 2: {box_size_str(box2)}, box 3: {box_size_str(box3)}")
                if current_span_start is not None:
                    global_spans.append((current_span_start, current_step))
                    current_span_start = None

            
            update_before_draw = time.perf_counter()
            # Draw pavings in VIBes
            fleet.draw([p1, p2, p3])
            update_end = time.perf_counter()

            update_steps.append(current_step)
            update_compute_times.append(update_before_draw - update_start)
            update_draw_times.append(update_end - update_before_draw)

        time.sleep(DT/1000)

    if PLOT_BOX_SIZES:
        plt.figure("Box sizes")
        plt.plot(steps, box2_sizes, label="Boat 2")
        plt.plot(steps, box3_sizes, label="Boat 3")
        used_labels = set()
        for start, end in global_spans:
            lbl = "Global maneuver" if "Global maneuver" not in used_labels else None
            used_labels.add("Global maneuver")
            plt.axvspan(start, end, color="gray", alpha=0.15, label=lbl)

            lbl = "Global start" if "Global start" not in used_labels else None
            used_labels.add("Global start")
            plt.axvline(x=start, color="black", linestyle="-", alpha=0.8, label=lbl)

            lbl = "Global end" if "Global end" not in used_labels else None
            used_labels.add("Global end")
            plt.axvline(x=end, color="black", linestyle="-", alpha=0.8, label=lbl)

        for step in boat2_starts:
            lbl = "Boat 2 start" if "Boat 2 start" not in used_labels else None
            used_labels.add("Boat 2 start")
            plt.axvline(x=step, color="blue", linestyle="--", alpha=0.8, label=lbl)
        plt.xlabel("Step")
        plt.ylabel("Box size (max diameter)")
        plt.legend()
        plt.tight_layout()
        plt.show()

    if PLOT_BOX_SIZES:
        fig, ax1 = plt.subplots(num="Update times")
        ax1.plot(update_steps, update_compute_times, color="tab:blue", label="Compute time")
        ax1.set_xlabel("Step")
        ax1.set_ylabel("Compute time (s)", color="tab:blue")
        ax1.tick_params(axis="y", labelcolor="tab:blue")

        ax2 = ax1.twinx()
        ax2.plot(update_steps, update_draw_times, color="tab:orange", label="Draw time")
        ax2.set_ylabel("Draw time (s)", color="tab:orange")
        ax2.tick_params(axis="y", labelcolor="tab:orange")

        lines = ax1.get_lines() + ax2.get_lines()
        labels = [line.get_label() for line in lines]
        ax1.legend(lines, labels, loc="best")

        fig.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
