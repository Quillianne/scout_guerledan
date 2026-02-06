"""
Fleet prediction helper built on equivalent_contractor.

- Choose mothership index
- Recursive or non-recursive movement update
- Update using distances to mothership + mothership position
- Optional draw helper
"""

from typing import Iterable, List, Optional, Sequence

from codac import Interval, IntervalVector

from utils.prediction import equivalent_contractor
from utils.vibes_display import VibesDisplay


class FleetPredictor:
    def __init__(
        self,
        initial_positions: Sequence[Sequence[float]],
        recursive: bool = False,
        init_uncertainty: float = 0.5,
        gps_uncertainty: float = 0.5,
        dist_uncertainty: float = 0.5,
        move_uncertainty: float = 0.5,
        precision: float = 1.0,
        margin: float = 10.0,
        use_paving_objects: bool = True,
        enable_display: bool = True,
    ):
        if len(initial_positions) != 3:
            raise ValueError("FleetPredictor attend exactement 3 bateaux")
        self.recursive = recursive
        self.init_uncertainty = init_uncertainty
        self.gps_uncertainty = gps_uncertainty
        self.dist_uncertainty = dist_uncertainty
        self.move_uncertainty = move_uncertainty
        self.contractors: List[equivalent_contractor] = [
            equivalent_contractor(IntervalVector([pos[0], pos[1]]).inflate(self.init_uncertainty))
            for pos in initial_positions
        ]

        self.display = None
        if enable_display and len(self.contractors) == 3:
            self.display = VibesDisplay(
                self.contractors[0],
                self.contractors[1],
                self.contractors[2],
                precision=precision,
                margin=margin,
                use_paving_objects=use_paving_objects,
            )

    def _apply_movement(self, idx: int, dx: float, dy: float):
        ctc = self.contractors[idx]
        if self.recursive:
            ctc.add_movement_condition(
                Interval(dx).inflate(self.move_uncertainty),
                Interval(dy).inflate(self.move_uncertainty),
            )
        else:
            if hasattr(ctc, "add_movement_condition_non_recursive"):
                ctc.add_movement_condition_non_recursive(
                    Interval(dx).inflate(self.move_uncertainty),
                    Interval(dy).inflate(self.move_uncertainty),
                )
            else:
                ctc.add_movement_condition(
                    Interval(dx).inflate(self.move_uncertainty),
                    Interval(dy).inflate(self.move_uncertainty),
                )

    def update(
        self,
        mothership_pos: Sequence[float],
        distances: Sequence[Optional[float]],
        movements: Optional[Sequence[Sequence[float]]] = None,
    ):
        """
        Update contractors from mothership position and pairwise distances.

        Args:
            mothership_pos: (x, y) position of mothership
            distances: [ms->s1, s1->s2, s2->ms]
            movements: optional list of (dx, dy) per boat
        """
        if movements is not None:
            for i, (dx, dy) in enumerate(movements):
                self._apply_movement(i, dx, dy)

        ms = self.contractors[0]
        s1 = self.contractors[1]
        s2 = self.contractors[2]
        gps_box = IntervalVector([mothership_pos[0], mothership_pos[1]]).inflate(self.gps_uncertainty)
        ms.add_gps_condition(gps_box)

        mode = "fixpointdist" if self.recursive else "fixpointall"

        if len(self.contractors) != 3 or len(distances) != 3:
            raise ValueError("FleetPredictor.update attend 3 bateaux et 3 distances.")

        d01, d12, d20 = distances
        if d01 is not None and d20 is not None:
            ms.add_double_distance_condition(
                Interval(d01).inflate(self.dist_uncertainty),
                s1.get_box(),
                Interval(d20).inflate(self.dist_uncertainty),
                s2.get_box(),
                mode=mode,
            )

        if d01 is not None and d12 is not None:
            s1.add_double_distance_condition(
                Interval(d01).inflate(self.dist_uncertainty),
                ms.get_box(),
                Interval(d12).inflate(self.dist_uncertainty),
                s2.get_box(),
                mode=mode,
            )

        if d12 is not None and d20 is not None:
            s2.add_double_distance_condition(
                Interval(d12).inflate(self.dist_uncertainty),
                s1.get_box(),
                Interval(d20).inflate(self.dist_uncertainty),
                ms.get_box(),
                mode=mode,
            )

        # Optional paving update when used by display
        for ctc in self.contractors:
            if hasattr(ctc, "update_paving"):
                ctc.update_paving()

    def get_boxes(self) -> List[IntervalVector]:
        return [ctc.get_box() for ctc in self.contractors]

    def draw(self, truth_positions: Optional[Iterable[Sequence[float]]] = None):
        if self.display is None:
            return
        if truth_positions is not None:
            self.display.set_truth_positions(list(truth_positions))
        self.display.draw()
