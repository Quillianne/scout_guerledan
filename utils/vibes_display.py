"""
VIBes display helpers for equivalent_contractor.

Displays:
- Contractor pavings (ctc)
- Current enclosing boxes
"""

from codac import Color, Figure2D, GraphicOutput, Interval, IntervalVector, axis


def compute_contractor_paving(ctc, search_box: IntervalVector, precision: float = 0.3) -> list:
    """
    Compute the paving of a contractor using SIVIA algorithm.

    Args:
        ctc: Contractor to visualize
        search_box: Initial search box
        precision: Minimum box size

    Returns:
        List of IntervalVector representing the paving
    """
    result = []
    stack = [IntervalVector(search_box)]

    while stack:
        box = stack.pop()

        contracted = IntervalVector(box)
        ctc.contract(contracted)

        if contracted.is_empty():
            continue

        if contracted.max_diam() < precision:
            result.append(contracted)
        else:
            idx = 0 if contracted[0].diam() > contracted[1].diam() else 1
            mid = contracted[idx].mid()

            box1 = IntervalVector(contracted)
            box2 = IntervalVector(contracted)

            box1[idx] = Interval(contracted[idx].lb(), mid)
            box2[idx] = Interval(mid, contracted[idx].ub())

            stack.append(box1)
            stack.append(box2)

    return result


class VibesDisplay:
    """
    Display three equivalent_contractor instances in VIBes.
    """

    def __init__(
        self,
        ctc_a,
        ctc_b,
        ctc_c,
        precision: float = 0.3,
        margin: float = 10.0,
        min_width: float = 40.0,
        min_height: float = 40.0,
        truth_box_size: float = 0.5,
    ):
        self.contractors = [ctc_a, ctc_b, ctc_c]
        self.precision = precision
        self.margin = margin
        self.min_width = min_width
        self.min_height = min_height
        self.truth_box_size = truth_box_size
        self.fig = Figure2D("Prediction (Contractors)", GraphicOutput.VIBES)
        self.colors = [Color.red(), Color.blue(), Color.green()]
        self._view_box = None
        self._truth_positions = None
        self._truth_pending = False

    def set_truth_positions(self, positions):
        """
        Stocke les positions réelles (x, y). Elles seront dessinées au prochain draw.

        Args:
            positions: iterable de positions [(x, y), ...] (None autorisé)
        """
        if positions is None:
            return
        self._truth_positions = list(positions)
        self._truth_pending = True

    def _ensure_min_size(self, box: IntervalVector) -> IntervalVector:
        width = box[0].diam()
        height = box[1].diam()

        if width < self.min_width:
            cx = box[0].mid()
            half = self.min_width / 2.0
            box[0] = Interval(cx - half, cx + half)

        if height < self.min_height:
            cy = box[1].mid()
            half = self.min_height / 2.0
            box[1] = Interval(cy - half, cy + half)

        return box

    def _update_view_box(self, hull: IntervalVector, mothership_box: IntervalVector):
        target = IntervalVector([
            [hull[0].lb() - self.margin, hull[0].ub() + self.margin],
            [hull[1].lb() - self.margin, hull[1].ub() + self.margin],
        ])
        target = self._ensure_min_size(target)

        cx = mothership_box[0].mid()
        cy = mothership_box[1].mid()

        base = IntervalVector([
            [cx - self.min_width / 2.0, cx + self.min_width / 2.0],
            [cy - self.min_height / 2.0, cy + self.min_height / 2.0],
        ])

        if self._view_box is None:
            self._view_box = IntervalVector(base)
            if not (
                target[0].lb() >= self._view_box[0].lb()
                and target[0].ub() <= self._view_box[0].ub()
                and target[1].lb() >= self._view_box[1].lb()
                and target[1].ub() <= self._view_box[1].ub()
            ):
                self._view_box |= target
                self._view_box = self._ensure_min_size(self._view_box)
            return

        # Keep view centered on mothership, but expand if needed
        self._view_box = IntervalVector(base)

        within_x = (
            target[0].lb() >= self._view_box[0].lb()
            and target[0].ub() <= self._view_box[0].ub()
        )
        within_y = (
            target[1].lb() >= self._view_box[1].lb()
            and target[1].ub() <= self._view_box[1].ub()
        )

        if not (within_x and within_y):
            self._view_box |= target
            self._view_box = self._ensure_min_size(self._view_box)

    def draw(self):
        """Draw pavings and current boxes for each contractor."""

        all_boxes = []
        pavings = []

        for idx, eq in enumerate(self.contractors):
            box = eq.get_box()
            all_boxes.append(box)

            paving_box = IntervalVector([
                [box[0].lb() - self.margin, box[0].ub() + self.margin],
                [box[1].lb() - self.margin, box[1].ub() + self.margin]
            ])

            pavings.append(compute_contractor_paving(eq.get_ctc(), paving_box, self.precision))

        if all_boxes:
            hull = IntervalVector(all_boxes[0])
            for b in all_boxes[1:]:
                hull |= b
            self._update_view_box(hull, all_boxes[0])

        if self._view_box is not None:
            self.fig.set_axes(
                axis(0, Interval(self._view_box[0].lb(), self._view_box[0].ub())),
                axis(1, Interval(self._view_box[1].lb(), self._view_box[1].ub())),
            )

        self.fig.clear()

        for idx, boxes in enumerate(pavings):
            color = self.colors[idx % len(self.colors)]
            for b in boxes:
                self.fig.draw_box(b, color)

        for idx, box in enumerate(all_boxes):
            color = self.colors[idx % len(self.colors)]
            self.fig.draw_box(box, color)

        if self._truth_pending and self._truth_positions:
            half = self.truth_box_size / 2.0
            for pos in self._truth_positions:
                if pos is None or len(pos) < 2:
                    continue
                x, y = pos[0], pos[1]
                if x is None or y is None:
                    continue
                truth_box = IntervalVector([[x - half, x + half], [y - half, y + half]])
                self.fig.draw_box(truth_box, Color.black())
            self._truth_pending = False
