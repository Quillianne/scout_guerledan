"""
Equivalent contractor utilities for positioning.

- Distance contraction via CtcInverse on distance to a neighbor box
- GPS correction via intersection with CtcWrapper
- Dead-reckoning via CtcInverse on backward motion model
"""

from codac import AnalyticFunction, CtcInverse, CtcWrapper, Interval, IntervalVector, VectorVar, cos, sin, sqrt, sqr, vec


def create_distance_contractor(neighbor_box, dist_interval):
    """
    Contrainte: distance(position_bateau, neighbor) ∈ dist_interval

    Args:
        neighbor_box: IntervalVector (position incertaine du voisin)
        dist_interval: Interval (distance mesurée)
    """
    x = VectorVar(2)
    bx = neighbor_box[0]
    by = neighbor_box[1]
    f_dist = AnalyticFunction([x], sqrt(sqr(x[0] - bx) + sqr(x[1] - by)))
    return CtcInverse(f_dist, dist_interval)


class equivalent_contractor:
    """
    Contracteur équivalent pour le positionnement.

    - add_distance_condition: contraction par distance
    - add_gps_condition: intersection avec une boîte GPS
    - add_movement_condition: dead reckoning via CtcInverse (f_back)
    """

    def __init__(self, initial_box):
        self.box = IntervalVector(initial_box)
        self.ctc = CtcWrapper(self.box)

    def add_distance_condition(self, dist_interval, neighbor_box):
        """
        Contraction par distance au voisin.

        Args:
            dist_interval: Interval
            neighbor_box: IntervalVector
        """
        ctc_dist = create_distance_contractor(neighbor_box, dist_interval)
        self.ctc = self.ctc & ctc_dist

    def add_gps_condition(self, gps_box):
        """
        Intersection avec la mesure GPS (boîte).

        Args:
            gps_box: IntervalVector
        """
        ctc_gps = CtcWrapper(IntervalVector(gps_box))
        self.ctc = self.ctc & ctc_gps

    def add_movement_condition(self, dx_interval, dy_interval):
        """
        Mise à jour par dead reckoning via CtcInverse.

        Args:
            dx_interval: Interval (déplacement en x)
            dy_interval: Interval (déplacement en y)
        """
        dx = Interval(dx_interval)
        dy = Interval(dy_interval)

        self.box[0] = self.box[0] + dx
        self.box[1] = self.box[1] + dy

        x = VectorVar(2)
        f_back = AnalyticFunction([x], vec(x[0] - dx, x[1] - dy))
        self.ctc = CtcInverse(f_back, self.ctc)

    def get_box(self):
        """Retourne la boîte contractée et met à jour self.box."""
        self.ctc.contract(self.box)
        return self.box

    def get_ctc(self):
        """Retourne le contracteur courant."""
        return self.ctc

    def set_ctc(self, box):
        """Met à jour le contracteur avec une boîte enveloppée."""
        self.box = IntervalVector(box)
        self.ctc = CtcWrapper(self.box)

    def contract(self, box):
        """Contracte une boîte avec le contracteur courant."""
        target = IntervalVector(box)
        self.ctc.contract(target)
        return target
