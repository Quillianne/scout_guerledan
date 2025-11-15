from codac import *
import numpy as np


class Observer:
    def __init__(self, interval: float):
        """initializes the estimation of the position"""
        self.estimation = np.array([interval])

    def update(self, time: float):
        """updates the estimation of the position, using the distances measured"""
        pass


        