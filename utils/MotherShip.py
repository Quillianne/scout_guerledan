import numpy as np
import time

from utils.bblib import init_blueboat 


class BBBoat:
    """
    Mother class for all boats (ScoutA, ScoutB, MotherShip)
    """
    def __init__(self, name:str="BBBoat", host:str="192.168.2.201", sysid:int=1):
        self.name = name

        self.MAV, self.IMU, self.GPS, self.MOTORS, self.NAV = init_blueboat(host, sysid)

        self.location = np.array(self.GPS.get_coords()).reshape((1, 2))
        


        





class MotherShip(BBBoat):
    """
    MotherShip class, derived from BBBoat
    """
    def __init__(self, name:str="MotherShip"):
        super().__init__(name=name)
        # Additional initialization for MotherShip can be added here

    def lissajous(self, t:float, size:int=40, duration:int=200):
        """
        Computes the position of a point following a lissajous curve.

        Parameters:
        -----------
        t : float
            current time to know where to put the point on the curve.
        size: int
            size of the curve (width of the curve).
        duration : int
            duration of a full loop on the curve.
        
        Returns:
        --------
        numpy.ndarray: 2x1 column array [x, y] position on the curve
        """
        # Lissajous curve parameters (figure-8 pattern)
        freq_x = 1.0
        freq_y = 2.0
        phase = np.pi / 2
        
        # Normalize time to [0, 2π] based on duration
        theta = 2 * np.pi * t / duration
        
        # Compute position on Lissajous curve
        x = size * np.sin(freq_x * theta)
        y = size * np.sin(freq_y * theta + phase)
        
        return np.array([[x], [y]]) 



