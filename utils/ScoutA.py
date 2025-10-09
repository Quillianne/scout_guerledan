from utils.MotherShip import BBBoat, MotherShip
import time



class ScoutA(BBBoat):
    def __init__(self, name = "ScoutA", host = "192.168.2.202", sysid = 2):
        super().__init__(name, host, sysid)

    




    def formation_triangle_1(self, mothership:MotherShip, duration:float=30):
        """
        Garder une formation en triangle equilatéral basée sur le mothership (sa position + son cap)
        """
        t0 = time.time()
        while time.time() - t0 < duration :
