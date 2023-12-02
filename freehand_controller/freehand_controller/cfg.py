import numpy as np
import math

# 机械手参数
unit_length = 1000
module1_config = {
    "p1": np.array([5.355 / unit_length, -6. / unit_length, 17.185 / unit_length]),
    "p2": np.array([5.355 / unit_length, 6. / unit_length, 17.185 / unit_length]),
    "p3": np.array([14.050 / unit_length, -6. / unit_length, 0]),
    "p4": np.array([14.050 / unit_length, 6. / unit_length, 0]),
    "l1": 22.9 / unit_length,
    "l2": 22.9 / unit_length,
    "z0": 4. / unit_length,
    "bevel": 256 * 64 * 2,
    # "screw": 2 * math.pi / 0.001,  
    "h": 0.001,
    "screw": 1 * 256 * 4 / 0.001, 
    "A": 9.5 / unit_length,
    "B": 32.35 / unit_length,
    "C": 29.867 / unit_length,
    "D": 7.3 / unit_length,
    "degree1_init": math.radians(61.4845),
    "degree2_init": math.radians(118.1266),
}
module2_config = {
    "p1": np.array([9.932 / unit_length, 0. / unit_length, 13.797 / unit_length]),
    "p2": np.array([0. / unit_length,  1. / unit_length, 0. / unit_length]),
    "p3": np.array([13.050 / unit_length, 0. / unit_length, 0]),
    "l1": 5.6 / unit_length,
    "l2": 18.4 / unit_length,
    "z0": 4. / unit_length,
    "bevel": 128 * 256 * 2,
    "screw": 4 * 256 * 4 / 0.001,   
    "h": 0.001,
    "A": 4.5 / unit_length,
    "B": 26.5 / unit_length,
    "C": 25.  / unit_length,
    "D": 6. / unit_length,
    "degree1_init": math.radians(45.0619),
    "degree2_init": math.radians(124.5317),
    # mcp-pip
    "Am": 7. / unit_length,
    "Bm": 46. / unit_length,
    "Cm": 45. / unit_length,
    "Dm": 6. / unit_length,
    "degree1m_init": math.radians(66.3557),
    "degree2m_init": math.radians(111.6954),
    
}
module3_config = {
    "p1": np.array([9.932 / unit_length, 0. / unit_length, 13.797 / unit_length]),
    "p2": np.array([0. / unit_length,  1. / unit_length, 0. / unit_length]),
    "p3": np.array([13.050 / unit_length, 0. / unit_length, 0]),
    "l1": 5.6 / unit_length,
    "l2": 18.4 / unit_length,
    "z0": 4. / unit_length,
    "bevel": 128 * 256 * 2,
    "screw": 4 * 256 * 4 / 0.001, 
    "h": 0.001,
    "A": 4.5 / unit_length,
    "B": 26.5 / unit_length,
    "C": 25. / unit_length,
    "D": 6. / unit_length,
    "degree1_init": math.radians(45.0619),
    "degree2_init": math.radians(124.5317),
    # mcp-pip
    "Am": 7. / unit_length,
    "Bm": 36.85 / unit_length,
    "Cm": 36. / unit_length,
    "Dm": 6. / unit_length,
    "degree1m_init": math.radians(67.63534),
    "degree2m_init": math.radians(103.46481),
}
module4_config = {
    "p1": np.array([10.219 / unit_length, -6. / unit_length, 12.311 / unit_length]),
    "p2": np.array([10.219 / unit_length,  6. / unit_length, 12.311 / unit_length]),
    "p3": np.array([12.050 / unit_length, -6. / unit_length, 0]),
    "p4": np.array([12.050 / unit_length,  6. / unit_length, 0]),
    "l1": 18.9 / unit_length,
    "l2": 18.9 / unit_length,
    "z0": (4.+2.5) / unit_length,
    "bevel": 256 * 275 * 2,
    "screw": 1 * 256 * 4 / 0.001, 
    "h": 0.001,
    "A": 7. / unit_length,
    "B": 37.61 / unit_length,
    "C": 36. / unit_length,
    "D": 5.5 / unit_length,
    "degree1_init": math.radians(71.98912),
    "degree2_init": math.radians(109.82193),
}
module_config = {
    "thumb": module4_config,
    "index": module1_config,
    "middle": module1_config,
    "fourth": module2_config,
    "little": module3_config,
}

MOTOR_NUM = 13
class FingerCommand:
    def __init__(self):
        self.zero()

    def zero(self):    
        self.qDes = np.zeros(MOTOR_NUM)
        self.qdDes = np.zeros(MOTOR_NUM)
        self.tauDes = np.zeros(MOTOR_NUM)
        self.kpDes = np.zeros(MOTOR_NUM)
        self.kdDes = np.zeros(MOTOR_NUM)

class FingerData:
    def __init__(self):
        self.zero()

    def zero(self):
        self.q = np.zeros(MOTOR_NUM)
        self.qd = np.zeros(MOTOR_NUM)
        self.cur = np.zeros(MOTOR_NUM)