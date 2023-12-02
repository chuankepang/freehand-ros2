#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
# import serial.tools.list_ports
import time
from freehand_controller.pyHand_api import pyHand, FingerControllerCommand, FingerControllerData

hand_ctrl = pyHand(is_sim=1)


def test_ik_fk():
    print(f'---------------测试-------------')
    print(f'电机向下为正，手指弯曲')
    print(f'----------------------------')
    joints_in = np.array([0., 0., 0.])
    pos = hand_ctrl.ik_index(joints_in)
    print(f'逆解输入:{joints_in}')
    print(f'逆解输出-电机弧度指令(rad):{pos}')
    
    index_cfg = hand_ctrl.get_module_config("index")
    # bevel = index_cfg["bevel"]
    # screw = index_cfg["screw"]
    joints_out = hand_ctrl.fk_index(pos)
    print(f'正解输入:{pos}')
    print(f'正解输出-关节角度(rad):{joints_out}')
    print(f'----------------------------')
    
    joints_in = np.array([math.radians(20), math.radians(90), math.radians(30)])
    pos = hand_ctrl.ik_index(joints_in)
    print(f'逆解输入:{joints_in}')
    print(f'逆解输出-电机弧度指令(rad):{pos}')
    
    index_cfg = hand_ctrl.get_module_config("index")
    # bevel = index_cfg["bevel"]
    # screw = index_cfg["screw"]
    joints_out = hand_ctrl.fk_index(pos)
    print(f'正解输入:{pos}')
    print(f'正解输出-关节角度(rad):{joints_out}')
    print(f'----------------------------')
    
    print(f'无名指与小拇指的逆解是一致的')

    joints_in = np.array([0., 0.])
    pos = hand_ctrl.ik_little(joints_in) # ik_little ik_fourth
    print(f'逆解输入:{joints_in}')
    print(f'逆解输出-电机弧度指令(rad):{pos}')
    

if __name__ == '__main__':
    test_ik_fk()
    
