#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
# import serial.tools.list_ports
import time
from hand_api import Hand
np.set_printoptions(precision=3, suppress=True)

def run():
    hand = Hand()
    print(f'---------------测试-------------')
    # 逆解测试
    pos = 1
    joints = np.array([0,pos,pos,0,pos,pos,0,pos,0,pos,0,0,0])
    print(f'joints: {joints}')
    pos = hand.ik_joints(joints)
    print(f'pos: {pos}')
    pos = hand.map_motor(pos)
    print(f'pos: {pos}')
    # print(f'cmd: {hand.cmd_}')
    
    # 一套动作演示
    dt = 1
    hand.action('open',delay=dt)    
    q0 = np.array([0,0.3,0,0,0.3,0,0,0,0,0,0,0,0.0])    
    q = q0    
    
    # hand.set_motor_ctrl_param(5, 0, 0)
    # hand.flush()
    # time.sleep(dt)
    # pos = 0
    # dir = 1
    # while 1:
    #     hand.set_motor_target(5, 0, 0, -5000)
        
    #     # hand.set_motor_target(5, pos, 0, 0)
    #     # if pos > 18000:
    #     #     dir = -1
        
    #     # if pos < 0:
    #     #     dir = 1
    #     # pos = dir*300 + pos
        
    #     hand.flush()
    #     time.sleep(dt)
        
    # while 1:
    #     q[5] += 1.5
    #     hand.set_joints_pos(q)
    #     time.sleep(dt)
    #     q[5] -= 1.5
    #     hand.set_joints_pos(q)
    #     time.sleep(dt)
    
    # 1.弯曲自由度演示
    for i in [2,1,5,4,7,9,11]:
        q[i] += 1.2
        hand.set_joints_pos(q)
        time.sleep(dt)
        q[i] -= 1.2
        hand.set_joints_pos(q)
        time.sleep(dt)
        
    # 2.测摆自由度演示
    dt = 0.5
    for i in [10,0,3,6,8]:
        q[i] -= 0.3
        hand.set_joints_pos(q)
        time.sleep(dt)    
    hand.action('tilt',delay=dt)
    hand.action('tilt_reverse',delay=dt)
    for i in reversed([10,0,3,6,8]):
        q[i] += 0.6
        hand.set_joints_pos(q)
        time.sleep(dt)    
    hand.action('open',delay=dt)    
    hand.action('diverge',delay=dt)    
    
    # 3.拇指自由度演示
    q = np.array([0,0.3,0,0,0.3,0,0,0,0,0,0,0,0.0])  
    q[-1] += 1.7
    q[-2] += 1
    hand.set_joints_pos(q)
    time.sleep(dt)
    q[-1] -= 1.7
    q[-2] -= 1
    hand.set_joints_pos(q)
    time.sleep(dt)
    
    q = np.array([0,0.3,0,0,0.3,0,0,0,0,0,0,0,0.0])  
    # 4.快速弯曲演示
    for _ in range(2):
        c = [[1,2],[4,5],[7],[9],[11]]
        for i in range(5):
            for id in c[i]:
                q[id] += 1.8
                hand.set_joints_pos(q)
            time.sleep(0.2)
        time.sleep(dt)
        for i in range(5):
            for id in c[i]:
                q[id] -= 1.8
                hand.set_joints_pos(q)
            time.sleep(0.2)    
        time.sleep(dt)
    # 5.数字演示
    for _ in range(1):
        hand.action('one')
        hand.action('two')
        hand.action('three')
        hand.action('four')
        hand.action('five')
        hand.action('six')
        hand.action('seven')
        hand.action('eight')
        hand.action('nine')
    hand.action('open',delay=2.0)
    
    # 6.接触演示
    hand.action('pinch1',delay=2.0)
    hand.action('pinch2',delay=2.0)
    hand.action('pinch3',delay=2.0)
    # hand.action('pinch4')
    
    # 7. 完成
    hand.action('ok') 
    time.sleep(2)
    hand.action('open',delay=dt) 
    
    
    # t = time.time()
    # while time.time()-t < 2:
    #     p = 1
    #     joints = np.array([0,p,p,0,p,p,0,p,0,p,0,0.7,0.5])
    #     hand.set_joints_pos(joints)
    # hand.action('init') 
    
    # for id in range(1,14):
    #     pos = 1
    #     dir = 1
    #     while(1):
    #         if pos >= 10000:
    #             dir = -1
    #         if pos <= 0:
    #             break
                
    #         hand.set_motor_target(id, pos, 0, 0)
    #         pos = dir*300 + pos
    #         hand.flush()
    #         time.sleep(0.01)
    #     time.sleep(0.5)
    # hand.action('init') 
    
    # p = 0
    # dir = 1
    # t = time.time()
    # while time.time()-t < 10:
    #     # print(f'cmd: {hand.cmd_}')
    #     if p >= 18000:
    #         dir = -1
    #     if p <= 0:
    #         dir = 1
    #     p = dir*300 + p
        
    #     pos = np.zeros(13)
    #     for id in range(13):    
    #         # if id == 9 or id == 10 or id == 11 or id == 12 or id == 13:
    #         if id == 9-1 or id == 10-1 or id == 11-1 or id == 12-1 or id == 13-1:
    #             pos[id] = 0
    #         else:
    #             pos[id] = p
    #     hand.set_motors_pos(pos)
    #     time.sleep(0.005)
    # hand.action('init') 
    
    # pos = 0
    # dir = 1
    # t = time.time()
    # while time.time()-t < 5:
    #     if pos >= 1.5:
    #         dir = -1
    #     if pos <= 0:
    #         dir = 1
            
    #     pos = dir*0.05 + pos
    #     joints = np.array([0,pos,0,0,pos,0,0,pos,0,pos,0,0,0])
    #     hand.set_joints_pos(joints)
    #     time.sleep(0.005)
    # hand.action('init') 
    
    # while(1):
    #     pass
    
    time.sleep(0.01)
    hand.close()
    
    
if __name__ == '__main__':
    run()
    
