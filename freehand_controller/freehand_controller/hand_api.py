import sys
import os
import socket
import time
import logging
from copy import copy, deepcopy  # lnl: added
from threading import Thread, RLock
import math
from math import cos, sin
import numpy as np
import os
import re
import threading
from scipy.optimize import root
from freehand_controller.model import trans2_theta_psi_D
from freehand_controller.cfg import *
import freehand_controller.TinyFrame as TF
from freehand_controller.ExtCommProtocol import *

        
class Hand:
    target_ip = '192.168.2.122'
    target_port = 6667
    receive_port = 6666
    def __init__(self, adjust_time=3, is_right=1):
        self.create_logger()
        self.is_right_ = is_right
        self.adjust_time_ = adjust_time
        self.logger_.info("当前使用右手")
        
        self.lock_ = RLock()
        # self.transit_receive_thread_ = Thread(target=self.transit_receive_loop, daemon=True)
        self.cmd_ = FingerCommand()
        self.data_ = FingerData()
        
        self.start()   
            
    def create_logger(self):
        self.logger_ = logging.getLogger()
        self.logger_.setLevel(level=logging.INFO)
        handler = logging.FileHandler("freehand.log")
        handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        console = logging.StreamHandler()
        console.setLevel(logging.INFO)
        self.logger_.addHandler(handler)
        self.logger_.addHandler(console)
        self.logger_.info("Start log")
        
    def start(self):
        self.logger_.info("初始化...")
        self.tf = TF.TinyFrame()
        self.tf.SOF_BYTE = 0x01
        self.tf.ID_BYTES = 0x01
        self.tf.LEN_BYTES = 0x02
        self.tf.TYPE_BYTES = 0x01
        self.tf.CKSUM_TYPE = 'crc16'
        
        self.tf.write = self.tf_write
        self.tf.add_fallback_listener(self.tf_general_listener)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_r = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_r.bind(("0.0.0.0", self.receive_port))
        
        # wake up device
        self.sock.sendto(b'\x01'*7, (self.target_ip, self.target_port))
        time.sleep(0.1)
        
        self.buff = bytes()
        
        # 设置默认参数：电压,单片机使能,电机初始KP,KD,PWM,正反转
        self.set_motor_voltage(10.0)
        self.flush()
        for id in range(1,5):
            self.set_mcu_enabled(id, 1)
        self.flush()
            
        self.id_        = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
        self.enc_dir_   = [1, 0, 0, 0, 1, 0, 0, 0, 1, 1,  1,  0,  0]
        self.motor_dir_ = [1, 1, 1, 0, 1, 1, 0, 1, 1, 1,  0,  1,  1]
        self.motor_kp_  = [3, 2, 4, 4, 3, 2, 2, 2, 1, 1,  1,  4,  4]
        self.motor_kd_  = [2, 1, 0, 0, 2, 1, 1, 1, 1, 1,  1,  0,  0]
        # 请不要更改Kp,Kd参数
    
        # 电机初始参数
        for i in range(MOTOR_NUM):
            self.set_motor_ctrl_param(self.id_[i], 0, 0)
            self.set_motor_enc_param(self.id_[i], self.enc_dir_[i], 1)
        self.flush()
        for i in range(MOTOR_NUM):   
            self.set_motor_param(self.id_[i], 0, 7999, self.motor_dir_[i])
        self.flush()
        
        # 运动至零位
        t_start = time.time()
        while time.time() - t_start < 0.5:
            for i in range(MOTOR_NUM):
                if self.id_[i] == 9 or self.id_[i] == 10:
                    pass
                else:
                    self.set_motor_target(self.id_[i], 0, 0, 500)
            self.flush()
            time.sleep(0.01)

        while time.time() - t_start < 5:
            for i in range(MOTOR_NUM):
                if self.id_[i] == 9 or self.id_[i] == 10:
                    pass
                elif self.id_[i] == 1 or self.id_[i] == 5:
                    self.set_motor_target(self.id_[i], 0, 0, -5000)
                elif self.id_[i] == 11:
                    self.set_motor_target(self.id_[i], 0, 0, -4000)    
                else:
                    self.set_motor_target(self.id_[i], 0, 0, -5000)
            self.flush()
            time.sleep(0.01)
        self.logger_.info("回到零位")
        
        t_start = time.time()
        while time.time() - t_start < self.adjust_time_:
            for i in range(MOTOR_NUM):
                self.set_motor_target(self.id_[i], 0, 0, 0)
            self.flush()
            time.sleep(0.1)
        self.logger_.info("调整电机...")
        
        # 清零编码器
        for i in range(MOTOR_NUM):
            if self.id_[i] == 9 or self.id_[i] == 10:
                pass
            else:
                self.clear_enc(self.id_[i])
        self.flush()
            
        # 电机控制初始参数 
        for _ in range(3):   
            for i in range(MOTOR_NUM):
                self.set_motor_ctrl_param(self.id_[i], self.motor_kp_[i], self.motor_kd_[i])
                self.set_motor_enc_param(self.id_[i], self.enc_dir_[i], 1)
            self.flush()
            time.sleep(0.1)
        
        self.is_running_ = True
        # self.transit_receive_thread_.start()
        self.create_timer()
        self.logger_.info("启动完成")
            
    def stop(self):
        # self.logger_.debug(str(self.is_running_))
        with self.lock_:
            self.is_running_ = False
            
    def close(self):
        self.stop()   
        
        for i in range(MOTOR_NUM):
            self.set_motor_ctrl_param(self.id_[i], self.motor_kp_[i], self.motor_kd_[i])
        self.flush()
        time.sleep(0.1)
        for id in range(1,5):
            self.set_mcu_enabled(id, 0)    
        self.flush()
        time.sleep(0.1) 
        
        self.logger_.info("关闭...")    
        # self.transit_receive_thread_.join()

    def free_cmd(self):
        with self.lock_:
            self.cmd_.zero()
            
    def create_timer(self):
        t = threading.Timer(0.005, self.transit_receive)
        t.start()
        
    def transit_receive(self):
        with self.lock_:
            for id in range(MOTOR_NUM):
                # TODO：当前版本暂不提供更改kp,kd
                # self.set_motor_ctrl_param(id, self.cmd_.kpDes[id], self.cmd_.kdDes[id])
                # time.sleep(0.005)
                # self.logger_.info(f'{self.cmd_.qDes}')   
                self.set_motor_target(self.id_[id], self.cmd_.qDes[id], self.cmd_.qdDes[id], self.cmd_.tauDes[id])
            self.flush()
            self.logger_.debug('发送...')    
                    
        # receive
        data, _ = self.sock_r.recvfrom(1024)
        data_str = data.decode('utf-8')
        motor_data = {}
        motor_pattern = re.compile(r'\[M(\d+)\]enc=(-?\d+\.?\d*), ctrl=(-?\d+\.?\d*)')
        for match in motor_pattern.finditer(data_str):
            motor_id = int(match.group(1))
            enc = float(match.group(2))
            ctrl = float(match.group(3))
            motor_data[motor_id] = {'enc': enc, 'ctrl': ctrl}
        
        with self.lock_:          
            for i in range(MOTOR_NUM):
                self.data_.q[i] = motor_data[i]['enc']
            print(f'q_recv :{self.data_.q}') # 查看编码器数据
        
        with self.lock_:
            flag = self.is_running_
            if flag:
                self.create_timer()
                 
    def transit_receive_loop(self):
        # 该函数后续不再使用
        flag = self.is_running_
        while flag:
            self.transit_receive()
            with self.lock_:
                flag = self.is_running_
            self.logger_.debug(str(self.is_running_))
        self.logger_.debug('线程结束')
            
    # def transit_receive(self):
    #     # 该函数后续不再使用
    #     for id in range(MOTOR_NUM):
    #         # self.set_motor_ctrl_param(id, self.cmd_.kpDes[id], self.cmd_.kdDes[id])
    #         # time.sleep(0.005)
    #         self.set_motor_target(self.id_[id], self.cmd_.qDes[id], self.cmd_.qdDes[id], self.cmd_.tauDes[id])
    #     self.flush()
    #     time.sleep(0.002)
        
    def send_cmd(self, cmd : ExtCommPack):
        rawdata = cmd.pack()
        self.tf.send(cmd.pkt_type, rawdata)
        # time.sleep(0.01)
        
    def flush(self):
        # print(self.buff.count)
        self.sock.sendto(self.buff, (self.target_ip, self.target_port))
        self.buff = bytes()
        # time.sleep(0.001)
        
    def tf_write(self, data: bytes) -> int:
        # self.sock.sendto(data, (self.target_ip, self.target_port))  
        self.buff += data

    def tf_general_listener(self, frame):
        pass
    
    def set_motor_voltage(self, voltage):
        pkt = ExtCommMotorVoltage(voltage)
        self.send_cmd(pkt)
        
    def set_mcu_enabled(self, mcu_id, enabled):
        pkt = ExtCommMcuEnabled(mcu_id, enabled)
        self.send_cmd(pkt)
    
    def clear_enc(self, motor_id):
        pkt = ExtCommClearEnc(motor_id)
        self.send_cmd(pkt)
        
    def set_motor_ctrl_param(self, motor_id, kp, kd):
        pkt = ExtCommMotorCtrlParam(motor_id, kp, kd)
        self.send_cmd(pkt)

    def set_motor_enc_param(self, motor_id, cw_counting_dir, cnt2rad_ratio):
        pkt = ExtCommMotorEncParam(motor_id, cw_counting_dir, cnt2rad_ratio)
        self.send_cmd(pkt)
        
    def set_motor_param(self, motor_id, min_pwm, max_pwm, cw_dir_level):
        pkt = ExtCommMotorParam(motor_id, min_pwm, max_pwm, cw_dir_level)
        self.send_cmd(pkt)
    
    def set_motor_target(self, motor_id, dest_pos, dest_vel, dest_torque):
        # 针对单个电机
        pkt = ExtCommMotorTarget(motor_id, dest_pos, dest_vel, dest_torque)
        self.send_cmd(pkt)

    def set_motors_param(self, kp=10*np.ones(MOTOR_NUM), kd=0*np.ones(MOTOR_NUM)):
        # 使用默认kp,kd即可,因为电机有三种,需要手动调整
        with self.lock_:
            self.cmd_.kpDes = kp
            self.cmd_.kdDes = kd

    def set_motors_pos(self, pos=np.zeros(MOTOR_NUM)):
        with self.lock_:
            self.cmd_.qDes = pos
            self.cmd_.kpDes = np.array(self.motor_kp_)
            self.cmd_.kdDes = np.array(self.motor_kd_)
            
        # 调试：发送写在主线程
        # for i in range(MOTOR_NUM):
        #     # self.set_motor_ctrl_param(id, self.cmd_.kpDes[id], self.cmd_.kdDes[id])
        #     # time.sleep(0.005)
        #     self.set_motor_target(self.id_[i], pos[i], 0, 0)
        #     # time.sleep(0.002)
        # self.flush()
        # # time.sleep(0.01)

    def set_motors_pos_with_params(self, pos=np.zeros(MOTOR_NUM),
                                   kp=10*np.ones(MOTOR_NUM), 
                                   kd=0*np.ones(MOTOR_NUM)):
        with self.lock_:
            self.cmd_.qDes = pos
            self.cmd_.kpDes = kp
            self.cmd_.kdDes = kd
                    
    def set_motor_force(self, tau=np.zeros(MOTOR_NUM)):
        with self.lock_:
            self.cmd_.tauDes = tau
            self.cmd_.kpDes = np.zeros_like(tau)
            self.cmd_.kpDes = np.zeros_like(tau)
            
    def map_motor(self, x):
        # 映射电机指令
        # motor_map = [8,7,5,6,2,1,3,10,4,9,11,12,13]
        motor_map = [6,7,4,1,5,0,3,8,2,9,12,11,10]
        ratio = [256 * 4,
                 256 * 4,
                 256 * 64 * 4,
                 256 * 4,
                 256 * 4,
                 256 * 64 * 4,
                 256 * 4,
                 256 * 256 * 16,  # 2
                 256 * 4,
                 256 * 256 * 16,  # 2
                 256 * 4,
                 256 * 4,
                 256 * 275 * 4]
        x_map = np.zeros_like(x)
        for i in range(MOTOR_NUM):
            x_map[motor_map[i]] = x[i] * ratio[i]
            
        # self.logger_.debug(f'{x_map}')   
        return x_map

    def set_joints_pos(self, joints):
        pos = self.map_motor(self.ik_joints(joints))
        # self.logger_.info(f'pos: {pos}')
        self.set_motors_pos(pos)
        # self.set_motors_trajectory(pos)
        
    def set_motors_trajectory(self, pos_end, duration=1, count=1000, method='linear'):
        pos_start = self.get_motors_pos()
        if method=='linear':
            for i in range(1,count):
                pos = (pos_end-pos_start)*i/count + pos_start
                self.set_motors_pos(pos)
                time.sleep(0.0001)
        else:
            # TODO：规划速度
            pass
        
    def set_joints_trajectory(self, joints, duration=0.1, count=2, method='linear'):
        # TODO: 暂不使用轨迹规划模块
        pos0 = self.get_motors_pos()
        pos1 = self.map_motor(self.ik_joints(joints))
        if method=='linear':
            for i in range(1,count):
                pos = (pos1-pos0)*i/count + pos0
                self.set_motors_pos(pos)
                time.sleep(duration/count)
        else:
            # 理论上还应该规划速度，不然不连续
            pass
            
    def feedback_control_pos(self):
        # TODO: 关节空间的闭环控制
        pass
        
    def get_motors_data(self):
        # TODO:目前只有角度反馈
        motor_data = None
        with self.lock_:
            motor_data = deepcopy(self.data_)
        return motor_data

    def get_motors_pos(self):
        # 此处是编码器pos
        motor_data = None
        with self.lock_:
            motor_data = deepcopy(self.data_.q)
        return motor_data
    
    def get_joints_pos(self):
        # TODO: 正解函数正在开发
        motor_pos = None
        with self.lock_:
            motor_pos = deepcopy(self.data_.q)
            
        # motor_data -= self.init_table
        joints = self.fk_joints(motor_pos)
        return joints
    
    def get_module_config(self, name: str):
        return module_config[name]
    
    # 首先是所有四杆机构关系
    def four_bar_forward(self, q, A, B, C, D, degree1_init, degree2_init):
        '''求解四杆机构角度关系,例如从PIP推导DIP
        '''    
        L1=math.sqrt(A**2+C**2-2*A*C*math.cos(q+degree1_init))
        parameter1=(C-A*math.cos(q+degree1_init))/L1
        parameter2=(L1**2+D**2-B**2)/(2*L1*D)
        return degree2_init-(math.acos(parameter2)-math.acos(parameter1))
        
    def four_bar_backward(self, q, A, B, C, D, degree1_init, degree2_init):
        L2=math.sqrt(C**2+D**2-2*C*D*math.cos(degree2_init-q))
        parameter1=math.sin(degree2_init-q)*D/L2
        parameter2=(L2^2+A^2-B^2)/(2*L2*A)
        return math.acos(parameter2)-math.asin(parameter1)-degree1_init
    
    def four_bar_pip2dip(self, q: float, name: str):
        '''所有手指的dip都与pip耦合

        :param q: _description_
        :type q: float
        :param name: _description_
        :type name: str
        :return: _description_
        :rtype: _type_
        '''
        cfg = self.get_module_config(name)
        return self.four_bar_forward(q, cfg["A"],cfg["B"],cfg["C"],cfg["D"],cfg["degree1_init"],cfg["degree2_init"])
        
    def four_bar_dip2pip(self, q: float, name: str):
        cfg = self.get_module_config(name)
        return self.four_bar_backward(q, cfg["A"],cfg["B"],cfg["C"],cfg["D"],cfg["degree1_init"],cfg["degree2_init"])
        
    def four_bar_mcp2pip(self, q: float, name: str):
        assert (name=="fourth" or name=="little")
        cfg = self.get_module_config(name)
        return self.four_bar_forward(q, cfg["Am"],cfg["Bm"],cfg["Cm"],cfg["Dm"],cfg["degree1m_init"],cfg["degree2m_init"])
        
    def four_bar_pip2mcp(self, q: float, name: str):
        assert (name=="fourth" or name=="little")
        cfg = self.get_module_config(name)
        return self.four_bar_backward(q, cfg["Am"],cfg["Bm"],cfg["Cm"],cfg["Dm"],cfg["degree1m_init"],cfg["degree2m_init"])
        
    # ik   
    def ik_tf(self, H: np.ndarray): 
        # TODO: SE3 to q and d
        pass
    
    def ik_joints(self, q: np.ndarray):
        assert q.shape[0] == 13
        d_index = self.ik_index(q[:3])
        d_middle = self.ik_middle(q[3:6])
        t = time.time()
        d_fourth = self.ik_fourth(q[6:8])
        self.logger_.debug(f"time: {time.time()-t}")
        d_little = self.ik_little(q[8:10])
        d_thumb = self.ik_thumb(q[10:])
        # 这个地方统一写电机圈数，然后在map里乘对应编码器减速比
        d = np.concatenate([d_index,d_middle,d_fourth,d_little,d_thumb]) 
        return d

    def ik_driver2(self, q: np.ndarray, name: str):
        '''用于求解无名指与小拇指的逆运动学,其中第一个关节总是指定为直驱关节角度

        :param q: _description_
        :type q: np.ndarray 
        :param name: _description_
        :type name: str
        '''
        assert q.shape[0] == 2
        assert (name=="fourth" or name=="little")
        cfg = self.get_module_config(name)
        psi = q[0]
        theta = q[1]
        bevel = psi / (2*math.pi)
        
        # ts = time.time()
        # rx = np.array([[1, 0, 0],
        #                 [0,cos(psi),-sin(psi)],
        #                 [0,sin(psi),cos(psi)]])    
        # ry = np.array([[cos(theta),0,sin(theta)],
        #                 [0,1,0],
        #                 [-sin(theta),0,cos(theta)]])
        # r = rx @ ry  # 顺序决定高低（左右手）
        # p1r = r @ cfg["p1"]
        # p2r = r @ cfg["p2"]
        
        # def ik_psu(x):
        #     l2 = np.array([x[0], x[1], x[2]])
        #     p3 = cfg["p3"]
        #     p3[2] = x[3]
        #     l1 = p1r - p3 - l2
        #     equ = l2 - np.dot(l2,p2r)*p2r - np.dot(l1,l2)*l1/cfg["l1"]
            
        #     return [equ[0],
        #             equ[1],
        #             equ[2],
        #             np.linalg.norm(l2)-cfg["l2"]]
    
        # sol_psu = root(ik_psu, np.array([0, 0, 0, 0]), method='lm')
        # self.logger_.debug(f"sol_psu: {sol_psu}")
        # d = sol_psu.x[3]
        # self.logger_.debug(f"d_time: {time.time() -ts}")
        
        # ts = time.time()
        d = trans2_theta_psi_D(theta,psi)/1000
        # self.logger_.debug(f"d2_time: {time.time() -ts}")
        # self.logger_.debug(f"d: {d}")
        # print(d)
        # self.logger_.debug(f"d2: {d2}")
        # self.logger_.debug(f"d_direct: {bevel}")
        
        d = np.array([d/cfg["h"], bevel])
        return d
        
    def ik_driver3(self, q: np.ndarray, name: str):
        '''用于求解食指,中指以及大拇指的逆运动学,其中第三个关节总是指定为直驱关节角度

        :param q: _description_
        :type q: np.ndarray
        :param name: _description_
        :type name: str
        :return: _description_
        :rtype: _type_
        '''
        assert q.shape[0] == 3
        assert (name=="index" or name=="middle" or name=="thumb")
        cfg = self.get_module_config(name)
        
        psi = q[0]
        theta = q[1]
        phi = q[2]
        # self.logger_.debug(f"psi: {psi}")
        # self.logger_.debug(f"theta: {theta}")
        # self.logger_.debug(f"phi: {phi}")
        
        # MCP
        rx = np.array([[1, 0, 0],
                        [0,cos(psi),-sin(psi)],
                        [0,sin(psi),cos(psi)]])    
        ry = np.array([[cos(theta),0,sin(theta)],
                        [0,1,0],
                        [-sin(theta),0,cos(theta)]])
        r = rx @ ry
        p1r = r @ cfg["p1"]
        p2r = r @ cfg["p2"]
        # self.logger_.debug(f"p1r: {p1r}")
        
        d_left  = -(p1r[2] - math.sqrt(cfg["l1"]**2 - np.sum((p1r[:2]-cfg["p3"][:2])**2)))
        d_right = -(p2r[2] - math.sqrt(cfg["l2"]**2 - np.sum((p2r[:2]-cfg["p4"][:2])**2)))
        self.logger_.debug(f"d_left: {d_left}")
        self.logger_.debug(f"d_right: {d_right}")
        d_left  -= cfg["z0"]
        d_right -= cfg["z0"]
        # print(d_left)
        # d = np.array([d_left, d_right, phi*cfg["bevel"]])
        d = np.array([d_left/cfg["h"], d_right/cfg["h"], phi/(2*math.pi)])
        return d
        
    def ik_index(self, q: np.ndarray):
        return self.ik_driver3(q, "index")
    
    def ik_middle(self, q: np.ndarray): 
        return self.ik_driver3(q, "middle")
    
    def ik_fourth(self, q: np.ndarray):
        return self.ik_driver2(q, "fourth")
    
    def ik_little(self, q: np.ndarray):
        return self.ik_driver2(q, "little")
    
    def ik_thumb(self, q: np.ndarray):
        return self.ik_driver3(q, "thumb")
        
    # fk,将给出所有耦合关节的参数
    def fk_tf(self, d):
        # d to q and SE3
        pass
    
    def fk_joints(self):
        pass
    
    def fk_driver2(self, q: np.ndarray, name: str):
        '''用于求解无名指与小拇指的正运动学,其中不包含直驱关节

        :param q: _description_
        :type q: np.ndarray
        :param name: _description_
        :type name: str
        '''
        assert q.shape[0] == 2
        assert (name=="fourth" or name=="little")
        cfg = self.get_module_config(name)
        
       
    def fk_driver3(self, d: np.ndarray, name: str):
        '''用于求解食指,中指以及大拇指的正运动学,其中d的第三维度总是指定为直驱关节,尽管大拇指的直驱位置与食指不同

        :param d: _description_
        :type d: np.ndarray
        :param name: _description_
        :type name: str
        :return: _description_
        :rtype: _type_
        '''
        assert d.shape[0] == 3
        assert (name=="index" or name=="middle" or name=="thumb")
        cfg = self.get_module_config(name)
        d_left = d[0] # 定义求出的电机位移始终为+,便于发送电机指令,但坐标系中为负数
        d_right = d[1]
        d_direct = d[2]
        # self.logger_.debug(f"d_left: {d_left}")
        # self.logger_.debug(f"d_right: {d_right}")
        
        # pip
        pip = d_direct/cfg["bevel"]
        
        # dip,四杆机构
        L1=math.sqrt(cfg["A"]**2+cfg["C"]**2-2*cfg["A"]*cfg["C"]*math.cos(pip+cfg["degree1_init"]))
        parameter1=(cfg["C"]-cfg["A"]*math.cos(pip+cfg["degree1_init"]))/L1
        parameter2=(L1**2+cfg["D"]**2-cfg["B"]**2)/(2*L1*cfg["D"])
        dip=cfg["degree2_init"]-(math.acos(parameter2)-math.acos(parameter1))
        
        # mcp
        d_left  /= cfg["screw"]
        d_right /= cfg["screw"]
        d_left  += cfg["z0"]
        d_right += cfg["z0"]
        d_left = -d_left # TODO: 有无负号
        d_right = -d_right
        self.logger_.debug(f"d_left: {d_left}")
        self.logger_.debug(f"d_right: {d_right}")
        
        def fk_pss(x):
            rx = np.array([[1, 0, 0],
                            [0,cos(x[0]),-sin(x[0])],
                            [0,sin(x[0]),cos(x[0])]])
            ry = np.array([[cos(x[1]),0,sin(x[1])],
                            [0,1,0],
                            [-sin(x[1]),0,cos(x[1])]])
            r = rx @ ry
            # r = ry @ rx 
            p1r = r @ cfg["p1"]
            p2r = r @ cfg["p2"]
            p3 = cfg["p3"]
            p3[2] = d_left
            p4 = cfg["p4"]
            p4[2] = d_right
            return [np.linalg.norm(r @ p1r - p3) - cfg["l1"],
                    np.linalg.norm(r @ p2r - p4) - cfg["l2"]]
            # return [np.sum((r @ p1r - p3)**2) - cfg["l1"]**2,
            #         np.sum((r @ p2r - p4)**2) - cfg["l2"]**2]
            
        def fk_pss2(x):
            a1 = cfg["p1"][0]
            a2 = cfg["p1"][1]
            a3 = cfg["p1"][2]
            a4 = cfg["p3"][0]
            a5 = cfg["p3"][1]
            a6 = d_left
            b1 = cfg["p2"][0]
            b2 = cfg["p2"][1]
            b3 = cfg["p2"][2]
            b4 = cfg["p4"][0]
            b5 = cfg["p4"][1]
            b6 = d_right
            A1 = np.sum(cfg["p1"]**2) + np.sum(cfg["p3"]**2) - cfg["l1"] ** 2 
            A2 = np.sum(cfg["p2"]**2) + np.sum(cfg["p4"]**2) - cfg["l2"] ** 2 
            
            return [cos(x[1])*a1*a4 + sin(x[1])*a3*a4 + cos(x[0])*a2*a5 + sin(x[0])*a2*a6 + 
                    sin(x[0])*sin(x[1])*a1*a5 - sin(x[0])*cos(x[1])*a3*a5 + cos(x[0])*sin(x[1])*a1*a6 + cos(x[0])*cos(x[1])*a3*a6 - A1 / 2,
                    cos(x[1])*b1*b4 + sin(x[1])*b3*b4 + cos(x[0])*b2*b5 + sin(x[0])*b2*b6 + 
                    sin(x[0])*sin(x[1])*b1*b5 - sin(x[0])*cos(x[1])*b3*b5 + cos(x[0])*sin(x[1])*b1*b6 + cos(x[0])*cos(x[1])*b3*b6 - A2 / 2]
   
        sol_pss = root(fk_pss2, np.array([math.radians(10), math.radians(50)]), method='broyden1')
        # broyden1 hybr anderson (lm) anderson linearmixing diagbroyden excitingmixing
        # print(sol_pss)
        try:
            self.logger_.debug(f"sol_pss: {sol_pss}")
            # self.logger_.debug(f"sol_pss: {type(sol_pss.success)}")
            if sol_pss.success:
                psi = sol_pss.x[0]
                theta = sol_pss.x[1]  # 这句话最后写,urdf才需要减去,后面用解出来的q2???
                q = np.array([psi, theta, pip, dip])
            else:
                q = np.array([None, None, pip, dip])
            return q
        except OSError as reason:
            print(f'reason: {str(reason)}')
                    
    def fk_index(self, q: np.ndarray):
        return self.fk_driver3(q, "index")
    
    def fk_middle(self, q: np.ndarray): 
        return self.fk_driver3(q, "middle")
    
    def fk_fourth(self, q: np.ndarray):
        return self.fk_driver2(q, "fourth")
    
    def fk_little(self, q: np.ndarray):
        return self.fk_driver2(q, "little")
    
    def fk_thumb(self, q: np.ndarray):
        return self.fk_driver3(q, "thumb")
    
    def action(self, action_name, delay=1.5):
        joints = np.zeros(MOTOR_NUM)
        if action_name == "one":
            joints = np.array([0.0,0.3,0.0,0.0,1.9,1.5,0.0,1.8,0.0,1.8,0.0,1.2,1.2])
            self.set_joints_pos(joints)
            # time.sleep(0.1)
            # joints = np.array([0,0.3,0,0,1.5,0.9,0,1.5,0,1.5,0,0.7,0.0])
            # self.set_joints_pos(joints)
        elif action_name == "two" or action_name == "yeah":
            joints = np.array([-0.3,0.3,0.0,0.3,0.3,0.0,0.0,1.8,0.0,1.8,0.0,1.2,1.2])
            self.set_joints_pos(joints)
        elif action_name == "three":
            joints = np.array([-0.3,0.3,0.0,0.0,0.3,0.0,0.3,0.0,0.0,1.8,0.0,1.2,1.2])
            self.set_joints_pos(joints)
        elif action_name == "four":
            joints = np.array([-0.3,0.3,0.0,0.0,0.3,0.0,0.0,0.0,0.3,0.0,0.0,1.2,1.2])
            self.set_joints_pos(joints)
        elif action_name == "five":
            joints = np.array([-0.3,0.3,0.0,0.0,0.3,0.0,0.0,0.0,0.3,0.0,0.0,0.0,0.0])
            self.set_joints_pos(joints)
        elif action_name == "six":
            joints = np.array([0.0,1.8,1.5,0.0,1.8,1.5,0.0,1.8,0.3,0.0,0.0,0.0,0.0])
            self.set_joints_pos(joints)
        elif action_name == "seven":    
            joints = np.array([0.0,1.2,0.5,0.0,1.2,0.5,0.0,1.8,0.0,1.8,0.0,0.9,1.2])
            self.set_joints_pos(joints)
            time.sleep(0.8)
            joints = np.array([0.0,1.7,0.5,0.0,1.7,0.5,0.0,1.8,0.0,1.8,0.0,0.9,1.2])
            self.set_joints_pos(joints)
        elif action_name == "eight":    
            joints = np.array([0.0,0.3,0.0,0.0,1.9,1.5,0.0,1.8,0.0,1.8,0.0,0.0,0.5])
            self.set_joints_pos(joints)
        elif action_name == "nine":    
            joints = np.array([0.0,0.3,1.0,0.0,1.9,1.5,0.0,1.8,0.0,1.8,0.0,1.2,1.2])
            self.set_joints_pos(joints)
        elif action_name == "fist":
            joints = np.array([0.0,1.8,0.9,0.0,1.8,0.9,0.0,1.8,0.0,1.8,0.0,0.7,0.5])
        elif action_name == "init":
            joints = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
            self.set_joints_pos(joints)
        elif action_name == "open":
            joints = np.array([0.0,0.3,0.0,0.0,0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
            self.set_joints_pos(joints)    
        elif action_name == "ok":
            joints = np.array([0.0,1.5,0.7,0.0,0.3,0.0,0.0,0.0,0.3,0.0,0.0,0.9,1.0])
            self.set_joints_pos(joints)
            # time.sleep(0.1)
            # joints = np.array([0.0,1.5,0.9,0.0,0.3,0.0,0.0,0.0,0.3,0.0,0.0,0.7,0.0])
            # self.set_joints_pos(joints) 
        elif action_name == "tilt":
            joints = np.array([0.3,0.3,0.0,0.3,0.3,0.0,0.3,0.0,0.3,0.0,0.3,0.0,0.0])
            self.set_joints_pos(joints)
        elif action_name == "tilt_reverse":
            joints = np.array([-0.3,0.3,0.0,-0.3,0.3,0.0,-0.3,0.0,-0.3,0.0,-0.3,0.0,0.0])
            self.set_joints_pos(joints)
        elif action_name == "diverge":
            joints = np.array([-0.3,0.3,0.0,-0.3,0.3,0.0,0.3,0.0,0.3,0.0,0,0.0,0.0])
            self.set_joints_pos(joints)   
        elif action_name == "pinch1": # 大拇指+食指
            joints = np.array([0.0,1.5,0.7,0.0,0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.9,1.0])
            self.set_joints_pos(joints)   
        elif action_name == "pinch2":
            joints = np.array([0.0,0.3,0.0,0.0,1.6,0.7,0.0,0.0,0.0,0.0,0.0,1.0,1.3]) 
            self.set_joints_pos(joints)   
        elif action_name == "pinch3":
            joints = np.array([0.0,0.3,0.0,0.0,0.3,0.0,0.1,1.1,0.0,0.0,0.0,1.1,1.7]) 
            self.set_joints_pos(joints)   
        elif action_name == "pinch4":
            joints = np.array([0.0,0.3,0.0,0.0,0.3,0.0,0.0,0.0,0.0,1.05,0.0,1.1,1.8]) 
            self.set_joints_pos(joints)   
        time.sleep(delay)
        self.logger_.info('='*30)
        self.logger_.info(action_name)

        
def main():
    hand = Hand()
    # hand.set_motor_voltage(10.0)
    
    # hand.set_mcu_enabled(1, 0)
    
    # hand.clear_enc(1)
    # # hand.clear_enc(2)
    # # hand.clear_enc(3)
    # # hand.clear_enc(4)
    
    # hand.set_motor_ctrl_param(1, 0, 0)
    # hand.set_motor_enc_param(1, 1, 2)
    # hand.set_motor_param(1, 0, 8000, 0)
    # # hand.set_motor_target(3, 9, 10, 11)
    # pass

if __name__ == '__main__':
    main()