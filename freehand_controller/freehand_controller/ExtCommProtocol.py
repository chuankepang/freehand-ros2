import struct
from abc import ABC, abstractmethod

# 定义 FLOAT_SER_FMT 类型
FLOAT_SER_FMT = 'i'

# 定义结构体
class ExtCommPack:
    def __init__(self):
        self.pkt_type = 0xff
        
    @abstractmethod
    def pack(self):
        pass

EXTCOMM_SET_MOTOR_VOLTAGE = 0x00

class ExtCommMotorVoltage(ExtCommPack):
    def __init__(self, voltage):
        self.voltage = voltage
        self.pkt_type = EXTCOMM_SET_MOTOR_VOLTAGE

    def pack(self):
        # 将结构体打包为 bytes
        return struct.pack('<' + FLOAT_SER_FMT, int(self.voltage * (1 << 9)))

EXTCOMM_SET_MCU_ENABLED = 0x01

class ExtCommMcuEnabled(ExtCommPack):
    def __init__(self, mcu_id, enabled):
        self.mcu_id = mcu_id
        self.enabled = enabled
        self.pkt_type = EXTCOMM_SET_MCU_ENABLED

    def pack(self):
        # 将结构体打包为 bytes
        return struct.pack('<BB', self.mcu_id, self.enabled)

EXTCOMM_CLEAR_ENC = 0x02

class ExtCommClearEnc(ExtCommPack):
    def __init__(self, motor_id):
        self.motor_id = motor_id
        self.pkt_type = EXTCOMM_CLEAR_ENC

    def pack(self):
        # 将结构体打包为 bytes
        return struct.pack('<B', self.motor_id)

EXTCOMM_SET_MOTOR_CTRL_PARAM = 0x03

class ExtCommMotorCtrlParam(ExtCommPack):
    def __init__(self, motor_id, kp, kd):
        self.motor_id = motor_id
        self.kp = kp
        self.kd = kd
        self.pkt_type = EXTCOMM_SET_MOTOR_CTRL_PARAM

    def pack(self):
        # 将结构体打包为 bytes
        return struct.pack('<' + 'B' + FLOAT_SER_FMT * 2, self.motor_id, int(self.kp * (1 << 9)), int(self.kd * (1 << 9)))

EXTCOMM_SET_MOTOR_PARAM = 0x04

class ExtCommMotorParam(ExtCommPack):
    def __init__(self, motor_id, min_pwm, max_pwm, cw_dir_level):
        self.motor_id = motor_id
        self.min_pwm = min_pwm
        self.max_pwm = max_pwm
        self.cw_dir_level = cw_dir_level
        self.pkt_type = EXTCOMM_SET_MOTOR_PARAM

    def pack(self):
        # 将结构体打包为 bytes
        return struct.pack('<BHHB', self.motor_id, self.min_pwm, self.max_pwm, self.cw_dir_level)

EXTCOMM_SET_MOTOR_ENC_PARAM = 0x05

class ExtCommMotorEncParam(ExtCommPack):
    def __init__(self, motor_id, cw_counting_dir, cnt2rad_ratio):
        self.motor_id = motor_id
        self.cw_counting_dir = cw_counting_dir
        self.cnt2rad_ratio = cnt2rad_ratio
        self.pkt_type = EXTCOMM_SET_MOTOR_ENC_PARAM

    def pack(self):
        # 将结构体打包为 bytes
        return struct.pack('<BB' + FLOAT_SER_FMT, self.motor_id, self.cw_counting_dir, int(self.cnt2rad_ratio * (1 << 9)))

EXTCOMM_SET_MOTOR_TARGET = 0x06

class ExtCommMotorTarget(ExtCommPack):
    def __init__(self, motor_id, dest_pos, dest_vel, dest_torque):
        self.motor_id = motor_id
        self.dest_pos = dest_pos
        self.dest_vel = dest_vel
        self.dest_torque = dest_torque
        self.pkt_type = EXTCOMM_SET_MOTOR_TARGET

    def pack(self):
        # 将结构体打包为 bytes
        return struct.pack('<B' + FLOAT_SER_FMT * 3, self.motor_id, int(self.dest_pos * (1 << 9)), int(self.dest_vel * (1 << 9)), int(self.dest_torque * (1 << 9)))