#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from sensor_msgs.msg import JointState

import math
import numpy as np
from freehand_controller.hand_api import Hand


class FreeHandDriver(Node):
    def __init__(self):
        super().__init__('freehand_ros_driver')
        
        self.joint_states_pub = self.create_publisher(JointState,"/joint_states", 10)
        self.joint_states = JointState()
        joint_name_urdf = ['fi1-1joint', 'fi1-2joint', 'fi1-3joint', 'fi1-4joint', \
            'fi2-1joint', 'fi2-2joint', 'fi2-3joint', 'fi2-4joint', \
            'fi3-1joint', 'fi3-2joint', 'fi3-3joint', 'fi3-4joint', \
            'fi4-1joint', 'fi4-2joint', 'fi4-3joint', 'fi4-4joint', \
            'fi5-1joint', 'fi5-2joint', 'fi5-3joint', 'fi5-4joint', ]
        self.joint_states.name = joint_name_urdf
        
        # joint_name_param = ['index_q1', 'index_q2', 'index_q3', 'index_q4', \
        #     'middle_q1', 'middle_q2', 'middle_q3', 'middle_q4', \
        #     'fourth_q1', 'fourth_q2', 'fourth_q3', 'fourth_q4', \
        #     'little_q1', 'little_q2', 'little_q3', 'little_q4', \
        #     'thumb_q1', 'thumb_q2', 'thumb_q3', 'thumb_q4', ]
        
        # 可以直接调节的
        joint_name_param = ['index_q1', 'index_q2', 'index_q3', \
            'middle_q1', 'middle_q2', 'middle_q3', \
            'fourth_q1', 'fourth_q2',  \
            'little_q1', 'little_q2',  \
            'thumb_q1', 'thumb_q2', 'thumb_q3']
        joint_side = ['index_q1', 'middle_q1', 'fourth_q1', 'little_q1', 'thumb_q1']
        # joint_limit_up = [0.3, 1.8, 1.8,0.3, 1.8, 1.8,0.3, 1.7,0.3, 1.7,0.3,1.7,1.7]
        joint_limit_up = [0.3, 2.0, 1.8, 0.3, 2.0, 1.8, 0.3, 1.8, 0.3, 1.8, 0.3, 1.8, 1.9]
        joint_limit_low = [-0.3, 0., 0.,-0.3, 0., 0.,-0.3, 0.,-0.3, 0.,-0.3,0.,0.]

        # joint_names = {}  # dict
        # for i in range(len(joint_name_urdf)):
        #     joint_names[joint_name_param[i]] = joint_name_urdf[i]
        for i in range(len(joint_name_param)):
            descriptor =  ParameterDescriptor(description='q (rad)',
                                             type = 3)
            ran = FloatingPointRange()
            ran.from_value = joint_limit_low[i]
            ran.to_value = joint_limit_up[i]
            
            # if joint_name_param[i] in joint_side:
            #     ran.from_value = -0.3
            #     ran.to_value = 0.3
            # else:
            #     ran.from_value = 0.
            #     ran.to_value = 1.7
                
            ran.step = 0.001
            descriptor.floating_point_range = [ran]
            self.declare_parameter(joint_name_param[i], 0., descriptor)

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.hand = Hand()

    def timer_callback(self):
        index_q1 = self.get_parameter('index_q1').get_parameter_value().double_value
        index_q2 = self.get_parameter('index_q2').get_parameter_value().double_value
        index_q3 = self.get_parameter('index_q3').get_parameter_value().double_value
        index_q4 = self.hand.four_bar_pip2dip(index_q3,'index')
        
        middle_q1 = self.get_parameter('middle_q1').get_parameter_value().double_value
        middle_q2 = self.get_parameter('middle_q2').get_parameter_value().double_value
        middle_q3 = self.get_parameter('middle_q3').get_parameter_value().double_value
        middle_q4 = self.hand.four_bar_pip2dip(middle_q3,'middle')
        
        fourth_q1 = self.get_parameter('fourth_q1').get_parameter_value().double_value
        fourth_q2 = self.get_parameter('fourth_q2').get_parameter_value().double_value
        fourth_q3 = self.hand.four_bar_mcp2pip(fourth_q2,'fourth')
        fourth_q4 = self.hand.four_bar_pip2dip(fourth_q3,'fourth')
        
        little_q1 = self.get_parameter('little_q1').get_parameter_value().double_value
        little_q2 = self.get_parameter('little_q2').get_parameter_value().double_value
        little_q3 = self.hand.four_bar_mcp2pip(little_q2,'little')
        little_q4 = self.hand.four_bar_pip2dip(little_q3,'little')
        
        thumb_q1 = self.get_parameter('thumb_q1').get_parameter_value().double_value
        thumb_q2 = self.get_parameter('thumb_q2').get_parameter_value().double_value
        thumb_q3 = self.get_parameter('thumb_q3').get_parameter_value().double_value
        thumb_q4 = self.hand.four_bar_pip2dip(thumb_q2,'thumb')
        
        joints = np.array([index_q1, index_q2, index_q3, \
            middle_q1, middle_q2, middle_q3, \
            fourth_q1, fourth_q2,  \
            little_q1, little_q2,  \
            thumb_q1, thumb_q2, thumb_q3])
        # self.get_logger().info(f'ik: {self.hand.ik_joints(joints)}')
        self.hand.set_joints_pos(joints)
        
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.header.frame_id = ""
        
        position = np.array([index_q1, index_q2, index_q3, index_q4, \
            middle_q1, middle_q2, middle_q3, middle_q4, \
            fourth_q1, fourth_q2, fourth_q3, fourth_q4, \
            little_q1, little_q2, little_q3, little_q4, \
            thumb_q3, thumb_q1, thumb_q2, thumb_q4,])
        self.joint_states.position = position.tolist()
        self.joint_states.velocity = []
        self.joint_states.effort = []
        self.joint_states_pub.publish(self.joint_states)


def main():
    rclpy.init()
    node = FreeHandDriver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
