#!/usr/bin/env python3
# -*-coding:utf8-*-
import rospy
import numpy as np
import time
import pygame
np.set_printoptions(precision=3, suppress=True)
from piper_msgs.msg import PosCmd
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState


class Joystick:
    def __init__(self, joystick_id):
        pygame.init()
        self.joystick_id = joystick_id

    def joystick_initialization(self):
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(self.joystick_id)
            self.joystick.init()
            print(f"Detected joystick: {self.joystick.get_name()}")
        else:
            print("No joystick detected.")
            raise ValueError("No joystick detected.")

    def _get_axis(self):
        axis_ls_horiz = self.joystick.get_axis(0)
        axis_ls_vert = self.joystick.get_axis(1)

        axis_rs_horiz = self.joystick.get_axis(3)
        axis_rs_vert = self.joystick.get_axis(4)

        axis_lb = self.joystick.get_axis(2)
        axis_rb = self.joystick.get_axis(5)

        return np.array([axis_ls_horiz, axis_ls_vert, axis_lb, axis_rs_horiz, axis_rs_vert, axis_rb])

    def _get_button(self):
        bu_a = self.joystick.get_button(0)
        bu_b = self.joystick.get_button(1)
        bu_x = self.joystick.get_button(2)
        bu_y = self.joystick.get_button(3)
        bu_lt = self.joystick.get_button(4)
        bu_rt = self.joystick.get_button(5)
        bu_start = self.joystick.get_button(6)
        bu_reset = self.joystick.get_button(7)

        return np.array([bu_a, bu_b, bu_x, bu_y, bu_lt, bu_rt, bu_start, bu_reset])

# [axis_ls_horiz, axis_ls_vert, axis_lb, axis_rs_horiz, axis_rs_vert, axis_rb, \
# bu_a, bu_b, bu_x, bu_y, bu_lt, bu_rt, bu_start, bu_reset]
    def get_joystick(self):
        pygame.event.pump()
        joystick_axis_sts = self._get_axis()
        joystick_button_sts = self._get_button()

        return np.concatenate([joystick_axis_sts, joystick_button_sts])

    def print_info(self):
        if pygame.joystick.get_count() > 0:
            for i in range(pygame.joystick.get_count()):
                print(
                    f"joystick name {i}: {pygame.joystick.Joystick(i).get_name()}")
        else:
            print("No joystick found")





class GamepadNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('gamepad_node', anonymous=True)
        
        # 创建订阅者，订阅手柄输入
        # self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        
        # 创建发布者，发布控制命令
        # self.cmd_pub = rospy.Publisher('pos_cmd', PosCmd, queue_size=10)
        self.joint_cmd_pub = rospy.Publisher('joint_ctrl_single', JointState, queue_size=10)
        self.piper_enabel_cmd = rospy.Publisher('enable_flag', Bool, queue_size=1)
        # 初始化Twist消息
        self.cmd = PosCmd()

        self.joint_cmd = JointState()
        self.joint_cmd.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        rospy.loginfo("Gamepad node initialized")


    def run_position_control(self):
            
            # Init Joystick
            joystick = Joystick(0)
            joystick.joystick_initialization()
            # piper pos init [x,y,z,rx,ry,rz]
            # piper_pos_init = [54.952, 0, 203.386, 0, 1.482, 0]
            piper_pos_init = [0, 0, 0, 0, 0, 0, 0]
            piper_pos = list(piper_pos_init)
            
            rate = rospy.Rate(100)  # 200 Hz
            angle_step = 0.2
            piper_enable_cmd = False
            piper_enable_cmd_prev = False
            while not rospy.is_shutdown():
                joystick_data = joystick.get_joystick()
                joy_axis_neg_indices = np.where(joystick_data < -0.9)[0]
                joy_sts_indices = np.where(joystick_data > 0.9)[0]

                for value in joy_axis_neg_indices:
                    # Joint_1
                    if value == 0:
                        # print("LS Left")
                        piper_pos[0] = piper_pos[0]+angle_step
                    # Joint_2
                    elif value == 1:
                        # print("LS Up")
                        piper_pos[1] = piper_pos[1]+angle_step
                    elif value == 2:
                        # print("LB Release")
                        continue
                    # Joint_5
                    elif value == 3:
                        #print("RS Left")
                        piper_pos[4] = piper_pos[4]+angle_step
                    # Joint_3
                    elif value == 4:
                        # print("RS Up")
                        piper_pos[2] = piper_pos[2]-angle_step
                    # Joint_4
                    elif value == 5:
                        # print("RB Release")
                        continue
        
                for value in joy_sts_indices:
                    # Joint_1
                    if value == 0:
                        # print("LS Right")
                        piper_pos[0] = piper_pos[0]-angle_step
                    # Joint_2
                    elif value == 1:
                        # print("LS Down")
                        piper_pos[1] = piper_pos[1]-angle_step
                    elif value == 2:
                        # print("LB Pressed")
                        # linear_vel = linear_vel - 0.0002
                        # if linear_vel < 0:
                        #     linear_vel = 0
                        # print(f"current vel: {linear_vel}")
                        pass
                    #Joint_5    
                    elif value == 3:
                        # print("RS Right")
                        piper_pos[4] = piper_pos[4]-angle_step
                    # Joint_3
                    elif value == 4:
                        # print("RS Down")
                        piper_pos[2] = piper_pos[2]+angle_step
                    elif value == 5:
                    #     # print("RB Pressed")
                    #     linear_vel = linear_vel + 0.0002
                    #     if linear_vel >0.3:
                    #         linear_vel = 0.3
                    #     print(f"current vel: {linear_vel}")
                          pass
                    elif value == 6:
                        # print("A Pressed")
                        pass

                    # Joint_6
                    elif value == 7:
                        # print("B Pressed")
                        piper_pos[5] = piper_pos[5]-angle_step
                    # Joint_6
                    elif value == 8:
                        # print("X Pressed")
                        piper_pos[5] = piper_pos[5]+angle_step
                    elif value == 9:
                        # print("Y Pressed")
                        pass
                    elif value == 10:
                        # print("LT Pressed")
                        # print("gripper open")
                        piper_pos[6] = piper_pos[6] + angle_step*0.1
                    elif value == 11:
                        # print("RT Pressed")
                        # print("gripper close")
                        piper_pos[6] = piper_pos[6] - angle_step*0.1
                    elif value == 12:
                        print("Start Pressed")
                        piper_enable_cmd = not piper_enable_cmd
                        last_change_time = time.time()
                    elif value == 13:
                        # print("reset")
                        piper_pos = list(piper_pos_init) 
                
                # 限制关节角度
                factor = np.pi/180
                piper_pos[0] = max(-154, min(154, piper_pos[0]))
                piper_pos[1] = max(0, min(195, piper_pos[1]))
                piper_pos[2] = max(-175, min(0, piper_pos[2]))
                piper_pos[3] = max(-106, min(106, piper_pos[3]))
                piper_pos[4] = max(-75, min(75, piper_pos[4]))
                piper_pos[5] = max(-100, min(100, piper_pos[5]))
                piper_pos[6] = max(0, min(4.58, piper_pos[6]))
                
                joints = []
                for i in range(7):
                    joints.append(piper_pos[i]*factor)

                # 
                if piper_enable_cmd != piper_enable_cmd_prev and time.time() - last_change_time > 0.5:
                    self.piper_enabel_cmd.publish(piper_enable_cmd)
                    piper_enable_cmd_prev = piper_enable_cmd

                # 发布关节角度  
                self.joint_cmd.header.stamp = rospy.Time.now()
                # self.joint_cmd.position = [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6] 
                self.joint_cmd.position = joints
                self.joint_cmd_pub.publish(self.joint_cmd)

                
                rate.sleep()


if __name__ == '__main__':
    try:
        node = GamepadNode()
        #node.run()
        node.run_position_control()
    except Exception as e:
        rospy.logerr(f"Error initializing gamepad node: {str(e)}") 