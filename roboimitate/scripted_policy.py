#计算机器人测试操作数据
import numpy as np
import matplotlib.pyplot as plt
from pyquaternion import Quaternion

from constants import SIM_TASK_CONFIGS,B_2_N
from ee_sim_env import make_ee_sim_env

import IPython
e = IPython.embed
from ctypes import *


class BasePolicy:
    def __init__(self, inject_noise=False):
        self.inject_noise = inject_noise
        self.step_count = 0
        self.left_trajectory = None
        self.right_trajectory = None

    def generate_trajectory(self, ts_first):
        raise NotImplementedError

    @staticmethod
    def interpolate(curr_waypoint, next_waypoint, t):
        t1= (next_waypoint["t"] - curr_waypoint["t"])
        if(t1 != 0):
            t_frac = (t - curr_waypoint["t"]) / (next_waypoint["t"] - curr_waypoint["t"])
            curr_xyz = curr_waypoint['xyz']
            curr_quat = curr_waypoint['quat']
            curr_grip = curr_waypoint['gripper']
            next_xyz = next_waypoint['xyz']
            next_quat = next_waypoint['quat']
            next_grip = next_waypoint['gripper']
            xyz = curr_xyz + (next_xyz - curr_xyz) * t_frac
            quat = curr_quat + (next_quat - curr_quat) * t_frac
            gripper = curr_grip + (next_grip - curr_grip) * t_frac
            return xyz, quat, gripper
        else:
            return curr_waypoint['xyz'],curr_waypoint['quat'],curr_waypoint['gripper']

    def __call__(self, ts):#根据末端位置、末端姿态，求解轨迹
        # generate trajectory at first timestep, then open-loop execution
        if self.step_count ==0:
            self.generate_trajectory(ts)

        # obtain left and right waypoints，当前、下一个
        #print( self.left_trajectory[0]['t'] )
        if self.left_trajectory[0]['t'] == self.step_count:
            self.curr_left_waypoint = self.left_trajectory.pop(0)
        next_left_waypoint = self.left_trajectory[0]
        
        if self.right_trajectory[0]['t'] == self.step_count:
            self.curr_right_waypoint = self.right_trajectory.pop(0)
        next_right_waypoint = self.right_trajectory[0]

        # 在当前和下一个waypoint之间，插值。获取当前时刻，应有的末端位置和姿态
        left_xyz, left_quat, left_gripper = self.interpolate(self.curr_left_waypoint, next_left_waypoint, self.step_count)
        right_xyz, right_quat, right_gripper = self.interpolate(self.curr_right_waypoint, next_right_waypoint, self.step_count)

        #print(left_xyz)
        # Inject noise
        if self.inject_noise:#增加噪声
            scale = 0.01
            left_xyz = left_xyz + np.random.uniform(-scale, scale, left_xyz.shape)
            right_xyz = right_xyz + np.random.uniform(-scale, scale, right_xyz.shape)

        action_left = np.concatenate([left_xyz, left_quat, [left_gripper]])#串联数据
        action_right = np.concatenate([right_xyz, right_quat, [right_gripper]])

        self.step_count += 1
        return np.concatenate([action_left, action_right])


class PickAndTransferPolicy(BasePolicy):

    def generate_trajectory(self, ts_first):
        #左右手末端夹子位置
        init_mocap_pose_right = ts_first.observation['mocap_pose_right']
        init_mocap_pose_left = ts_first.observation['mocap_pose_left']

        box_info = np.array(ts_first.observation['env_state'])
        box_xyz = box_info[:3]
        box_quat = box_info[3:]
        print(f"Generate trajectory for {box_xyz}")

        gripper_pick_quat = Quaternion(init_mocap_pose_right[3:])#机械臂的默认航向
        gripper_pick_quat = gripper_pick_quat * Quaternion(axis=[0.0, 1.0, 0.0], degrees=-60)

        meet_left_quat = Quaternion(axis=[1.0, 0.0, 0.0], degrees=90)

        meet_xyz = np.array([0, 0.5, 0.25])#交互点   需要明确坐标系  0左右  1前后  2高度
        lr_off=0.02
        #一个一个点
        self.left_trajectory = [#时间 位置与姿态  后面输入给运动学逆解---可以替换为外部操控
            {"t": 0, "xyz": init_mocap_pose_left[:3], "quat": init_mocap_pose_left[3:], "gripper": 0}, # sleep
            {"t": 200, "xyz": meet_xyz + np.array([-0.1, 0, -0.02]), "quat": meet_left_quat.elements, "gripper": 1}, # approach meet position
            {"t": 260, "xyz": meet_xyz + np.array([0.02, 0, -0.02]), "quat": meet_left_quat.elements, "gripper": 1}, # move to meet position
            {"t": 310, "xyz": meet_xyz + np.array([0.02, 0, -0.02]), "quat": meet_left_quat.elements, "gripper": 0}, # close gripper
            {"t": 360, "xyz": meet_xyz + np.array([-0.1, 0, -0.02]), "quat": np.array([1, 0, 0, 0]), "gripper": 0}, # move left
            {"t": 400, "xyz": meet_xyz + np.array([-0.1, 0, -0.02]), "quat": np.array([1, 0, 0, 0]), "gripper": 0}, # stay
        ]

        self.right_trajectory = [
            {"t": 0, "xyz":   init_mocap_pose_right[:3], "quat": init_mocap_pose_right[3:], "gripper": 0}, # sleep
            {"t": 90, "xyz":  box_xyz + np.array([lr_off, 0, 0.08]), "quat": gripper_pick_quat.elements, "gripper": 1}, # approach the cube
            {"t": 130, "xyz": box_xyz + np.array([lr_off, 0, -0.017]), "quat": gripper_pick_quat.elements, "gripper": 1}, # go down
            {"t": 170, "xyz": box_xyz + np.array([lr_off, 0, -0.017]), "quat": gripper_pick_quat.elements, "gripper": 0}, # close gripper
            {"t": 200, "xyz": meet_xyz + np.array([0.05, 0, 0.01]), "quat": gripper_pick_quat.elements, "gripper": 0}, # approach meet position
            {"t": 220, "xyz": meet_xyz + np.array([0.05, 0, 0.01]), "quat": gripper_pick_quat.elements, "gripper": 0}, # move to meet position
            {"t": 310, "xyz": meet_xyz + np.array([0.05, 0, 0.01]), "quat": gripper_pick_quat.elements, "gripper": 1}, # open gripper
            {"t": 360, "xyz": meet_xyz + np.array([0.1, 0, 0]), "quat": gripper_pick_quat.elements, "gripper": 1}, # move to right
            {"t": 400, "xyz": meet_xyz + np.array([0.1, 0, 0]), "quat": gripper_pick_quat.elements, "gripper": 1}, # stay
        ]


class PickAndTransferPolicy_human(BasePolicy):##------人工数据---------人形机器人Cube lyx

    def euler_to_quaternion(self,euler, degree_mode=1):
        #roll, pitch, yaw = euler
        pitch = euler[0]
        roll = euler[1]
        yaw = euler[2]
        # degree_mode=1:【输入】是角度制，否则弧度制
        if degree_mode == 1:
            roll = np.deg2rad(roll)
            pitch = np.deg2rad(pitch)
            yaw = np.deg2rad(yaw)

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        q = np.array([qw, qx, qy, qz])
        return q
 
    def generate_trajectory(self, ts_first):
        #初始值：
        att_init=[0,0,0]#pitch roll yaw
        pos_init=[0.12,0.17,-0.13]#x y z 机体坐标系
        
        pos_init=np.array(pos_init)+np.array(B_2_N)#转换为全局

        quad_init=self.euler_to_quaternion(att_init)
        print("att_init=",att_init,"quad_init=",quad_init)
        init_pose_left =   np.array([pos_init[0],pos_init[1],pos_init[2],quad_init[0],quad_init[1],quad_init[2],quad_init[3]])
        init_pose_right =  np.array([pos_init[0],-pos_init[1],pos_init[2],quad_init[0],quad_init[1],quad_init[2],quad_init[3]])

        box_info = np.array(ts_first.observation['env_state'])[:7] #一个box是7,red_box,
        box_xyz = box_info[:3]
        box_quat = box_info[3:]
        
        blue_box_info = np.array(ts_first.observation['env_state'])[7:14] #一个box是7
        blue_box_xyz = blue_box_info[:3]
        blue_box_quat = blue_box_info[3:]

        green_box_info = np.array(ts_first.observation['env_state'])[14:] #一个box是7
        green_box_xyz = green_box_info[:3]
        green_box_quat = green_box_info[3:]

        print(f"RED box pos in N：{box_xyz}")
        print(f"BLUE box pos in N：{blue_box_xyz}")
        print(f"GREEN box pos in N：{green_box_xyz}")
        
        gripper_pick_quat_left = Quaternion(init_pose_left[3:])#机械臂的默认航向 对应的四元数
        gripper_pick_quat_left = gripper_pick_quat_left * Quaternion(axis=[0.0, 1.0, 0.0], degrees=60)#进行旋转
        gripper_place_quat_left = Quaternion(init_pose_left[3:])#机械臂的默认航向 对应的四元数
        gripper_place_quat_left = gripper_place_quat_left * Quaternion(axis=[0.0, 0.0, -1.0], degrees=0)#进行旋转

        gripper_pick_quat_right = Quaternion(init_pose_right[3:])#机械臂的默认航向 对应的四元数
        gripper_pick_quat_right = gripper_pick_quat_right * Quaternion(axis=[0.0, 1.0, 0.0], degrees=60)#进行旋转
        gripper_place_quat_right = Quaternion(init_pose_right[3:])#机械臂的默认航向 对应的四元数
        gripper_place_quat_right = gripper_place_quat_right * Quaternion(axis=[0.0, 0.0, -1.0], degrees=0)#进行旋转
 
        #末端轨迹，世界坐标系 
        self.left_trajectory = [#该时刻的末端位置 该时刻的末端姿态 该时刻的夹爪。 后面输入给运动学逆解---可以替换为外部操控 -往右边
            {"t": 0, "xyz": init_pose_left[:3], "quat": init_pose_left[3:], "gripper": 1}, # sleep
            {"t": 50, "xyz": box_xyz[:3] + np.array([-0.07, -0.05, 0.15]), "quat":  gripper_pick_quat_left.elements, "gripper": 0.6}, # approach meet position
            {"t": 100, "xyz":  box_xyz[:3] + np.array([-0.07, -0.05, 0.026]), "quat": gripper_pick_quat_left.elements, "gripper": 0.6}, # approach the cube
            {"t": 120, "xyz":  box_xyz[:3] + np.array([-0.07, -0.05, 0.026]), "quat": gripper_pick_quat_left.elements, "gripper": 0.0}, # approach the cube
            {"t": 150, "xyz":  box_xyz[:3] + np.array([-0.07, -0.05, 0.026]), "quat": gripper_pick_quat_left.elements, "gripper": 0.0}, # approach the cube
            {"t": 180, "xyz":  init_pose_left[:3], "quat": init_pose_left[3:], "gripper": 0}, # approach the cube
            # # {"t": 190, "xyz":  np.array([0.2, 0.375, 0.15]), "quat": gripper_place_quat_left.elements, "gripper": 0.0}, # approach the cube
            # # {"t": 250, "xyz":  np.array([0.2, 0.375, 0.15]), "quat": gripper_place_quat_left.elements, "gripper": 1}, # approach the cube
            {"t": 300, "xyz":  blue_box_xyz[:3]+np.array([-0.07, -0.05, 0.026]) , "quat": gripper_pick_quat_left.elements, "gripper": 0.0}, # approach the cube
            # {"t": 300, "xyz":  blue_box_xyz[:3] + np.array([-0.07, -0.01, 0.15]), "quat": gripper_place_quat_left.elements, "gripper": 1}, # approach the cube
            #{"t": 220, "xyz":  init_pose_left[:3], "quat": gripper_place_quat_left.elements, "gripper": 1}, # approach the cube
            # {"t": 400, "xyz":  init_pose_left[:3], "quat": init_pose_left[3:], "gripper": 1}, # approach the cube
        ]

        self.right_trajectory = [
            {"t": 0, "xyz": init_pose_right[:3], "quat": init_pose_right[3:], "gripper": 1}, # sleep
            # {"t": 50, "xyz": init_pose_right[:3] , "quat": init_pose_right[3:], "gripper": 1}, # sleep
            # {"t": 100, "xyz": init_pose_right[:3], "quat":  init_pose_right[3:], "gripper": 1}, # sleep
            # {"t": 120, "xyz": init_pose_right[:3] , "quat":  init_pose_right[3:], "gripper": 0.0}, # sleep
            # {"t": 300, "xyz": init_pose_right[:3], "quat":  init_pose_right[3:], "gripper": 0.0}, # sleep
            {"t": 50, "xyz": green_box_xyz[:3] + np.array([-0.07, -0.01, 0.15]), "quat":  gripper_pick_quat_right.elements, "gripper": 1}, # approach meet position
            {"t": 100, "xyz":  green_box_xyz[:3] + np.array([-0.07, -0.01, 0.026]), "quat": gripper_pick_quat_right.elements, "gripper": 1}, # approach the cube
            {"t": 120, "xyz":  green_box_xyz[:3] + np.array([-0.07, -0.01, 0.026]), "quat": gripper_pick_quat_right.elements, "gripper": 0.0}, # approach the cube
            {"t": 135, "xyz":  green_box_xyz[:3] + np.array([-0.07, -0.01, 0.026]), "quat": gripper_pick_quat_right.elements, "gripper": 0.0},
            {"t": 170, "xyz":  init_pose_right[:3], "quat": init_pose_right[3:], "gripper": 0}, # approach the cube
            {"t": 190, "xyz":  np.array([0.2, -0.375, 0.15]), "quat": gripper_place_quat_right.elements, "gripper": 0.0}, # approach the cube
            {"t": 300, "xyz":  np.array([0.2, -0.375, 0.15]), "quat": gripper_place_quat_right.elements, "gripper": 1}, # approach the cube
            #{"t": 220, "xyz":  init_pose_right[:3], "quat": gripper_place_quat_right.elements, "gripper": 1}, # approach the cube
            #{"t": 400, "xyz":  init_pose_right[:3], "quat": init_pose_right[3:], "gripper": 1}, # approach the cube
        ]


class PickAndTransferPolicy_human1(BasePolicy):##------人工数据---------人形机器人 Slope

    def euler_to_quaternion(self,euler, degree_mode=1):
        #roll, pitch, yaw = euler
        pitch = euler[0]
        roll = euler[1]
        yaw = euler[2]
        # degree_mode=1:【输入】是角度制，否则弧度制
        if degree_mode == 1:
            roll = np.deg2rad(roll)
            pitch = np.deg2rad(pitch)
            yaw = np.deg2rad(yaw)

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        q = np.array([qw, qx, qy, qz])
        return q
 
    def generate_trajectory(self, ts_first):
        #初始值：
        att_init=[0,0,0]#pitch roll yaw
        pos_init=[0.12,0.17,-0.13]#x y z 机体坐标系
        
        pos_init=np.array(pos_init)+np.array(B_2_N)#转换为全局

        quad_init=self.euler_to_quaternion(att_init)
        print("att_init=",att_init,"quad_init=",quad_init)
        init_pose_left =   np.array([pos_init[0],pos_init[1],pos_init[2],quad_init[0],quad_init[1],quad_init[2],quad_init[3]])
        init_pose_right =  np.array([pos_init[0],-pos_init[1],pos_init[2],quad_init[0],quad_init[1],quad_init[2],quad_init[3]])
        
        box_info = np.array(ts_first.observation['env_state'])[:7] #一个box是7,red_box,
        box_xyz = box_info[:3]
        box_quat = box_info[3:]
        
        blue_box_info = np.array(ts_first.observation['env_state'])[7+7:7+7+7] #一个box是7,red_box
        blue_box_xyz = blue_box_info[:3]
        blue_box_quat = blue_box_info[3:]
        print(f"RED box pos in N：{box_xyz}")
        print(f"BLUE box pos in N：{blue_box_xyz}")
        
        gripper_pick_quat_left = Quaternion(init_pose_left[3:])#机械臂的默认航向 对应的四元数
        gripper_pick_quat_left = gripper_pick_quat_left * Quaternion(axis=[0.0, 1.0, 0.0], degrees=60)#进行旋转
        gripper_place_quat_left = Quaternion(init_pose_left[3:])#机械臂的默认航向 对应的四元数
        gripper_place_quat_left = gripper_place_quat_left * Quaternion(axis=[0.0, 0.0, -1.0], degrees=0)#进行旋转

        gripper_pick_quat_right = Quaternion(init_pose_right[3:])#机械臂的默认航向 对应的四元数
        gripper_pick_quat_right = gripper_pick_quat_right * Quaternion(axis=[0.0, 1.0, 0.0], degrees=60)#进行旋转
        gripper_place_quat_right = Quaternion(init_pose_right[3:])#机械臂的默认航向 对应的四元数
        gripper_place_quat_right = gripper_place_quat_right * Quaternion(axis=[0.0, 0.0, -1.0], degrees=0)#进行旋转
 
        #末端轨迹，世界坐标系 
        self.left_trajectory = [#该时刻的末端位置 该时刻的末端姿态 该时刻的夹爪。 后面输入给运动学逆解---可以替换为外部操控 -往右边
            {"t": 0, "xyz": init_pose_left[:3], "quat": init_pose_left[3:], "gripper": 1}, # sleep
            {"t": 50, "xyz": box_xyz[:3] + np.array([-0.07, -0.035, 0.15]), "quat":  gripper_pick_quat_left.elements, "gripper": 0.6}, # approach meet position
            {"t": 100, "xyz":  box_xyz[:3] + np.array([-0.07, -0.035, 0.026]), "quat": gripper_pick_quat_left.elements, "gripper": 0.6}, # approach the cube
            {"t": 120, "xyz":  box_xyz[:3] + np.array([-0.07, -0.035, 0.026]), "quat": gripper_pick_quat_left.elements, "gripper": 0.0}, # approach the cube
            {"t": 135, "xyz":  box_xyz[:3] + np.array([-0.07, -0.035, 0.026]), "quat": gripper_pick_quat_left.elements, "gripper": 0.0}, # approach the cube
            {"t": 170, "xyz":  init_pose_left[:3], "quat": init_pose_left[3:], "gripper": 0}, # approach the cube
            {"t": 190, "xyz":  np.array([0.2, 0.375, 0.15]), "quat": gripper_place_quat_left.elements, "gripper": 0.0}, # approach the cube
            {"t": 250, "xyz":  np.array([0.2, 0.375, 0.15]), "quat": gripper_place_quat_left.elements, "gripper": 1}, # approach the cube
            {"t": 540, "xyz":  init_pose_left[:3], "quat": init_pose_left[3:], "gripper": 1}, # approach the cube
        ]

        self.right_trajectory = [
            {"t": 0, "xyz": init_pose_right[:3], "quat": init_pose_right[3:], "gripper": 1}, # sleep
            {"t": 50, "xyz": blue_box_xyz[:3] + np.array([-0.07, -0.01, 0.15]), "quat":  gripper_pick_quat_right.elements, "gripper": 1}, # approach meet position
            {"t": 100, "xyz":  blue_box_xyz[:3] + np.array([-0.07, -0.01, 0.026]), "quat": gripper_pick_quat_right.elements, "gripper": 1}, # approach the cube
            {"t": 120, "xyz":  blue_box_xyz[:3] + np.array([-0.07, -0.01, 0.026]), "quat": gripper_pick_quat_right.elements, "gripper": 0.0}, # approach the cube
            {"t": 135, "xyz":  blue_box_xyz[:3] + np.array([-0.07, -0.01, 0.026]), "quat": gripper_pick_quat_right.elements, "gripper": 0.0},
            {"t": 190, "xyz":  np.array([0.258, -0.2, 0.11]), "quat": gripper_place_quat_right.elements, "gripper": 0.0}, # approach the cube
            {"t": 250, "xyz":  np.array([0.268, -0.05, 0.11]), "quat": gripper_place_quat_right.elements, "gripper": 0.0}, # approach the cube
            {"t": 280, "xyz":  np.array([0.03, -0.05, 0.11]), "quat": gripper_place_quat_right.elements, "gripper": 0.0}, # approach the cube
            {"t": 320, "xyz":  np.array([0.03, -0.2, 0.11]), "quat": gripper_place_quat_right.elements, "gripper": 0.0}, # approach the cube
            {"t": 340, "xyz":  np.array([0.258, -0.2, 0.11]), "quat": gripper_place_quat_right.elements, "gripper": 0.0}, # approach the cube
            {"t": 360, "xyz":  np.array([0.268, -0.05, 0.11]), "quat": gripper_place_quat_right.elements, "gripper": 0.0}, # approach the cube
            {"t": 380, "xyz":  np.array([0.03, -0.05, 0.11]), "quat": gripper_place_quat_right.elements, "gripper": 0.0}, # approach the cube
            {"t": 420, "xyz":  np.array([0.03, -0.2, 0.11]), "quat": gripper_place_quat_right.elements, "gripper": 0.0}, # approach the cube
            {"t": 440, "xyz":  np.array([0.258, -0.2, 0.11]), "quat": gripper_place_quat_right.elements, "gripper": 0.0}, # approach the cube
            {"t": 460, "xyz":  np.array([0.268, -0.05, 0.11]), "quat": gripper_place_quat_right.elements, "gripper": 0.0}, # approach the cube
            {"t": 480, "xyz":  np.array([0.03, -0.05, 0.11]), "quat": gripper_place_quat_right.elements, "gripper": 0.0}, # approach the cube
            {"t": 500, "xyz":  np.array([0.03, -0.2, 0.11]), "quat": gripper_place_quat_right.elements, "gripper": 0.0}, # approach the cube
                      
        ]

class InsertionPolicy(BasePolicy):

    def generate_trajectory(self, ts_first):
        init_mocap_pose_right = ts_first.observation['mocap_pose_right']
        init_mocap_pose_left = ts_first.observation['mocap_pose_left']

        peg_info = np.array(ts_first.observation['env_state'])[:7]
        peg_xyz = peg_info[:3]
        peg_quat = peg_info[3:]

        socket_info = np.array(ts_first.observation['env_state'])[7:]
        socket_xyz = socket_info[:3]
        socket_quat = socket_info[3:]

        gripper_pick_quat_right = Quaternion(init_mocap_pose_right[3:])
        gripper_pick_quat_right = gripper_pick_quat_right * Quaternion(axis=[0.0, 1.0, 0.0], degrees=-60)

        gripper_pick_quat_left = Quaternion(init_mocap_pose_right[3:])
        gripper_pick_quat_left = gripper_pick_quat_left * Quaternion(axis=[0.0, 1.0, 0.0], degrees=60)

        meet_xyz = np.array([0, 0.5, 0.15])
        lift_right = 0.00715

        self.left_trajectory = [
            {"t": 0, "xyz": init_mocap_pose_left[:3], "quat": init_mocap_pose_left[3:], "gripper": 0}, # sleep
            {"t": 120, "xyz": socket_xyz + np.array([0, 0, 0.08]), "quat": gripper_pick_quat_left.elements, "gripper": 1}, # approach the cube
            {"t": 170, "xyz": socket_xyz + np.array([0, 0, -0.03]), "quat": gripper_pick_quat_left.elements, "gripper": 1}, # go down
            {"t": 220, "xyz": socket_xyz + np.array([0, 0, -0.03]), "quat": gripper_pick_quat_left.elements, "gripper": 0}, # close gripper
            {"t": 285, "xyz": meet_xyz + np.array([-0.1, 0, 0]), "quat": gripper_pick_quat_left.elements, "gripper": 0}, # approach meet position
            {"t": 340, "xyz": meet_xyz + np.array([-0.05, 0, 0]), "quat": gripper_pick_quat_left.elements,"gripper": 0},  # insertion
            {"t": 400, "xyz": meet_xyz + np.array([-0.05, 0, 0]), "quat": gripper_pick_quat_left.elements, "gripper": 0},  # insertion
        ]

        self.right_trajectory = [
            {"t": 0, "xyz": init_mocap_pose_right[:3], "quat": init_mocap_pose_right[3:], "gripper": 0}, # sleep
            {"t": 120, "xyz": peg_xyz + np.array([0, 0, 0.08]), "quat": gripper_pick_quat_right.elements, "gripper": 1}, # approach the cube
            {"t": 170, "xyz": peg_xyz + np.array([0, 0, -0.03]), "quat": gripper_pick_quat_right.elements, "gripper": 1}, # go down
            {"t": 220, "xyz": peg_xyz + np.array([0, 0, -0.03]), "quat": gripper_pick_quat_right.elements, "gripper": 0}, # close gripper
            {"t": 285, "xyz": meet_xyz + np.array([0.1, 0, lift_right]), "quat": gripper_pick_quat_right.elements, "gripper": 0}, # approach meet position
            {"t": 340, "xyz": meet_xyz + np.array([0.05, 0, lift_right]), "quat": gripper_pick_quat_right.elements, "gripper": 0},  # insertion
            {"t": 400, "xyz": meet_xyz + np.array([0.05, 0, lift_right]), "quat": gripper_pick_quat_right.elements, "gripper": 0},  # insertion

        ]


def test_policy(task_name):
    # example rolling out pick_and_transfer policy
    onscreen_render = True
    inject_noise = False

    # setup the environment
    episode_len = SIM_TASK_CONFIGS[task_name]['episode_len']
    if 'sim_transfer_cube' in task_name:
        env = make_ee_sim_env('sim_transfer_cube')
    elif 'sim_insertion' in task_name:
        env = make_ee_sim_env('sim_insertion')
    else:
        raise NotImplementedError

    for episode_idx in range(2):
        ts = env.reset()
        episode = [ts]
        if onscreen_render:
            ax = plt.subplot()
            plt_img = ax.imshow(ts.observation['images']['angle'])
            plt.ion()

        policy = PickAndTransferPolicy(inject_noise)
        for step in range(episode_len):
            action = policy(ts)
            ts = env.step(action)
            episode.append(ts)
            if onscreen_render:
                plt_img.set_data(ts.observation['images']['angle'])
                plt.pause(0.02)
        plt.close()

        episode_return = np.sum([ts.reward for ts in episode[1:]])
        if episode_return > 0:
            print(f"{episode_idx} Successful, {episode_return}")
        else:
            print(f"{episode_idx} Failed")


if __name__ == '__main__':
    test_task_name = 'sim_transfer_cube_scripted'
    test_policy(test_task_name)

