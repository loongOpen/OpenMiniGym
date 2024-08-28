import numpy as np
import collections
import os
from ctypes import *

from constants import DT, XML_DIR, START_ARM_POSE,B_2_N,DOF_ARM,DOF_CAP
from constants import PUPPET_GRIPPER_POSITION_CLOSE,PUPPET_GRIPPER_POSITION_OPEN
from constants import PUPPET_GRIPPER_POSITION_UNNORMALIZE_FN
from constants import PUPPET_GRIPPER_POSITION_NORMALIZE_FN
from constants import PUPPET_GRIPPER_VELOCITY_NORMALIZE_FN

from utils import sample_box_pose, sample_insertion_pose, amao_sample_box_pose
from dm_control import mujoco
from dm_control.rl import control
from dm_control.suite import base
import math
import sys
# sys.path.append(r'/home/vipp/test/3-ALOHA/0_ALOHA_codes/aloha_act_plus/DrWang_IK_solver/pinocchio_-inv-kine/PythonVer')
sys.path.append(r'/home/tinymal/下载/imitate_robohome/software/DrSun_IK_solver/ttst')
#from MainTest import Dr_W_IK_solver
from DrSun_IK_solver.ttst.DrS_IK import DrS_IK_solver


import IPython
e = IPython.embed

# kp = [200, 250, 40, 40, 10, 10, 1,   200, 250, 40, 40, 10, 10, 1]
# kd = [2, 2.5, 0.4, 0.4, 0.1, 0.1, 0.01, 2, 2.5, 0.4, 0.4, 0.1, 0.1, 0.01]
# PUPPET_GRIPPER_POSITION_OPEN,-PUPPET_GRIPPER_POSITION_OPEN
#初始化角度
tar_q_init=START_ARM_POSE

tar_q=START_ARM_POSE

timer =0


def quaternion_to_euler(q):
    """
    Convert a quaternion into euler angles (yaw, pitch, roll)
    q: A list of four elements representing a quaternion [q_w, q_x, q_y, q_z]
    """
    # Extract the values from the quaternion
    w, x, y, z = q

    # Yaw (Z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # Pitch (Y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Roll (X-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    return pitch, roll, yaw 


def make_ee_sim_env(task_name):#仿真环境
    """
    Environment for simulated robot bi-manual manipulation, with end-effector control.
    Action space:      [left_arm_pose (7),             # position and quaternion for end effector
                        left_gripper_positions (1),    # normalized gripper position (0: close, 1: open)
                        right_arm_pose (7),            # position and quaternion for end effector
                        right_gripper_positions (1),]  # normalized gripper position (0: close, 1: open)

    Observation space: {"qpos": Concat[ left_arm_qpos (6),         # absolute joint position
                                        left_gripper_position (1),  # normalized gripper position (0: close, 1: open)
                                        right_arm_qpos (6),         # absolute joint position
                                        right_gripper_qpos (1)]     # normalized gripper position (0: close, 1: open)
                        "qvel": Concat[ left_arm_qvel (6),         # absolute joint velocity (rad)
                                        left_gripper_velocity (1),  # normalized gripper velocity (pos: opening, neg: closing)
                                        right_arm_qvel (6),         # absolute joint velocity (rad)
                                        right_gripper_qvel (1)]     # normalized gripper velocity (pos: opening, neg: closing)
                        "images": {"main": (480x640x3)}        # h, w, c, dtype='uint8'
    """
    if 'sim_transfer_cube' in task_name:
        xml_path = os.path.join(XML_DIR, f'bimanual_viperx_ee_transfer_cube.xml')#/home/tinymal/下载/act_plus2/assets
        physics = mujoco.Physics.from_xml_path(xml_path)
        task = TransferCubeEETask(random=False)
        env = control.Environment(physics, task, time_limit=20, control_timestep=DT,
                                  n_sub_steps=None, flat_observation=False)
    elif 'sim_insertion' in task_name:
        xml_path = os.path.join(XML_DIR, f'bimanual_viperx_ee_insertion.xml')
        physics = mujoco.Physics.from_xml_path(xml_path)
        task = InsertionEETask(random=False)
        env = control.Environment(physics, task, time_limit=20, control_timestep=DT,
                                  n_sub_steps=None, flat_observation=False)
    elif 'human_test' == task_name:#---------------------------------------------new doghome
        xml_path = os.path.join(XML_DIR, f'human_test.xml')
        physics = mujoco.Physics.from_xml_path(xml_path)
        task = TransferCubeEETask_Human(random=False)
        env = control.Environment(physics, task, time_limit=20, control_timestep=DT,
                                  n_sub_steps=None, flat_observation=False)
    elif 'human_test1' == task_name:#---------------------------------------------new doghome
        xml_path = os.path.join(XML_DIR, f'human_test1.xml')
        physics = mujoco.Physics.from_xml_path(xml_path)
        task = TransferCubeEETask_Human1(random=False)
        env = control.Environment(physics, task, time_limit=20, control_timestep=DT,
                                  n_sub_steps=None, flat_observation=False)        
    else:
        raise NotImplementedError
    return env

class BimanualViperXEETask(base.Task):
    def __init__(self, random=None):
        super().__init__(random=random)
        timer=0

    def before_step(self, action, physics):#输入末端位姿轨迹，输出关节角
        global timer
        a_len = len(action) // 2
        #世界坐标系下的末端位姿态
        action_left = action[:a_len]
        action_right = action[a_len:]
        end_pos1=action_left[0:3]
        end_pos2=action_right[0:3]
        #end_pos2=np.array([0.2,-0.15,0.1])
        #转换坐标系到机体
        end_pos1=end_pos1-B_2_N
        end_pos2=end_pos2-B_2_N

        #计算运动学
        g_left_ctrl = PUPPET_GRIPPER_POSITION_UNNORMALIZE_FN(action_left[7])#限制爪子幅度 归一化
        g_right_ctrl = PUPPET_GRIPPER_POSITION_UNNORMALIZE_FN(action_right[7])
        #----------------------------------
        pitch_l, roll_l, yaw_l = quaternion_to_euler(action_left[3:7])
        pitch_r, roll_r, yaw_r = quaternion_to_euler(action_right[3:7])
        #print("quaternion_to_euler",yaw_l, pitch_l, roll_l)
        #print(end_pos1,end_pos2)
        real_arrayleft =  (c_double * 7)(pitch_l+90, roll_l, yaw_l, end_pos1[2]*1000, end_pos1[0]*1000, end_pos1[1]*1000, 0.52333)
        real_arrayright = (c_double * 7)(pitch_r+90, -roll_r, -yaw_r, end_pos2[2]*1000, end_pos2[0]*1000,-end_pos2[1]*1000 ,0.52333)#not use now
        
        att=[90,0,0]#pitch roll yaw
        pos=[-0.13,0.15,0.17]#z x y 逆运动学顺序    
        if 0:#用一个已知动作测试逆解，没问题：
            timer=timer+0.05
            #pos[0]=-1.5+math.sin(timer)*0.5#上下  0点为肩部所以为负数
            #pos[1]=2.5+math.sin(timer)*0.5#前后   在机器人前方所以为+
            #pos[2]=1.5+math.sin(timer)*0.5#左右   0在机器人正中心 左右对称
            #print(pos)
            real_arrayleft = (c_double * 7)(att[0]/180*3.1415, att[1]/180*3.1415, att[2]/180*3.1415,  pos[0]*1000, pos[1]*1000, pos[2]*1000, 0.52333)#单位mm
            real_arrayright = (c_double * 7)(att[0]/180*3.1415, att[1]/180*3.1415, att[2]/180*3.1415, pos[0]*1000, pos[1]*1000, pos[2]*1000, 0.52333)
        #real_arrayleft = (c_double * 7)(att[0]/180*3.1415, att[1]/180*3.1415, att[2]/180*3.1415,  pos[0]*1000, pos[1]*1000, pos[2]*1000, 0.52333)#单位mm
        #real_arrayright = (c_double * 7)(att[0]/180*3.1415, att[1]/180*3.1415, att[2]/180*3.1415, pos[0]*1000, pos[1]*1000, pos[2]*1000, 0.52333)
        q_tar = DrS_IK_solver(real_arrayleft, real_arrayright) #孙博
 
        #q_tar = tar_q_init
        #算出来14个关节角，Left: 0:7 +抓1+抓2; right：[9:15]+抓1+抓2  18个自由度
        new_qpos = np.concatenate([q_tar[0:DOF_ARM],[0,0],q_tar[DOF_ARM:DOF_ARM*2],[0,0]]) #18
        # new_qpos = np.concatenate([q_tar[0:1],np.zeros(17)])
        #np.copyto(physics.data.qpos[:18], new_qpos)
        
        qpos_raw = physics.data.qpos.copy()[:(DOF_ARM+2)*2]
        #print("amao_qpos=",qpos_raw,len(qpos_raw))

        tar_q_flt=np.zeros((DOF_ARM+2)*2)
        flt=1
        for i in range (0,len(qpos_raw)):
            tar_q_flt[i]=new_qpos[i]*flt+(1-flt)*qpos_raw[i]
        
       
        #print("qpos_raw=",qpos_raw," len=",len(qpos_raw))
        #control_signal = np.clip(kp * (tar_q - qpos_raw) - kd * (qvel_raw)  , -target_joint_u_d, target_joint_u_d)
        tar_q_flt[DOF_ARM]=g_left_ctrl
        tar_q_flt[DOF_ARM+1]=-g_left_ctrl#PUPPET_GRIPPER_POSITION_OPEN

        tar_q_flt[(DOF_ARM+1)*2]=g_right_ctrl
        tar_q_flt[(DOF_ARM+1)*2+1]=-g_right_ctrl 
        
        np.copyto(physics.data.ctrl, np.concatenate([tar_q_flt,np.zeros((DOF_ARM+2)*2)]))#输出位置控制器
        #print(len(physics.data.ctrl,))
 
    def initialize_robots(self, physics):
        # reset joint position

        physics.named.data.qpos[:(DOF_ARM+2)*2] = tar_q_init#START_ARM_POSE #amao
        print(tar_q_init)

        # reset mocap to align with end effector
        # to obtain these numbers:
        # (1) make an ee_sim env and reset to the same start_pose
        # (2) get env._physics.named.data.xpos['vx300s_left/gripper_link']
        #     get env._physics.named.data.xquat['vx300s_left/gripper_link']
        #     repeat the same for right side  mocap传感器可以直接获取物体位姿 并进行IK解算
        
        # np.copyto(physics.data.mocap_pos[0], [-0.31718881+0.1, 0.5, 0.29525084])
        # np.copyto(physics.data.mocap_quat[0], [1, 0, 0, 0])
        # # right
        # np.copyto(physics.data.mocap_pos[1], np.array([0.31718881-0.1, 0.49999888, 0.29525084]))
        # np.copyto(physics.data.mocap_quat[1],  [1, 0, 0, 0])

        # reset gripper control
        close_gripper_control = np.array([
            PUPPET_GRIPPER_POSITION_CLOSE,
            -PUPPET_GRIPPER_POSITION_CLOSE,
            PUPPET_GRIPPER_POSITION_CLOSE,
            -PUPPET_GRIPPER_POSITION_CLOSE,
        ])
        np.copyto(physics.data.ctrl, np.concatenate([tar_q_init,np.zeros((DOF_ARM+2)*2)])) #amao
        #世龙有18个ctrl，邢博有36多个(18pos+18vel)，原本有只有四个夹爪   contrl=<actuator>
        # np.copyto(physics.data.ctrl, close_gripper_control)

    def initialize_episode(self, physics):
        """Sets the state of the environment at the start of each episode."""
        super().initialize_episode(physics)

    @staticmethod
    def get_qpos(physics):#14+2=16 (末端位置+姿态)*2+2归一化行程  用于输出 7dof关节
        qpos_raw = physics.data.qpos.copy()
        left_qpos_raw = qpos_raw[:(DOF_ARM+2)]#7+2
        right_qpos_raw = qpos_raw[(DOF_ARM+2):(DOF_ARM+2)*2]
        left_arm_qpos = left_qpos_raw[:DOF_ARM]
        right_arm_qpos = right_qpos_raw[:DOF_ARM]
        left_gripper_qpos = [PUPPET_GRIPPER_POSITION_NORMALIZE_FN(left_qpos_raw[DOF_ARM])]#夹具2dof合并为一个行程输出
        right_gripper_qpos = [PUPPET_GRIPPER_POSITION_NORMALIZE_FN(right_qpos_raw[DOF_ARM])]
        return np.concatenate([left_arm_qpos, left_gripper_qpos, right_arm_qpos, right_gripper_qpos])

    @staticmethod
    def get_qvel(physics):
        qvel_raw = physics.data.qvel.copy()
        left_qvel_raw = qvel_raw[:(DOF_ARM+2)]
        right_qvel_raw = qvel_raw[(DOF_ARM+2):(DOF_ARM+2)*2]
        left_arm_qvel = left_qvel_raw[:DOF_ARM]
        right_arm_qvel = right_qvel_raw[:DOF_ARM]
        left_gripper_qvel = [PUPPET_GRIPPER_VELOCITY_NORMALIZE_FN(left_qvel_raw[DOF_ARM])]
        right_gripper_qvel = [PUPPET_GRIPPER_VELOCITY_NORMALIZE_FN(right_qvel_raw[DOF_ARM])]
        return np.concatenate([left_arm_qvel, left_gripper_qvel, right_arm_qvel, right_gripper_qvel])

    @staticmethod
    def get_env_state(physics):
        raise NotImplementedError

    def get_observation(self, physics):
        # note: it is important to do .copy()
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos(physics)# d=16
        obs['qvel'] = self.get_qvel(physics) #16
        obs['env_state'] = self.get_env_state(physics) # d=7 box位姿
        #print("qpos=",len( obs['qpos']), "env=",len( obs['env_state']))
        obs['images'] = dict()
        obs['images']['top'] = physics.render(height=480, width=640, camera_id='top')#图像尺寸
        obs['images']['angle'] = physics.render(height=480, width=640, camera_id='angle')
        obs['images']['vis'] = physics.render(height=480, width=640, camera_id='front_close')

        obs['images']['left_wrist'] = physics.render(height=480, width=640, camera_id='left_wrist')
        obs['images']['right_wrist'] = physics.render(height=480, width=640, camera_id='right_wrist')

        # used in scripted policy to obtain starting pose
        obs['mocap_pose_left'] = [0,0,0,0,0,0,0]
        obs['mocap_pose_right'] =  [0,0,0,0,0,0,0]
        # obs['mocap_pose_left']=[-0.21718881 , 0.5     ,    0.29525084  ,1.       ,   0.     ,     0. ,0.      ]  
        # obs['mocap_pose_right'] =[0.21718881, 0.49999888 ,0.29525084 ,1.   ,      0.   ,      0.,0. ]
        # obs['mocap_pose_left'] = np.concatenate([physics.data.mocap_pos[0], physics.data.mocap_quat[0]]).copy()
        # obs['mocap_pose_right'] =  np.concatenate([physics.data.mocap_pos[1], physics.data.mocap_quat[1]]).copy()

        # used when replaying joint trajectory
        obs['gripper_ctrl'] = physics.data.ctrl.copy()
        return obs

    def get_reward(self, physics):
        raise NotImplementedError


class TransferCubeEETask_Human(BimanualViperXEETask):#-----------------doghome
    def __init__(self, random=None):
        super().__init__(random=random)
        self.max_reward = 5

    def initialize_episode(self, physics):
        """Sets the state of the environment at the start of each episode."""
        self.initialize_robots(physics)
        box_start_idx = physics.model.name2id('red_box_joint', 'joint') 
        # a=physics.data.qpos[box_start_idx+7:box_start_idx+10]
        cube_pose = amao_sample_box_pose(physics.data.qpos[box_start_idx:box_start_idx+3]) #red
        cube_pose1 = amao_sample_box_pose(physics.data.qpos[box_start_idx+7:box_start_idx+10]) #blue
        cube_pose2 = amao_sample_box_pose(physics.data.qpos[box_start_idx+14:box_start_idx+17]) #green
        
        np.copyto(physics.data.qpos[box_start_idx : box_start_idx + 7], cube_pose) #随机红块的初始位置
        np.copyto(physics.data.qpos[box_start_idx+7 : box_start_idx + 14], cube_pose1)#随机蓝球的初始位置
        np.copyto(physics.data.qpos[box_start_idx+14 : box_start_idx + 21], cube_pose2)#随机绿球的初始位置

        super().initialize_episode(physics)

    @staticmethod
    def get_env_state(physics):
        env_state = physics.data.qpos.copy()[(DOF_ARM+2)*2:]
        return env_state

    def get_reward(self, physics):
        # return whether left gripper is holding the box
        all_contact_pairs = []
        for i_contact in range(physics.data.ncon):
            id_geom_1 = physics.data.contact[i_contact].geom1
            id_geom_2 = physics.data.contact[i_contact].geom2
            name_geom_1 = physics.model.id2name(id_geom_1, 'geom')
            name_geom_2 = physics.model.id2name(id_geom_2, 'geom')
            contact_pair = (name_geom_1, name_geom_2)
            all_contact_pairs.append(contact_pair)

        touch_left_gripper = ("red_box", "vx300s_left/10_left_gripper_finger") in all_contact_pairs #左爪夹住
        touch_right_gripper = ("red_box", "vx300s_right/10_right_gripper_finger") in all_contact_pairs#又抓夹住
        touch_table = ("red_box", "table") in all_contact_pairs
        cube_red_place_good = ("red_box", "table") in all_contact_pairs
        cube_green_place_good = ("green_box", "table") in all_contact_pairs
        cube_red_on_blue = ("red_box", "blue_box") in all_contact_pairs
        # print("Reward check",touch_left_gripper,touch_right_gripper,touch_table,cube_red_place_good)
        #print(cube_red_place_good,cube_blue_place_good)
        reward = 0
        if touch_right_gripper:
            reward = 1
        if touch_right_gripper and not touch_table: # lifted
            reward = 2
        if touch_left_gripper: # attempted transfer
            reward = 3
        if touch_left_gripper and not touch_table: # successful transfer
            reward = 4
        if cube_red_on_blue and cube_green_place_good: # successful transfer
            reward = 5
       
        return reward

class TransferCubeEETask_Human1(BimanualViperXEETask):#-----------------doghome slope
    def __init__(self, random=None):
        super().__init__(random=random)
        self.max_reward = 5

    def initialize_episode(self, physics):
        """Sets the state of the environment at the start of each episode."""
        self.initialize_robots(physics)
        box_start_idx = physics.model.name2id('red_box_joint', 'joint') 
        # a=physics.data.qpos[box_start_idx+7:box_start_idx+10]
        cube_pose = amao_sample_box_pose(physics.data.qpos[box_start_idx:box_start_idx+3]) #red
        cube_pose1 = amao_sample_box_pose(physics.data.qpos[box_start_idx+7:box_start_idx+10]) #blue
        
        np.copyto(physics.data.qpos[box_start_idx : box_start_idx + 7], cube_pose) #随机红块的初始位置
        np.copyto(physics.data.qpos[box_start_idx+7 : box_start_idx + 14], cube_pose1)#随机篮球的初始位置

        box_start_idx = physics.model.name2id('ball_joint1', 'joint') 
        print(box_start_idx)
        super().initialize_episode(physics)

    @staticmethod
    def get_env_state(physics):
        env_state = physics.data.qpos.copy()[(DOF_ARM+2)*2:]
        return env_state

    def get_reward(self, physics):
        # return whether left gripper is holding the box
        all_contact_pairs = []
        for i_contact in range(physics.data.ncon):
            id_geom_1 = physics.data.contact[i_contact].geom1
            id_geom_2 = physics.data.contact[i_contact].geom2
            name_geom_1 = physics.model.id2name(id_geom_1, 'geom')
            name_geom_2 = physics.model.id2name(id_geom_2, 'geom')
            contact_pair = (name_geom_1, name_geom_2)
            all_contact_pairs.append(contact_pair)

        touch_left_gripper = ("red_box", "vx300s_left/10_left_gripper_finger") in all_contact_pairs #左爪夹住
        touch_right_gripper = ("red_box", "vx300s_right/10_right_gripper_finger") in all_contact_pairs#又抓夹住
        touch_table = ("red_box", "table") in all_contact_pairs
        cube_red_place_good = ("red_box", "table") in all_contact_pairs
        cube_blue_place_good = ("blue_box", "table") in all_contact_pairs
        # print("Reward check",touch_left_gripper,touch_right_gripper,touch_table,cube_red_place_good)
        #print(cube_red_place_good,cube_blue_place_good)
        reward = 0
        if touch_right_gripper:
            reward = 1
        if touch_right_gripper and not touch_table: # lifted
            reward = 2
        if touch_left_gripper: # attempted transfer
            reward = 3
        if touch_left_gripper and not touch_table: # successful transfer
            reward = 4
        if cube_red_place_good and cube_blue_place_good: # successful transfer
            reward = 5
        return reward


class TransferCubeEETask(BimanualViperXEETask):
    def __init__(self, random=None):
        super().__init__(random=random)
        self.max_reward = 4

    def initialize_episode(self, physics):
        """Sets the state of the environment at the start of each episode."""
        self.initialize_robots(physics)
        # randomize box position
        cube_pose = sample_box_pose()
        box_start_idx = physics.model.name2id('red_box_joint', 'joint')
        np.copyto(physics.data.qpos[box_start_idx : box_start_idx + 7], cube_pose)
        # print(f"randomized cube position to {cube_position}")

        super().initialize_episode(physics)

    @staticmethod
    def get_env_state(physics):
        env_state = physics.data.qpos.copy()[16:]
        return env_state

    def get_reward(self, physics):
        # return whether left gripper is holding the box
        all_contact_pairs = []
        for i_contact in range(physics.data.ncon):
            id_geom_1 = physics.data.contact[i_contact].geom1
            id_geom_2 = physics.data.contact[i_contact].geom2
            name_geom_1 = physics.model.id2name(id_geom_1, 'geom')
            name_geom_2 = physics.model.id2name(id_geom_2, 'geom')
            contact_pair = (name_geom_1, name_geom_2)
            all_contact_pairs.append(contact_pair)

        touch_left_gripper = ("red_box", "vx300s_left/10_left_gripper_finger") in all_contact_pairs
        touch_right_gripper = ("red_box", "vx300s_right/10_right_gripper_finger") in all_contact_pairs
        touch_table = ("red_box", "table") in all_contact_pairs

        reward = 0
        if touch_right_gripper:
            reward = 1
        if touch_right_gripper and not touch_table: # lifted
            reward = 2
        if touch_left_gripper: # attempted transfer
            reward = 3
        if touch_left_gripper and not touch_table: # successful transfer
            reward = 4
        return reward


class InsertionEETask(BimanualViperXEETask):
    def __init__(self, random=None):
        super().__init__(random=random)
        self.max_reward = 4

    def initialize_episode(self, physics):
        """Sets the state of the environment at the start of each episode."""
        self.initialize_robots(physics)
        # randomize peg and socket position
        peg_pose, socket_pose = sample_insertion_pose()
        id2index = lambda j_id: 16 + (j_id - 16) * 7 # first 16 is robot qpos, 7 is pose dim # hacky

        peg_start_id = physics.model.name2id('red_peg_joint', 'joint')
        peg_start_idx = id2index(peg_start_id)
        np.copyto(physics.data.qpos[peg_start_idx : peg_start_idx + 7], peg_pose)
        # print(f"randomized cube position to {cube_position}")

        socket_start_id = physics.model.name2id('blue_socket_joint', 'joint')
        socket_start_idx = id2index(socket_start_id)
        np.copyto(physics.data.qpos[socket_start_idx : socket_start_idx + 7], socket_pose)
        # print(f"randomized cube position to {cube_position}")

        super().initialize_episode(physics)

    @staticmethod
    def get_env_state(physics):
        env_state = physics.data.qpos.copy()[18:]
        return env_state

    def get_reward(self, physics):
        # return whether peg touches the pin
        all_contact_pairs = []
        for i_contact in range(physics.data.ncon):
            id_geom_1 = physics.data.contact[i_contact].geom1
            id_geom_2 = physics.data.contact[i_contact].geom2
            name_geom_1 = physics.model.id2name(id_geom_1, 'geom')
            name_geom_2 = physics.model.id2name(id_geom_2, 'geom')
            contact_pair = (name_geom_1, name_geom_2)
            all_contact_pairs.append(contact_pair)

        touch_right_gripper = ("red_peg", "vx300s_right/10_right_gripper_finger") in all_contact_pairs
        touch_left_gripper = ("socket-1", "vx300s_left/10_left_gripper_finger") in all_contact_pairs or \
                             ("socket-2", "vx300s_left/10_left_gripper_finger") in all_contact_pairs or \
                             ("socket-3", "vx300s_left/10_left_gripper_finger") in all_contact_pairs or \
                             ("socket-4", "vx300s_left/10_left_gripper_finger") in all_contact_pairs

        peg_touch_table = ("red_peg", "table") in all_contact_pairs
        socket_touch_table = ("socket-1", "table") in all_contact_pairs or \
                             ("socket-2", "table") in all_contact_pairs or \
                             ("socket-3", "table") in all_contact_pairs or \
                             ("socket-4", "table") in all_contact_pairs
        peg_touch_socket = ("red_peg", "socket-1") in all_contact_pairs or \
                           ("red_peg", "socket-2") in all_contact_pairs or \
                           ("red_peg", "socket-3") in all_contact_pairs or \
                           ("red_peg", "socket-4") in all_contact_pairs
        pin_touched = ("red_peg", "pin") in all_contact_pairs

        reward = 0
        if touch_left_gripper and touch_right_gripper: # touch both
            reward = 1
        if touch_left_gripper and touch_right_gripper and (not peg_touch_table) and (not socket_touch_table): # grasp both
            reward = 2
        if peg_touch_socket and (not peg_touch_table) and (not socket_touch_table): # peg and socket touching
            reward = 3
        if pin_touched: # successful insertion
            reward = 4
        return reward
