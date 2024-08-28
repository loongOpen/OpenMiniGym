import time
import numpy as np
import collections
import matplotlib.pyplot as plt
import dm_env
from pyquaternion import Quaternion
from constants_real import DT, START_ARM_POSE, MASTER_GRIPPER_JOINT_NORMALIZE_FN, PUPPET_GRIPPER_JOINT_UNNORMALIZE_FN
from constants_real import PUPPET_GRIPPER_POSITION_NORMALIZE_FN, PUPPET_GRIPPER_VELOCITY_NORMALIZE_FN
from constants_real import PUPPET_GRIPPER_JOINT_OPEN, PUPPET_GRIPPER_JOINT_CLOSE,DOF_ARM,DOF_CAP
from robot_utils import Recorder, ImageRecorder
from robot_utils import setup_master_bot, setup_puppet_bot, move_arms, move_grippers

import IPython
e = IPython.embed

class RealEnv:
    """
    Environment for real robot bi-manual manipulation
    Action space:      [left_arm_qpos (6),             # absolute joint position
                        left_gripper_positions (1),    # normalized gripper position (0: close, 1: open)
                        right_arm_qpos (6),            # absolute joint position
                        right_gripper_positions (1),]  # normalized gripper position (0: close, 1: open)

    Observation space: {"qpos": Concat[ left_arm_qpos (6),          # absolute joint position
                                        left_gripper_position (1),  # normalized gripper position (0: close, 1: open)
                                        right_arm_qpos (6),         # absolute joint position
                                        right_gripper_qpos (1)]     # normalized gripper position (0: close, 1: open)
                        "qvel": Concat[ left_arm_qvel (6),         # absolute joint velocity (rad)
                                        left_gripper_velocity (1),  # normalized gripper velocity (pos: opening, neg: closing)
                                        right_arm_qvel (6),         # absolute joint velocity (rad)
                                        right_gripper_qvel (1)]     # normalized gripper velocity (pos: opening, neg: closing)
                        "images": {"cam_high": (480x640x3),        # h, w, c, dtype='uint8'
                                   "cam_low": (480x640x3),         # h, w, c, dtype='uint8'
                                   "cam_left_wrist": (480x640x3),  # h, w, c, dtype='uint8'
                                   "cam_right_wrist": (480x640x3)} # h, w, c, dtype='uint8'
    """

    def __init__(self, init_node, setup_robots=True, setup_base=False):
       
        if setup_robots:
            self.setup_robots()
        
        if setup_base:
            self.setup_base()
        
        # self.setup_t265()
        #self.setup_dxl()

        self.recorder_left = Recorder('left', init_node=True)
        self.recorder_right = Recorder('right', init_node=False)
        self.recorder_base = Recorder('base', init_node=False)

        self.recorder_left_exp = Recorder('left_exp', init_node=False)
        self.recorder_right_exp = Recorder('right_exp', init_node=False)

        self.image_recorder = ImageRecorder(init_node=False)
      
    
    def setup_t265(self):
        i=0
        # self.pipeline = rs.pipeline()
        # cfg = rs.config()
        # # if only pose stream is enabled, fps is higher (202 vs 30)
        # cfg.enable_stream(rs.stream.pose)
        # self.pipeline.start(cfg)
    
    def setup_dxl(self):
        i=0
        # self.dxl_client = DynamixelClient([1, 2], port='/dev/ttyDXL_wheels', lazy_connect=True)
        # self.wheel_r = 0.101 / 2  # 101 mm is the diameter
        # self.base_r = 0.622  # 622 mm is the distance between the two wheels
    
    def setup_base(self):#复位底盘
        i=0
        # self.tracer = pyagxrobots.pysdkugv.TracerBase()
        # self.tracer.EnableCAN()

    def setup_robots(self):#复位机械臂
        i=0
        #setup_puppet_bot(self.puppet_bot_left)
        #setup_puppet_bot(self.puppet_bot_right)

    def get_qpos(self):
        left_qpos_raw = self.recorder_left.qpos
        #print(len(left_qpos_raw),left_qpos_raw)
        right_qpos_raw = self.recorder_right.qpos

        left_arm_qpos = left_qpos_raw[:DOF_ARM]#6dof +1cap
        #print(left_arm_qpos)
        right_arm_qpos = right_qpos_raw[:DOF_ARM]
        
        left_gripper_qpos = [left_qpos_raw[DOF_ARM+DOF_CAP-1]]#[PUPPET_GRIPPER_POSITION_NORMALIZE_FN(left_qpos_raw[7])] # this is position not joint
        right_gripper_qpos = [right_qpos_raw[DOF_ARM+DOF_CAP-1]]#[PUPPET_GRIPPER_POSITION_NORMALIZE_FN(right_qpos_raw[7])] # this is position not joint
        #print(len(left_qpos_raw),len(left_arm_qpos),len(left_gripper_qpos))
        return np.concatenate([left_arm_qpos, left_gripper_qpos, right_arm_qpos, right_gripper_qpos])

    def get_qvel(self):
        left_qvel_raw = self.recorder_left.qvel
        right_qvel_raw = self.recorder_right.qvel
        left_arm_qvel = left_qvel_raw[:DOF_ARM]
        right_arm_qvel = right_qvel_raw[:DOF_ARM]
        left_gripper_qvel = [left_qvel_raw[DOF_ARM+DOF_CAP-1]]#[PUPPET_GRIPPER_VELOCITY_NORMALIZE_FN(left_qvel_raw[7])]
        right_gripper_qvel = [left_qvel_raw[DOF_ARM+DOF_CAP-1]]#[PUPPET_GRIPPER_VELOCITY_NORMALIZE_FN(right_qvel_raw[7])]
        return np.concatenate([left_arm_qvel, left_gripper_qvel, right_arm_qvel, right_gripper_qvel])

    def get_effort(self):
        left_effort_raw = self.recorder_left.effort
        right_effort_raw = self.recorder_right.effort
        left_robot_effort = left_effort_raw[:DOF_ARM+DOF_CAP]
        right_robot_effort = right_effort_raw[:DOF_ARM+DOF_CAP]
        return np.concatenate([left_robot_effort, right_robot_effort])

    def get_images(self):
        return self.image_recorder.get_images()

    def get_base_vel_t265(self):
        i=0

    def get_base_vel(self):
        # left_vel, right_vel = self.dxl_client.read_pos_vel_cur()[1]
        # right_vel = -right_vel # right wheel is inverted
        # base_linear_vel = (left_vel + right_vel) * self.wheel_r / 2
        # base_angular_vel = (right_vel - left_vel) * self.wheel_r / self.base_r
        temp = self.recorder_base.qvel
        base_linear_vel  =temp[0]
        base_angular_vel =temp[1]
        #print(base_linear_vel,base_angular_vel)
        return np.array([base_linear_vel, base_angular_vel])

    def get_tracer_vel(self):
        #linear_vel, angular_vel = self.tracer.GetLinearVelocity(), self.tracer.GetAngularVelocity()
        temp = self.recorder_base.qvel
        linear_vel  =temp[0]
        angular_vel =temp[1]
        return np.array([linear_vel, angular_vel])


    def set_gripper_pose(self, left_gripper_desired_pos_normalized, right_gripper_desired_pos_normalized):
        left_gripper_desired_joint = PUPPET_GRIPPER_JOINT_UNNORMALIZE_FN(left_gripper_desired_pos_normalized)
        self.gripper_command.cmd = left_gripper_desired_joint
        self.puppet_bot_left.gripper.core.pub_single.publish(self.gripper_command)

        right_gripper_desired_joint = PUPPET_GRIPPER_JOINT_UNNORMALIZE_FN(right_gripper_desired_pos_normalized)
        self.gripper_command.cmd = right_gripper_desired_joint
        self.puppet_bot_right.gripper.core.pub_single.publish(self.gripper_command)

    def _reset_joints(self):
        reset_position = START_ARM_POSE[:DOF_ARM]
        move_arms([self.puppet_bot_left, self.puppet_bot_right], [reset_position, reset_position], move_time=1)

    def _reset_gripper(self):
        """Set to position mode and do position resets: first open then close. Then change back to PWM mode"""
        move_grippers([self.puppet_bot_left, self.puppet_bot_right], [PUPPET_GRIPPER_JOINT_OPEN] * 2, move_time=0.5)
        move_grippers([self.puppet_bot_left, self.puppet_bot_right], [PUPPET_GRIPPER_JOINT_CLOSE] * 2, move_time=1)

    def get_observation(self, get_tracer_vel=False):
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos()
        #print(len(obs['qpos']))
        obs['qvel'] = self.get_qvel()
        obs['effort'] = self.get_effort()
        obs['images'] = self.get_images()
        # obs['base_vel_t265'] = self.get_base_vel_t265()
        obs['base_vel'] = self.get_base_vel()
        if get_tracer_vel:
            obs['tracer_vel'] = self.get_tracer_vel()
        return obs

    def get_reward(self):
        return 0

    def reset(self, fake=False):
        if not fake:
            # Reboot puppet robot gripper motors
            i=0#复位
            # self.puppet_bot_left.dxl.robot_reboot_motors("single", "gripper", True)
            # self.puppet_bot_right.dxl.robot_reboot_motors("single", "gripper", True)
            # self._reset_joints()
            # self._reset_gripper()
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation())

    def step(self, action, base_action=None, get_tracer_vel=False, get_obs=True):
        state_len = int(len(action) / 2)
        left_action = action[:state_len]
        right_action = action[state_len:]
        #发送从机械臂和底盘质量


        # self.puppet_bot_left.arm.set_joint_positions(left_action[:6], blocking=False)
        # self.puppet_bot_right.arm.set_joint_positions(right_action[:6], blocking=False)
        # self.set_gripper_pose(left_action[-1], right_action[-1])
        # if base_action is not None:
        #     # linear_vel_limit = 1.5
        #     # angular_vel_limit = 1.5
        #     # base_action_linear = np.clip(base_action[0], -linear_vel_limit, linear_vel_limit)
        #     # base_action_angular = np.clip(base_action[1], -angular_vel_limit, angular_vel_limit)
        #     base_action_linear, base_action_angular = base_action#底盘
        #     self.tracer.SetMotionCommand(linear_vel=base_action_linear, angular_vel=base_action_angular)
        time.sleep(DT)
        if get_obs:
            obs = self.get_observation(get_tracer_vel)
        else:
            obs = None
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=obs)

    def get_action(self,use_fb=False):
        action = np.zeros(DOF_ARM*2+DOF_CAP*2) # 6 joint + 1 gripper, for two arms  14
        # Arm actions
        # action[:6] = master_bot_left.dxl.joint_states.position[:6]
        # action[7:7+6] = master_bot_right.dxl.joint_states.position[:6]
        # # Gripper actions
        # action[6] = MASTER_GRIPPER_JOINT_NORMALIZE_FN(master_bot_left.dxl.joint_states.position[6])
        # action[7+6] = MASTER_GRIPPER_JOINT_NORMALIZE_FN(master_bot_right.dxl.joint_states.position[6])
        if use_fb==True:
            left_qpos_raw = self.recorder_left.qpos
            right_qpos_raw = self.recorder_right.qpos
        else:
            left_qpos_raw = self.recorder_left_exp.qpos
            right_qpos_raw = self.recorder_right_exp.qpos
        action[:DOF_ARM]  = left_qpos_raw[:DOF_ARM]#6dof +1cap
        action[DOF_ARM+DOF_CAP:DOF_ARM+DOF_CAP+DOF_ARM] = right_qpos_raw[:DOF_ARM]
        action[DOF_ARM] = left_qpos_raw[DOF_ARM]#[PUPPET_GRIPPER_POSITION_NORMALIZE_FN(left_qpos_raw[7])] # this is position not joint
        action[DOF_ARM+DOF_CAP+DOF_ARM] = right_qpos_raw[DOF_ARM]#[PUPPET         
        #print(len(action),action)   
        return action

def get_action(use_fb=False):#unuse
    action = np.zeros(14) # 6 joint + 1 gripper, for two arms

    return action

# def get_base_action():



def make_real_env(init_node, setup_robots=True, setup_base=False):
    env = RealEnv(init_node, setup_robots, setup_base)
    return env


def test_real_teleop():
    """
    Test bimanual teleoperation and show image observations onscreen.
    It first reads joint poses from both master arms.
    Then use it as actions to step the environment.
    The environment returns full observations including images.

    An alternative approach is to have separate scripts for teleoperation and observation recording.
    This script will result in higher fidelity (obs, action) pairs
    """

    onscreen_render = True
    render_cam = 'cam_left_wrist'

    # # source of data
    # master_bot_left = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper",
    #                                           robot_name=f'master_left', init_node=True)
    # master_bot_right = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper",
    #                                            robot_name=f'master_right', init_node=False)
    # setup_master_bot(master_bot_left)
    #setup_master_bot(master_bot_right)

    # setup the environment
    env = make_real_env(init_node=False)
    ts = env.reset(fake=True)
    episode = [ts]
    # setup visualization
    if onscreen_render:
        ax = plt.subplot()
        plt_img = ax.imshow(ts.observation['images'][render_cam])
        plt.ion()

    for t in range(1000):
        #action = get_action(master_bot_left, master_bot_right)
        ts = env.step(action)
        episode.append(ts)

        if onscreen_render:
            plt_img.set_data(ts.observation['images'][render_cam])
            plt.pause(DT)
        else:
            time.sleep(DT)


if __name__ == '__main__':
    test_real_teleop()

