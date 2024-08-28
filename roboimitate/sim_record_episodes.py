import time
import os
import numpy as np
import argparse
import matplotlib.pyplot as plt
import h5py

from constants import PUPPET_GRIPPER_POSITION_NORMALIZE_FN, SIM_TASK_CONFIGS,DOF_ARM,DOF_CAP
from ee_sim_env import make_ee_sim_env
from sim_env import make_sim_env, BOX_POSE
from scripted_policy import PickAndTransferPolicy, InsertionPolicy,PickAndTransferPolicy_human,PickAndTransferPolicy_human1

import IPython
e = IPython.embed


def main(args):
    """
    Generate demonstration data in simulation.
    First rollout the policy (defined in ee space) in ee_sim_env. Obtain the joint trajectory.
    Replace the gripper joint positions with the commanded joint position.
    Replay this joint trajectory (as action sequence) in sim_env, and record all observations.
    Save this episode of data, and continue to next episode of data collection.
    """

    task_name = args['task_name']
    dataset_dir = args['dataset_dir']
    num_episodes = args['num_episodes']
    onscreen_render = args['onscreen_render']
    inject_noise = False
    render_cam_name1 = 'angle'
    render_cam_name2 = 'top'
    render_cam_name3 = 'vis'
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir, exist_ok=True)

    episode_len = SIM_TASK_CONFIGS[task_name]['episode_len']
    camera_names = SIM_TASK_CONFIGS[task_name]['camera_names']
    print(task_name)
    if task_name == 'sim_transfer_cube_scripted':#选择离线抓取策略，人工教学 /home/tinymal/下载/act_plus2/scripted_policy.py
        policy_cls = PickAndTransferPolicy
    elif task_name == 'sim_insertion_scripted':
        policy_cls = InsertionPolicy
    elif task_name == 'sim_transfer_cube_scripted_mirror':
        policy_cls = PickAndTransferPolicy
    elif task_name == 'human_test':#-------------------lyx
        policy_cls = PickAndTransferPolicy_human
    elif task_name == 'human_test1':
        policy_cls = PickAndTransferPolicy_human1       
    else:
        raise NotImplementedError

#模拟主手（采用末端位姿驱动机械臂）-----------------------------------------------------------------
    success = []
    save_idx=0
    for episode_idx in range(num_episodes):
        stu_flag=0
        teacher_flag=0
        print(f'{episode_idx}')
        print('Rollout out EE space scripted policy',episode_len)
        # setup the environment
        env = make_ee_sim_env(task_name)#初始化仿真 /episode_idx=home/tinymal/下载/act_plus2/ee_sim_env.py
        ts = env.reset()
        episode = [ts]
        policy = policy_cls(inject_noise)
        plt.figure(figsize=(22,10))
        # setup plotting 就第一次绘制
        if onscreen_render:
            ax1 = plt.subplot(2,3,1)
            plt_img1 = ax1.imshow(ts.observation['images'][render_cam_name1])
            ax2 = plt.subplot(2,3,2)
            plt_img2= ax2.imshow(ts.observation['images'][render_cam_name2])
            ax3 = plt.subplot(2,3,3)
            plt_img3= ax3.imshow(ts.observation['images'][render_cam_name3])
            ax4 = plt.subplot(2,3,4)
            plt_img4= ax4.imshow(ts.observation['images']['left_wrist'])
            ax5 = plt.subplot(2,3,6)
            plt_img5= ax5.imshow(ts.observation['images']['right_wrist'])
            plt.ion()
        for step in range(episode_len):
            action = policy(ts)#输出顺序：【left_xyz, left_quat, left_gripper  right_xyz, right_quat, right_gripper】
            #print('----action---------')
            #print(action)
            #UDP 采用UDP 动态手套数据代替离线轨迹
            
            #print("action episode_len=",action,"len=",len(action))
            ts = env.step(action)#驱动机械臂
            episode.append(ts)
            if onscreen_render:
                plt_img1.set_data(ts.observation['images'][render_cam_name1])
                plt_img2.set_data(ts.observation['images'][render_cam_name2])
                plt_img3.set_data(ts.observation['images'][render_cam_name3])
                plt_img4.set_data(ts.observation['images']['left_wrist'])
                plt_img5.set_data(ts.observation['images']['right_wrist'])
                plt.pause(0.002)
        plt.close()
        #while(1):
        #    print("finish")
        episode_return = np.sum([ts.reward for ts in episode[1:]])
        episode_max_reward = np.max([ts.reward for ts in episode[1:]])
        if episode_max_reward == env.task.max_reward:
            stu_flag=1
            print(f"{episode_idx} Successful-从臂, {episode_return}")
        else:
            print(f"{episode_idx} Failed-从臂")

        joint_traj = [ts.observation['qpos'] for ts in episode]#反馈角度 会随着时间长度增加 qpos=16 7+1+7+1
        # replace gripper pose with gripper control
        gripper_ctrl_traj = [ts.observation['gripper_ctrl'] for ts in episode]#控制角度 会随着时间长度增加  7+7+1+1
        #unuse?
        # for joint, ctrl in zip(joint_traj, gripper_ctrl_traj):
        #     left_ctrl = PUPPET_GRIPPER_POSITION_NORMALIZE_FN(ctrl[0])
        #     right_ctrl = PUPPET_GRIPPER_POSITION_NORMALIZE_FN(ctrl[2])
        #     joint[6] = left_ctrl
        #     joint[6+7] = right_ctrl

        subtask_info = episode[0].observation['env_state'].copy() # box pose at step 0  env_state=14 2个方块的位姿  xyz+wxyz
        # clear unused variables
        del env
        del episode
        del policy

        # setup the environment
#模拟从手（采用关节角度驱动机械臂）------------------------------------------------------
        if stu_flag==1:
            print('-------------------------Replaying joint commands--------------------',task_name)
            env = make_sim_env(task_name)
            BOX_POSE[0] = subtask_info # make sure the sim_env has the same object configurations as ee_sim_env
            ts = env.reset()

            episode_replay = [ts]
            # setup plotting
            plt.figure(figsize=(22,10))
            if onscreen_render:
                ax1 = plt.subplot(1,3,2)
                plt_img1= ax1.imshow(ts.observation['images'][render_cam_name2])
                ax2 = plt.subplot(1,3,1)
                plt_img2= ax2.imshow(ts.observation['images']['left_wrist'])
                ax3 = plt.subplot(1,3,3)
                plt_img3= ax3.imshow(ts.observation['images']['right_wrist'])
                plt.ion()
            for t in range(len(joint_traj)): # note: this will increase episode length by 1
                action = joint_traj[t]#读取之前缓存的角度轨迹
                #仿真从臂时，action是关节角7+1+7+1
                ts = env.step(action)#获取仿真数据 /home/tinymal/下载/aloha/act_plus3/sim_env.py
                episode_replay.append(ts)#写入需要存储的数据 从手
                if onscreen_render:
                    plt_img1.set_data(ts.observation['images'][render_cam_name2])
                    plt_img2.set_data(ts.observation['images']['left_wrist'])
                    plt_img3.set_data(ts.observation['images']['right_wrist'])
                    plt.pause(0.02)#50hz

            episode_return = np.sum([ts.reward for ts in episode_replay[1:]])
            episode_max_reward = np.max([ts.reward for ts in episode_replay[1:]])
            if episode_max_reward == env.task.max_reward:#unuse for trainning
                success.append(1)
                teacher_flag=1
                print(f"{episode_idx} Successful-replay, {episode_return}")
            else:
                success.append(0)
                print(f"{episode_idx} Failed-replay")

            plt.close()

            """
            For each timestep:
            observations
            - images
                - each_cam_name     (480, 640, 3) 'uint8'
            - qpos                  (16,)         'float64'
            - qvel                  (16,)         'float64'

            action                  (16,)         'float64'
            """
            if teacher_flag and stu_flag:    
                data_dict = {
                    '/observations/qpos': [],
                    '/observations/qvel': [],
                    '/action': [],
                }
                print("save camera_names:=",camera_names)
                for cam_name in camera_names:
                    data_dict[f'/observations/images/{cam_name}'] = []

                # because the replaying, there will be eps_len + 1 actions and eps_len + 2 timesteps
                # truncate here to be consistent
                joint_traj = joint_traj[:-1]
                episode_replay = episode_replay[:-1]

                # len(joint_traj) i.e. actions: max_timesteps
                # len(episode_replay) i.e. time steps: max_timesteps + 1
                max_timesteps = len(joint_traj)
                while joint_traj:
                    action = joint_traj.pop(0)
                    print(action)
                    ts = episode_replay.pop(0)
                    data_dict['/observations/qpos'].append(ts.observation['qpos'])
                    data_dict['/observations/qvel'].append(ts.observation['qvel'])
                    data_dict['/action'].append(action)
                    for cam_name in camera_names:
                        data_dict[f'/observations/images/{cam_name}'].append(ts.observation['images'][cam_name])

                # HDF5
                t0 = time.time()
                dataset_path = os.path.join(dataset_dir,task_name, f'episode_{save_idx}')
                with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024 ** 2 * 2) as root:#rdcc_nbytes=1024 ** 2 *
                    root.attrs['sim'] = True
                    obs = root.create_group('observations')
                    image = obs.create_group('images')
                    for cam_name in camera_names:
                        _ = image.create_dataset(cam_name, (max_timesteps, 480, 640, 3), dtype='uint8',
                                                chunks=(1, 480, 640, 3), )
                    # =compression='gzip',compression_opts=2,)
                    # compression=32001, compression_opts=(0, 0, 0, 0, 9, 1, 1), shuffle=False)
                    qpos = obs.create_dataset('qpos', (max_timesteps, (DOF_ARM+1)*2))#doghome 7+7+2
                    qvel = obs.create_dataset('qvel', (max_timesteps, (DOF_ARM+1)*2))
                    action = root.create_dataset('action', (max_timesteps, (DOF_ARM+1)*2))

                    for name, array in data_dict.items():
                        root[name][...] = array
                print(f'Saving: {time.time() - t0:.1f} secs\n')
                save_idx+=1

    print(f'Saved to {dataset_dir}')
    print(f'Success: {np.sum(success)} / {len(success)}')

if __name__ == '__main__':
    from constants_all import DATA_DIR_ALL
    parser = argparse.ArgumentParser()
    # parser.add_argument('--task_name', action='store', type=str, default='sim_insertion_scripted')
    parser.add_argument('--task_name', action='store', type=str, default='human_test1')
    parser.add_argument('--dataset_dir', action='store', type=str, default=DATA_DIR_ALL)#'../saved_data/')
    parser.add_argument('--num_episodes', action='store', type=int, default=8)
    parser.add_argument('--onscreen_render', default=True) #action='store_true'
    
    main(vars(parser.parse_args()))

