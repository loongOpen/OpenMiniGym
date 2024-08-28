#录制数据   数据长度在constants中 录制话题在robot_utils  末端位姿版本
import os
import time
import h5py
import argparse
import numpy as np
from tqdm import tqdm
import cv2
import threading
from constants_real import FPS_RECORD, START_ARM_POSE, TASK_CONFIGS, FPS,DOF_CAP,DOF_ARM, DT,HAND_NUM
from robot_utils import Recorder, ImageRecorder, get_arm_gripper_positions
from robot_utils import move_arms, torque_on, torque_off, move_grippers
from real_env import make_real_env, get_action
from constants_real  import ROBOT_IP,PORT_UDP
import IPython
import math
import socket
import struct

e = IPython.embed

#使用 /visualize_episodes.py 绘制录制的数据曲线 

tx_data_udp =  [0]*500
play_rate=0.0
record_mode=0
start_record=1
episode_idx=0
def send_float(tx_Buf,data):
    temp_B= struct.pack('f',float(data))
    tx_Buf.append(temp_B[0])
    tx_Buf.append(temp_B[1])
    tx_Buf.append(temp_B[2])
    tx_Buf.append(temp_B[3])

def send_int(tx_Buf,data):
    temp_B= struct.pack('i',int(data))
    tx_Buf.append(temp_B[0])
    tx_Buf.append(temp_B[1])
    tx_Buf.append(temp_B[2])
    tx_Buf.append(temp_B[3])

def send_char(tx_Buf,data):
    tx_Buf.append(int(data))

def udp_convert_tx(): 
    global tx_data_udp,play_rate,record_mode
    tx_data_udp_temp=[]
    send_float(tx_data_udp_temp,record_mode)
    send_float(tx_data_udp_temp,play_rate)
    send_float(tx_data_udp_temp,(float)(episode_idx))
    
    for i in range(len(tx_data_udp_temp)):
        tx_data_udp[i]=tx_data_udp_temp[i]
    return len(tx_data_udp_temp) 
 
def wait_for_input():
    print("Press 'o' to start recording, 'p' to stop recording:")
    while True:
        key = input().strip().lower()
        if key == 'o':
            return 1
        elif key == 'p':
            return 0
        else:
            print("Invalid input. Press 'o' to start recording, 'p' to stop recording:")

def capture_one_episode(dt, max_timesteps, camera_names, dataset_dir, dataset_name, overwrite):
    global play_rate,tx_data_udp,record_mode,start_record
    print(f'Dataset name: {dataset_name}')
    env = make_real_env(init_node=False, setup_robots=False)
    # saving dataset
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    dataset_path = os.path.join(dataset_dir, dataset_name)
    print("dataset_path:",dataset_path)
    if os.path.isfile(dataset_path) and not overwrite:
        print(f'Dataset already exist at \n{dataset_path}\nHint: set overwrite to True.')
        exit()
    print("Wait Recorder")
    time.sleep(1)
    print("Start")
    # Data collection
    ts = env.reset(fake=True)
    timesteps = [ts]
    actions = []
    actual_dt_history = []
    time0 = time.time()
    DT = 1 / FPS_RECORD
    t_print=0

    # addr_udp_ocu =(ROBOT_IP,PORT_UDP)# ("127.0.0.1", 3333)#send to this port
    # ser_udp_ocu = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    record_mode=1
    for t in tqdm(range(max_timesteps)):
        play_rate= (float)(t)/(float)(max_timesteps)

        # try:
        #     len_tx=udp_convert_tx()#转换发送协议
        # except:
        #     len_tx=0

        # if len_tx:
        #     tx_data_temp =  [0]*len_tx
        #     for i in range(len_tx):
        #         tx_data_temp[i]=tx_data_udp[i]
        #     try:
        #         ser_udp_ocu.sendto(bytearray (tx_data_temp), addr_udp_ocu)#在此将机器人状态数据回传OCU
        #     except:
        #         print("Sending Error!!!\n")

        t0 = time.time() #
        action = env.get_action()#IK的角度指令
        t1 = time.time() #
        ts = env.step(action)
        t2 = time.time() #
        timesteps.append(ts)
        actions.append(action)
        actual_dt_history.append([t0, t1, t2])#时间  
        time.sleep(max(0, DT - (time.time() - t0)))
        if(time.time()-t_print>2) and 0:
            t_print=time.time()
            print("qpos:",ts.observation['qpos'])
            print("acti:",action)
            print("base:",ts.observation['base_vel'])
        # data_dict['/observations/qpos'].append(ts.observation['qpos'])
        # data_dict['/observations/qvel'].append(ts.observation['qvel'])
        # data_dict['/observations/effort'].append(ts.observation['effort'])
        # data_dict['/action'].append(action)
        # data_dict['/base_action'].append(ts.observation['base_vel'])
        if start_record==0:
            record_mode=0
            start_record=0
            print("Force Stop Recording!")
            return False

    print(f'Avg fps: {max_timesteps / (time.time() - time0)}')

   

    freq_mean = print_dt_diagnosis(actual_dt_history)
    if freq_mean < 30:
        print(f'\n\nfreq_mean is {freq_mean}, lower than 30, re-collecting... \n\n\n\n')
        return False

    """
    For each timestep:
    observations
    - images
        - cam_high          (480, 640, 3) 'uint8'
        - cam_low           (480, 640, 3) 'uint8'
        - cam_left_wrist    (480, 640, 3) 'uint8'
        - cam_right_wrist   (480, 640, 3) 'uint8'
    - qpos                  (14,)         'float64'
    - qvel                  (14,)         'float64'
    
    action                  (14,)         'float64'
    base_action             (2,)          'float64'
    """

    data_dict = {
        '/observations/qpos': [],
        '/observations/qvel': [],
        '/observations/effort': [],
        '/action': [],
        '/base_action': [],
        # '/base_action_t265': [],
    }
    for cam_name in camera_names:
        print("!!!!!!!!!!!!!!")
        print(camera_names)
        data_dict[f'/observations/images/{cam_name}'] = []

    # len(action): max_timesteps, len(time_steps): max_timesteps + 1
    while actions:
        action = actions.pop(0)
        ts = timesteps.pop(0)
        data_dict['/observations/qpos'].append(ts.observation['qpos'])
        data_dict['/observations/qvel'].append(ts.observation['qvel'])
        data_dict['/observations/effort'].append(ts.observation['effort'])
        data_dict['/action'].append(action)
        data_dict['/base_action'].append(ts.observation['base_vel'])
        # data_dict['/base_action_t265'].append(ts.observation['base_vel_t265'])
        for cam_name in camera_names:
            data_dict[f'/observations/images/{cam_name}'].append(ts.observation['images'][cam_name])
    
    # plot /base_action vs /base_action_t265
    # import matplotlib.pyplot as plt
    # plt.plot(np.array(data_dict['/base_action'])[:, 0], label='base_action_linear')
    # plt.plot(np.array(data_dict['/base_action'])[:, 1], label='base_action_angular')
    # plt.plot(np.array(data_dict['/base_action_t265'])[:, 0], '--', label='base_action_t265_linear')
    # plt.plot(np.array(data_dict['/base_action_t265'])[:, 1], '--', label='base_action_t265_angular')
    # plt.legend()
    # plt.savefig('record_episodes_vel_debug.png', dpi=300)

    COMPRESS = True

    if COMPRESS:
        # JPEG compression
        t0 = time.time()
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50] # tried as low as 20, seems fine
        compressed_len = []
        for cam_name in camera_names:
            image_list = data_dict[f'/observations/images/{cam_name}']
            compressed_list = []
            compressed_len.append([])
            for image in image_list:
                result, encoded_image = cv2.imencode('.jpg', image, encode_param) # 0.02 sec # cv2.imdecode(encoded_image, 1)
                compressed_list.append(encoded_image)
                compressed_len[-1].append(len(encoded_image))
            data_dict[f'/observations/images/{cam_name}'] = compressed_list
        print(f'compression: {time.time() - t0:.2f}s')

        # pad so it has same length
        t0 = time.time()
        compressed_len = np.array(compressed_len)
        padded_size = compressed_len.max()
        for cam_name in camera_names:
            compressed_image_list = data_dict[f'/observations/images/{cam_name}']
            padded_compressed_image_list = []
            for compressed_image in compressed_image_list:
                padded_compressed_image = np.zeros(padded_size, dtype='uint8')
                image_len = len(compressed_image)
                # print("!!!!!!!!!!!!!!!!!!!!!!!!!!1")
                # print(image_len)
                padded_compressed_image[:image_len] = compressed_image.flatten()
                padded_compressed_image_list.append(padded_compressed_image)
            data_dict[f'/observations/images/{cam_name}'] = padded_compressed_image_list
        print(f'padding: {time.time() - t0:.2f}s')

    # HDF5
    t0 = time.time()
    with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024**2*2) as root:
        root.attrs['sim'] = False
        root.attrs['compress'] = COMPRESS
        obs = root.create_group('observations')
        image = obs.create_group('images')
        for cam_name in camera_names:
            if COMPRESS:
                _ = image.create_dataset(cam_name, (max_timesteps, padded_size), dtype='uint8',
                                         chunks=(1, padded_size), )
            else:
                _ = image.create_dataset(cam_name, (max_timesteps, 480, 640, 3), dtype='uint8',
                                         chunks=(1, 480, 640, 3), )
        _ = obs.create_dataset('qpos', (max_timesteps, (6+DOF_CAP)*HAND_NUM))#dof all ee
        _ = obs.create_dataset('qvel', (max_timesteps, (6+DOF_CAP)*HAND_NUM))
        _ = obs.create_dataset('effort', (max_timesteps, (6+DOF_CAP)*HAND_NUM))
        _ = root.create_dataset('action', (max_timesteps, (6+DOF_CAP)*HAND_NUM))
        _ = root.create_dataset('base_action', (max_timesteps, 2))
        # _ = root.create_dataset('base_action_t265', (max_timesteps, 2))

        for name, array in data_dict.items():
            root[name][...] = array

        if COMPRESS:
            _ = root.create_dataset('compress_len', (len(camera_names), max_timesteps))
            root['/compress_len'][...] = compressed_len

    print(f'Saving: {time.time() - t0:.1f} secs')
    record_mode=0
    start_record = wait_for_input()
    return True


def main(args):
    from constants_real import TASK_NAME
    global episode_idx
    #task_config = TASK_CONFIGS[args['task_name']]
    task_config = TASK_CONFIGS[TASK_NAME]
    dataset_dir = task_config['dataset_dir']
    max_timesteps = task_config['episode_len']
    camera_names = task_config['camera_names']
    print("Record system on, waiting for trigger command!")
    print("Note::you need to reset Sim world first, then run this scripted!")
    print("Note::In simulation Windows,Press J to start,Press K to stop!")
    while True:
        if start_record==1:
            if args['episode_idx'] is not None:
                episode_idx = args['episode_idx']
            else:
                episode_idx = get_auto_index(dataset_dir)
            overwrite = True
             
            dataset_name = f'episode_{episode_idx}'
            print(dataset_name + '\n')
            is_healthy = capture_one_episode(DT, max_timesteps, camera_names, dataset_dir, dataset_name, overwrite)
            # if is_healthy:
            #     break


def get_auto_index(dataset_dir, dataset_name_prefix = '', data_suffix = 'hdf5'):
    max_idx = 1000
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    for i in range(max_idx+1):
        if not os.path.isfile(os.path.join(dataset_dir, f'{dataset_name_prefix}episode_{i}.{data_suffix}')):
            return i
    raise Exception(f"Error getting auto index, or more than {max_idx} episodes")


def print_dt_diagnosis(actual_dt_history):
    actual_dt_history = np.array(actual_dt_history)
    get_action_time = actual_dt_history[:, 1] - actual_dt_history[:, 0]
    step_env_time = actual_dt_history[:, 2] - actual_dt_history[:, 1]
    total_time = actual_dt_history[:, 2] - actual_dt_history[:, 0]

    dt_mean = np.mean(total_time)
    dt_std = np.std(total_time)
    freq_mean = 1 / dt_mean
    print(f'Avg freq: {freq_mean:.2f} Get action: {np.mean(get_action_time):.3f} Step env: {np.mean(step_env_time):.3f}')
    return freq_mean

def debug():
    print(f'====== Debug mode ======')
    recorder = Recorder('right', is_debug=True)
    image_recorder = ImageRecorder(init_node=False, is_debug=True)
    while True:
        time.sleep(1)
        recorder.print_diagnostics()
        image_recorder.print_diagnostics()

def decode_float(rx_Buf,start_Byte_num):
    global rx_num_now
    temp=bytes([rx_Buf[start_Byte_num],rx_Buf[start_Byte_num+1],rx_Buf[start_Byte_num+2],rx_Buf[start_Byte_num+3]])
    rx_num_now= rx_num_now+4
    return struct.unpack('f',temp)[0]

def udp_convert_rx(data_rx): 
    global rx_num_now,record_mode,start_record
    tx_data_udp_temp=[]
    rx_num_now = 0
    start_record=decode_float(data_rx,rx_num_now)
    # print(start_record)

def thread_udp_send():#状态 客户端 
    global tx_data_udp
    addr_udp_ocu =(ROBOT_IP,PORT_UDP)# ("127.0.0.1", 3333)#send to this port
    ser_udp_ocu = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #local_addr = ("", 3334)#本地端口
    #ser_udp_ocu.bind(local_addr)

    delay_time=0.02
    while 1:
        time.sleep(delay_time)
        try:
            len_tx=udp_convert_tx()#转换发送协议
        except:
            len_tx=0
        # print(len_tx)
        if len_tx:
            tx_data_temp =  [0]*len_tx
            for i in range(len_tx):
                tx_data_temp[i]=tx_data_udp[i]
            try:
                ser_udp_ocu.sendto(bytearray (tx_data_temp), addr_udp_ocu)#在此将机器人状态数据回传OCU
            except:
                print("Sending Error1!!!\n")

            #print(tx_data_temp)
        #读取遥控器指令，并由状态机完成操控
        try:
            data, addr = ser_udp_ocu.recvfrom(256)
            if data:
                udp_convert_rx(data)
                #print ("got data from", addr, 'len=',len(data))#在此对遥控器下发指令进行解码
                #print (data)
        except:
            if data:
                udp_convert_rx(data)
            print ("err UDP data from", addr, 'len=',len(data))
                
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_name', action='store', type=str, help='Task name.', required=False)
    parser.add_argument('--episode_idx', action='store', type=int, help='Episode index.', default=None, required=False)
    
    #开启线程1
    mythread = threading.Thread(target=thread_udp_send, name='UDP线程')
    cond = threading.Condition() # 锁
    mythread.start()

    #开启线程2
    mythread1 = threading.Thread(target=main(vars(parser.parse_args())), name='录制')
    cond1 = threading.Condition() # 锁
    mythread1.start()

    #main(vars(parser.parse_args())) # TODO
    #debug()


