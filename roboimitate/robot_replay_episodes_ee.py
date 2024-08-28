#读取数据回放到机器人 需要增加id参数
import os
import numpy as np
import cv2
import h5py
import argparse
from tqdm import tqdm
import matplotlib.pyplot as plt
from constants_real import DT,TASK_CONFIGS,FPS
from constants_real  import ROBOT_IP,PORT_UDP,AUTO_START
import time
import IPython
import math
import socket
import struct
import threading
e = IPython.embed

JOINT_NAMES = ["X", "Y", "Z", "ROL", "PIT", "YAW"]
STATE_NAMES = JOINT_NAMES + ["Cap"]
BASE_STATE_NAMES = ["VelX", "RotZ"]

if AUTO_START==0:
    start_replay=0  
else:
    start_replay=1
replay_mode=0
def load_hdf5(dataset_dir, dataset_name):
    dataset_path = os.path.join(dataset_dir, dataset_name + '.hdf5')
    if not os.path.isfile(dataset_path):
        print(f'Dataset does not exist at \n{dataset_path}\n')
        exit()

    with h5py.File(dataset_path, 'r') as root:
        is_sim = root.attrs['sim']
        compressed = root.attrs.get('compress', False)
        qpos = root['/observations/qpos'][()]
        qvel = root['/observations/qvel'][()]
        if 'effort' in root.keys():
            effort = root['/observations/effort'][()]
        else:
            effort = None
        action = root['/action'][()]
        base_action = root['/base_action'][()]
        image_dict = dict()
        for cam_name in root[f'/observations/images/'].keys():
            image_dict[cam_name] = root[f'/observations/images/{cam_name}'][()]
        if compressed:
            compress_len = root['/compress_len'][()]

    if compressed:
        for cam_id, cam_name in enumerate(image_dict.keys()):
            # un-pad and uncompress
            padded_compressed_image_list = image_dict[cam_name]
            image_list = []
            for frame_id, padded_compressed_image in enumerate(padded_compressed_image_list): # [:1000] to save memory
                image_len = int(compress_len[cam_id, frame_id])
                compressed_image = padded_compressed_image
                image = cv2.imdecode(compressed_image, 1)
                image_list.append(image)
            image_dict[cam_name] = image_list


    return qpos, qvel, effort, action, base_action, image_dict


tx_data_udp =  [0]*500
arm_pos_exp=[0.35,-0.1,0]
arm_att_exp=[0,0,0]
cap_rate_exp=[1,1]
base_vel_exp=[0,0,0]
key=[0,0,0,0,0]
play_rate=0.0

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

def udp_convert_tx(qpos,action,base_action): 
    global tx_data_udp,play_rate
    tx_data_udp_temp=[]
    send_float(tx_data_udp_temp,10)
    send_float(tx_data_udp_temp,base_action[0])
    send_float(tx_data_udp_temp,0)
    send_float(tx_data_udp_temp,base_action[1])
 
    send_float(tx_data_udp_temp,action[0])#x
    send_float(tx_data_udp_temp,action[1])#y
    send_float(tx_data_udp_temp,action[2])#z

    send_float(tx_data_udp_temp,action[3])#r
    send_float(tx_data_udp_temp,action[4])#p
    send_float(tx_data_udp_temp,action[5])#y

    send_float(tx_data_udp_temp,action[6])#cap

    send_float(tx_data_udp_temp,action[7])#x
    send_float(tx_data_udp_temp,action[8])#y
    send_float(tx_data_udp_temp,action[9])#z

    send_float(tx_data_udp_temp,action[10])#r
    send_float(tx_data_udp_temp,action[11])#p
    send_float(tx_data_udp_temp,action[12])#y

    send_float(tx_data_udp_temp,action[13])#cap

    send_float(tx_data_udp_temp,play_rate)#
    #print(qpos)
    
    for i in range(len(tx_data_udp_temp)):
        tx_data_udp[i]=tx_data_udp_temp[i]
    return len(tx_data_udp_temp) 
 
                
def main(args):
    from constants_real import TASK_NAME
    global play_rate,start_replay,replay_mode
    addr_udp_ocu =(ROBOT_IP,PORT_UDP)# ("127.0.0.1", 3333)#send to this port
    ser_udp_ocu = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Note::replay system on, waiting for trigger command!")
    print("Note::you need to reset Sim world first, then run this scripted!")
    print("Note::In simulation Windows,Press J to start,Press K to stop!")    
    while 1:
        if start_replay==1:
            replay_mode=1
            dataset_dir = args['dataset_dir']
            episode_idx = args['id']
            ismirror = args['ismirror']
            if ismirror:
                dataset_name = f'mirror_episode_{episode_idx}'
            else:
                dataset_name = f'episode_{episode_idx}'

            task_config = TASK_CONFIGS[TASK_NAME]
            

            dataset_name = f'episode_{episode_idx}'
            dataset_dir = task_config['dataset_dir']    
            dataset_path = os.path.join(dataset_dir, dataset_name + '.hdf5')

            qpos, qvel, effort, action, base_action, image_dict = load_hdf5(dataset_dir, dataset_name)
            print('hdf5 loaded:',dataset_dir,dataset_name)
            #print(len(base_action))
            #print(base_action[0],qpos[0])
            #读取并驱动机器人
            FILE_LEN=len(base_action)
            DT = 1 / FPS
            for i in tqdm(range(FILE_LEN)):
                if start_replay==0:
                    replay_mode=0
                    print("Force quit replay!")
                    break

                time1 = time.time()
                play_rate= (float)(i)/(float)(FILE_LEN)
                
                try:
                    len_tx=udp_convert_tx(qpos[i],action[i],base_action[i])#转换发送协议
                except:
                    len_tx=0
                #print(len_tx)
                if len_tx:
                    tx_data_temp =  [0]*len_tx
                    for i in range(len_tx):
                        tx_data_temp[i]=tx_data_udp[i]
                    try:
                        ser_udp_ocu.sendto(bytearray (tx_data_temp), addr_udp_ocu)#在此将机器人状态数据回传OCU
                        #print(tx_data_temp)
                    except:
                        print("Sending Error!!!\n")

                # print("time1:",time1)
                # print("qpos:",qpos[i])
                # print("acti:",action[i])
                # print("base:",base_action[i])
        
                time.sleep(max(0, DT - (time.time() - time1)))
            
            if start_replay==1:
                save_videos(image_dict, DT, video_path=os.path.join(dataset_dir, dataset_name + '_video.avi'))
                visualize_joints(qpos, action, plot_path=os.path.join(dataset_dir, dataset_name + '_epos.png'))
                # visualize_single(effort, 'effort', plot_path=os.path.join(dataset_dir, dataset_name + '_effort.png'))
                # visualize_single(action - qpos, 'tracking_error', plot_path=os.path.join(dataset_dir, dataset_name + '_error.png'))
                visualize_base(base_action, plot_path=os.path.join(dataset_dir, dataset_name + '_base_action.png'))
                # visualize_timestamp(t_list, dataset_path) # TODO addn timestamp back
                start_replay=0
            replay_mode=0


def save_videos(video, dt, video_path=None):
    if isinstance(video, list):
        cam_names = list(video[0].keys())
        h, w, _ = video[0][cam_names[0]].shape
        w = w * len(cam_names)
        fps = int(1/dt)
        out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc('I','4','2','0'), fps, (w, h))
        for ts, image_dict in enumerate(video):
            images = []
            for cam_name in cam_names:
                image = image_dict[cam_name]
                image = image[:, :, [2, 1, 0]] # swap B and R channel
                images.append(image)
            images = np.concatenate(images, axis=1)
            out.write(images)
        out.release()
        print(f'Saved video to: {video_path}')
    elif isinstance(video, dict):
        cam_names = list(video.keys())
        all_cam_videos = []
        for cam_name in cam_names:
            all_cam_videos.append(video[cam_name])
        all_cam_videos = np.concatenate(all_cam_videos, axis=2) # width dimension

        n_frames, h, w, _ = all_cam_videos.shape
        fps = int(1 / dt)
        out = cv2.VideoWriter(video_path,cv2.VideoWriter_fourcc('I','4','2','0'), fps, (w, h))
        for t in range(n_frames):
            image = all_cam_videos[t]
            image = image[:, :, [2, 1, 0]]  # swap B and R channel
            out.write(image)
        out.release()
        print(f'Saved video to: {video_path}')


def visualize_joints(qpos_list, command_list, plot_path=None, ylim=None, label_overwrite=None):
    if label_overwrite:
        label1, label2 = label_overwrite
    else:
        label1, label2 = 'State', 'Command'

    qpos = np.array(qpos_list) # ts, dim
    command = np.array(command_list)
    num_ts, num_dim = qpos.shape
    h, w = 2, num_dim
    num_figs = num_dim
    fig, axs = plt.subplots(num_figs, 1, figsize=(8, 2 * num_dim))

    # plot joint state
    all_names = [name + '_left' for name in STATE_NAMES] + [name + '_right' for name in STATE_NAMES]
    for dim_idx in range(num_dim):
        ax = axs[dim_idx]
        ax.plot(qpos[:, dim_idx], label=label1)
        ax.set_title(f'Joint {dim_idx}: {all_names[dim_idx]}')
        ax.legend()

    # plot arm command
    for dim_idx in range(num_dim):
        ax = axs[dim_idx]
        ax.plot(command[:, dim_idx], label=label2)
        ax.legend()

    if ylim:
        for dim_idx in range(num_dim):
            ax = axs[dim_idx]
            ax.set_ylim(ylim)

    plt.tight_layout()
    plt.savefig(plot_path)
    print(f'Saved qpos plot to: {plot_path}')
    plt.close()

def visualize_single(efforts_list, label, plot_path=None, ylim=None, label_overwrite=None):
    efforts = np.array(efforts_list) # ts, dim
    num_ts, num_dim = efforts.shape
    h, w = 2, num_dim
    num_figs = num_dim
    fig, axs = plt.subplots(num_figs, 1, figsize=(w, h * num_figs))

    # plot joint state
    all_names = [name + '_left' for name in STATE_NAMES] + [name + '_right' for name in STATE_NAMES]
    for dim_idx in range(num_dim):
        ax = axs[dim_idx]
        ax.plot(efforts[:, dim_idx], label=label)
        ax.set_title(f'Joint {dim_idx}: {all_names[dim_idx]}')
        ax.legend()

    if ylim:
        for dim_idx in range(num_dim):
            ax = axs[dim_idx]
            ax.set_ylim(ylim)

    plt.tight_layout()
    plt.savefig(plot_path)
    print(f'Saved effort plot to: {plot_path}')
    plt.close()

def visualize_base(readings, plot_path=None):
    readings = np.array(readings) # ts, dim
    num_ts, num_dim = readings.shape
    num_figs = num_dim
    fig, axs = plt.subplots(num_figs, 1, figsize=(8, 2 * num_dim))

    # plot joint state
    all_names = BASE_STATE_NAMES
    for dim_idx in range(num_dim):
        ax = axs[dim_idx]
        ax.plot(readings[:, dim_idx], label='raw')
        ax.plot(np.convolve(readings[:, dim_idx], np.ones(20)/20, mode='same'), label='smoothed_20')
        ax.plot(np.convolve(readings[:, dim_idx], np.ones(10)/10, mode='same'), label='smoothed_10')
        ax.plot(np.convolve(readings[:, dim_idx], np.ones(5)/5, mode='same'), label='smoothed_5')
        ax.set_title(f'Joint {dim_idx}: {all_names[dim_idx]}')
        ax.legend()

    # if ylim:
    #     for dim_idx in range(num_dim):
    #         ax = axs[dim_idx]
    #         ax.set_ylim(ylim)

    plt.tight_layout()
    plt.savefig(plot_path)
    print(f'Saved effort plot to: {plot_path}')
    plt.close()


def visualize_timestamp(t_list, dataset_path):
    plot_path = dataset_path.replace('.pkl', '_timestamp.png')
    h, w = 4, 10
    fig, axs = plt.subplots(2, 1, figsize=(w, h*2))
    # process t_list
    t_float = []
    for secs, nsecs in t_list:
        t_float.append(secs + nsecs * 10E-10)
    t_float = np.array(t_float)

    ax = axs[0]
    ax.plot(np.arange(len(t_float)), t_float)
    ax.set_title(f'Camera frame timestamps')
    ax.set_xlabel('timestep')
    ax.set_ylabel('time (sec)')

    ax = axs[1]
    ax.plot(np.arange(len(t_float)-1), t_float[:-1] - t_float[1:])
    ax.set_title(f'dt')
    ax.set_xlabel('timestep')
    ax.set_ylabel('time (sec)')

    plt.tight_layout()
    plt.savefig(plot_path)
    print(f'Saved timestamp plot to: {plot_path}')
    plt.close()


def decode_float(rx_Buf,start_Byte_num):
    global rx_num_now
    temp=bytes([rx_Buf[start_Byte_num],rx_Buf[start_Byte_num+1],rx_Buf[start_Byte_num+2],rx_Buf[start_Byte_num+3]])
    rx_num_now= rx_num_now+4
    return struct.unpack('f',temp)[0]

def udp_convert_rx(data_rx): 
    from constants_real import AUTO_START
    global rx_num_now,start_replay
    tx_data_udp_temp=[]
    rx_num_now = 0
    if AUTO_START==0:
        start_replay=decode_float(data_rx,rx_num_now)

def udp_convert_tx_mode(): 
    global tx_data_udp,replay_mode
    tx_data_udp_temp=[]
    send_float(tx_data_udp_temp,replay_mode)
    send_float(tx_data_udp_temp,0)
    send_float(tx_data_udp_temp,(float)(0))
    send_float(tx_data_udp_temp,(float)(0))#
    for i in range(len(tx_data_udp_temp)):
        tx_data_udp[i]=tx_data_udp_temp[i]
    return len(tx_data_udp_temp) 

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
            len_tx=udp_convert_tx_mode()#转换发送协议
        except:
            len_tx=0

        if len_tx:
            tx_data_temp =  [0]*len_tx
            for i in range(len_tx):
                tx_data_temp[i]=tx_data_udp[i]
            try:
                ser_udp_ocu.sendto(bytearray (tx_data_temp), addr_udp_ocu)#在此将机器人状态数据回传OCU
                # print(tx_data_temp)
            except:
                print("Sending Error!!!\n")

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
    parser.add_argument('--dataset_dir', action='store', type=str, help='Dataset dir.', required=False)
    parser.add_argument('--id', action='store', type=int, help='Episode index.', required=True)
    parser.add_argument('--ismirror', action='store_true')
    #开启线程1
    mythread = threading.Thread(target=thread_udp_send, name='UDP线程')
    cond = threading.Condition() # 锁
    mythread.start()

    #开启线程2
    mythread1 = threading.Thread(target=main(vars(parser.parse_args())), name='播放')
    cond1 = threading.Condition() # 锁
    mythread1.start()
