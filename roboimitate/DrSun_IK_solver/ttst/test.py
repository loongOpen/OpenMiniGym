import time
import mujoco
import mujoco.viewer
import numpy as np
import socket
import struct
import cv2
from ctypes import *

# 加载模型
m = mujoco.MjModel.from_xml_path('assets/human_test.xml')
d = mujoco.MjData(m)

# 设置相机参数
# camera_name = "front_close"
# camera_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
# if camera_id == -1:
#     raise ValueError(f"Camera '{camera_name}' not found")
#
# # 设置离屏渲染
# viewport_width, viewport_height = 640, 480
# render_context = mujoco.MjrContext(m, mujoco.mjtFontScale.mjFONTSCALE_100.value)
# mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, render_context)
# mujoco.mjr_setViewport(0, 0, viewport_width, viewport_height, render_context)

# lib = CDLL('/home/byz/ik_7dof_fourier/ik_fa.so')
# 关节目标位置和控制速度
# target_joint_position = 1.0  # 目标位置 (弧度)
target_joint_u_d = 18  # 目标位置 (弧度)
server_address = ('127.0.0.1', 5005)
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind(server_address)
# server_address1 = ('127.0.0.1', 5006)
# server_socket1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# server_socket1.bind(server_address1)
# server_socket1.setblocking(False)
up_up_up = [1.0, 3.14, 3.14, 0, 3.14, 1.57, 1.57, 0, 0.024, 1.0, 0, 1.57, 2.8, 3.14, 0.8, 1.57, 0, 0.024]
down_down_down = [-2.6, 0, -3.14, -2.8, -3.14, -1.57, -1.57, -0.024, 0, -2.6, -3.14, -1.57, 0, -3.14, -0.8, -1.57, -0.024, 0]
kp = [200, 250, 40, 40, 10, 10, 10, 300, 300, 200, 250, 40, 40, 10, 10, 10, 300, 300]
kd = [6, 7.5, 1.2, 1.2, 0.3, 0.3, 0.3, 1, 1, 6, 7.5, 1.2, 1.2, 0.3, 0.3, 0.3, 1, 1]
numbers = [-0.5279536247253418, -1.602133870124817, -0.8233610987663269, -1.2198679447174072, -0.9738451242446899, -0.3204014301300049, 0.5859276652336121, 0, 0,            -0.20602308213710785, -2.190190076828003, -1.5612424612045288, -1.3894191980361938, 0.35497674345970154, 0.3094855546951294, 0.10307402163743973, 0, 0]
numbers[1]=numbers[1]+2.007
numbers[2]=numbers[2]+1.5708
# numbers[9]=-numbers[9]
numbers[10]=numbers[10]+2.007
numbers[11]=numbers[11]+1.5708
numbers[12]=-numbers[12]
numbers[13]=-numbers[13]
# numbers[9]=numbers[9]+1.5708
# numbers[5]=numbers[5]+1.5708
image_count = 0  # 计数器，用于保存图像的文件名
lib_l = CDLL('/home/vipp/test/3-ALOHA/act_plus3/DrSun_IK_solver/ttst/ik_7dof_fourier/test_l.so')
# lib_r = CDLL('/home/byz/ik_7dof_fourier/test_r.so')
with mujoco.viewer.launch_passive(m, d) as viewer:
    # start = time.time()
    while viewer.is_running() and time.time():
        step_start = time.time()
        i = 0
        data, address = server_socket.recvfrom(1024)
        # received_numbers = [float(num) for num in data.decode().split(',')]
        # received_numbers = [int.from_bytes(data[i:i + 4], 'little') for i in range(0, len(data), 4)]
        received_numbers = [struct.unpack('>f', data[i:i + 4])[0] for i in range(0, len(data), 4)]

        # data1, address1 = server_socket1.recvfrom(1024)
        # # received_numbers = [float(num) for num in data.decode().split(',')]
        # # received_numbers = [int.from_bytes(data[i:i + 4], 'little') for i in range(0, len(data), 4)]
        # # print(address1)
        # print(data1)
        # received_numbers1 = [struct.unpack('>f', data1[i:i + 4])[0] for i in range(0, len(data1), 4)]
        # received_numbers1 = [float(num) for num in data1.decode().split(',')]
        # received_numbers1 = np.frombuffer(data1, dtype='>f4')
        lib_l.ik_left.restype = POINTER(c_double)
        # lib_l.ik_right.restype = POINTER(c_double)
        print("接收到的浮点数列表:", received_numbers)
        # print(received_numbers1)
        real_arrayleft = (c_double * 7)(received_numbers[0], received_numbers[1], received_numbers[2], received_numbers[3], received_numbers[4], received_numbers[5], received_numbers[6])
        real_arrayright = (c_double * 7)(received_numbers[7], received_numbers[8], received_numbers[9],
                                        received_numbers[10], received_numbers[11], received_numbers[12],
                                        received_numbers[13])
        left_ptr = lib_l.ik_left(real_arrayleft)
        right_ptr = lib_l.ik_left(real_arrayright)
        theta_left = [left_ptr[i] for i in range(7)]
        theta_right = [right_ptr[i] for i in range(7)]
        # theta_right = [-0.20602308213710785, -2.190190076828003, -1.5612424612045288, -1.3894191980361938, 0.35497674345970154, 0.3094855546951294, 0.10307402163743973]
        theta_left[1] = theta_left[1] + 2.007
        theta_left[2] = theta_left[2] + 1.5708
        theta_right[1] = -theta_right[1] - 2.007
        theta_right[2] = theta_right[2] + 1.5708
        theta_right[3] = -theta_right[3]
        theta_right[5] = -theta_right[5]
        theta_right[6] = -theta_right[6]
        number = [theta_left[0], theta_left[1], theta_left[2], theta_left[3], theta_left[4], theta_left[5], theta_left[6], -received_numbers[14], received_numbers[14],        theta_right[0], theta_right[1], theta_right[2], theta_right[3], theta_right[4], theta_right[5], theta_right[6], -received_numbers[15], received_numbers[15]]

        # current_joint_position = d.qpos[1]  # 假设我们控制第一个关节
        # control_signal = np.clip(kp * (received_numbers - d.qpos) - kd * (d.qvel)  , -target_joint_u_d, target_joint_u_d)
        # control_signal = np.clip(received_numbers, down_down_down, up_up_up)
        lib_l.free_memory(left_ptr)
        lib_l.free_memory(right_ptr)
        while i < 18:
            # i=6
            # d.ctrl[i]=control_signal[i]
            # d.ctrl[i] = kp[i] * (received_numbers[i] - d.qpos[i]) - kd[i] * (d.qvel[i])  # 应用控制信号
            d.ctrl[i] = kp[i] * (number[i] - d.qpos[i]) - kd[i] * (d.qvel[i])  # 应用控制信号
            # print(f"{kp[i]}*{received_numbers[i})
            # print(kp[i] * (received_numbers[i] - d.qpos[i]))
            i = i + 1
            # print(i)
        # lib_l.free_memory(left_ptr)
        # lib_r.free_memory(right_ptr)
        # 进行物理仿真步骤
        mujoco.mj_step(m, d)

        # 切换观察器选项
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

        # 更新观察器状态
        viewer.sync()
        # 获取相机图像

        # 离屏渲染并获取图像
        # mujoco.mjv_updateScene(m, d, mujoco.mjvSceneFlags(), None, render_context)
        # pixels = np.zeros((viewport_height, viewport_width, 3), dtype=np.uint8)
        # mujoco.mjr_render(viewport, render_context, m, d)
        # mujoco.mjr_readPixels(pixels, None, viewport, render_context, m, d)
        #
        # # 转换图像格式并保存
        # pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        # image_count += 1
        # filename = f"camera_image_{image_count}.png"
        # cv2.imwrite(filename, pixels)


        # 等待下一个仿真步骤
        # time_until_next_step = m.opt.timestep - (time.time() - step_start)
        # if time_until_next_step > 0:
        #     time.sleep(time_until_next_step)
        # time.sleep(0.02)

print("Simulation finished.")