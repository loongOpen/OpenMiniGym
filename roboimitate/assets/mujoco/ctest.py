import time
import mujoco
import mujoco.viewer
import numpy as np
import socket


# 加载模型
m = mujoco.MjModel.from_xml_path('/home/jiao/act-plus-plus/sonnieURDF_upbody/up_people.xml')
# m = mujoco.MjModel.from_xml_path('/home/jiao/act-plus-plus/assets/vx300s_right.xml')
d = mujoco.MjData(m)

target_joint_position = 1.0  # 目标位置 (弧度)
target_joint_u_d = 18  # 目标位置 (弧度)
server_address = ('192.168.112.35', 5005)
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind(server_address)
kp = [200, 250, 40, 40, 10, 10, 1, 200, 250, 40, 40, 10, 10, 1]
kd = [2, 2.5, 0.4, 0.4, 0.1, 0.1, 0.01, 2, 2.5, 0.4, 0.4, 0.1, 0.1, 0.01]

with mujoco.viewer.launch_passive(m, d) as viewer:
    # start = time.time()
    while viewer.is_running() and time.time():
        step_start = time.time()
        i = 0
        data, address = server_socket.recvfrom(1024)
        received_numbers = [float(num) for num in data.decode().split(',')]
        # current_joint_position = d.qpos[1]  # 假设我们控制第一个关节
        control_signal = np.clip(kp * (received_numbers - d.qpos) - kd * (d.qvel)  , -target_joint_u_d, target_joint_u_d)
        while i < 14:
            # i=6
            d.ctrl[i]=control_signal[i]
            # d.ctrl[i] = kp[i] * (received_numbers[i] - d.qpos[i]) - kd[i] * (d.qvel[i])  # 应用控制信号
            # print(f"{kp[i]}*{received_numbers[i})
            # print(kp[i] * (received_numbers[i] - d.qpos[i]))
            i = i + 1
            # print(i)

        # 进行物理仿真步骤
        mujoco.mj_step(m, d)

        # 切换观察器选项
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

        # 更新观察器状态
        viewer.sync()

        # 等待下一个仿真步骤
        # time_until_next_step = m.opt.timestep - (time.time() - step_start)
        # if time_until_next_step > 0:
        #     time.sleep(time_until_next_step)
        time.sleep(0.01)

print("Simulation finished.")
