1. IK，需要重新编译生成适配的test_l.so：
```
cd somewhere && DrSun_IK_solver/ttst/ik_7dof_fourier
gedit test_l.cpp
#修改line 11的路径，保存退出
#void* handle = dlopen("X /DrSun_IK_solver/ttst/ik_7dof_fourier/libarmdecode_v5_L.so", RTLD_LAZY);
g++ -o test_l.so -shared -fPIC test_l.cpp
```
2. 修改ee_sim_env.py路径：
```
import sys
sys.path.append(r'XXX/DrSun_IK_solver/ttst')
```
3. 运行:
```
#生成示教数据并存储至save_data：
record_sim_episodes.py
#训练&测试：
imitate_episodes.py
```

#-----------------------------------机器人数据采集<br>
在constant_all.py中修改全局路径参数<br>
robot_record_episodes.py  录制数据  在 constants_real中修改录制长度与存储路径<br>
robot_replay_episodes.py --id 1 播放数据驱动机器人并绘制数据曲线<br>

imitate_learning.py 采用数据进行训练<br>
imitate_run.py 采用训练好的模型驱动机器人<br>

ee后缀为直接记录机械臂末端的数据版本<br>