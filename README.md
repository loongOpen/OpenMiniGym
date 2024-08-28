## OpenMiniGym开源具身道场机器人
> 一个轻量化的开源具身智能抓取机器人项目， 包含Robohome训练道场，RoboButler低成本开源抓取机器人，RoboImitate具身模仿学习软件包，RoboGrasp通用抓取软件包，RoboOcu机器人遥操控软件包。项目提供了基于Webots和Mujoco下的数字仿真环境，通过软硬件解耦设计方法，可以采用统一的软件代码完成对仿真和实物样机的驱动，从而加速控制软件的开发，同时也可以在数字仿真环境中产生合成数据，从而用于对模型进行预训练并开展对具身模仿学习算法的研究，同时面向规则物品抓取提供了基于PointsCloud的端到端抓取算法可以完成最优抓取位姿估计，最终面向机器人远程操控提供了基于OpenVPN下的内网穿透和多样化的遥操控接口（键盘、遥控器、体感）。

| 软件包 | 说明 | 依赖|
|:-------:|:-------:|:-------:|
| Robohome | 具身智能训练道场提供Webots和Mujoco下的仿真 | Webots2021a，Ubuntu20.04，pinocchio |
| RoboImitate| 基于扩散神经网络的具身模仿学习软件包 | pytorch，cuda |
| RoboButler| 开源低成本具身抓取机器人软件包 | QT，RBDL |
| RoboGrasp| 基于PointCloud的点云端到端抓取位姿估计软件包| ROS Noetic，PCL |
| RoboOcu| 机器人操作上位机，遥操控，手势识别等软件包，内网穿透工具| python，OpenVPN |

## 1.Robohome数字道场软件包
> 具身智能训练道场，是一个虚拟的数据训练环境，可以在其中加载RoboButler机器人完成对模仿学习数据的采集，并可用于自主导航、抓取控制等算法的开发，数字道场采用共享内存方式与ROS进行通讯，从而实现了软硬件解耦隔离，目前采用Webots2021a或Mujoco为主要的物理仿真引擎。  <br />
**Webots2021a**<br />
**ROS noetic**<br />
**Pinocchio** <br /> 
**RBLD** <br />
**Mujoco210** <br />  
Robohome项目源码在本项目Robohome文件夹中，其中包含了必要的控制器、模型与World: <br />

### 1.1 Webots下的仿真
>（1）在Ubuntu20.04系统中完成Webots2021a版本软件安装与ROS noetic软件的安装。<br />
>（2）拷贝Third party中的pinocchio动力学库到本地，完成cmake .. 和make， make install进行安装。<br />
>（3）拷贝Robohome软件包到本地，解压world文件夹的素材与材质，在controllers\tinyscretch文件夹新建build目录。<br />
>（4）在math_src文件中打开kin_math_pino.cpp，修改调用的URDF路径，在build目录中进行cmake ..和make。<br />
>（5）将build目录下生成的tinystech可执行文件复制到\controllers\tinyscretch下替换原始文件。（也可以安装QT creator加载cmakelist进行编译）<br />
>（6）打开Webots加载apartment_simple_big1文件，如在控制台没有报错，机器人控制器加载正常。<br />
>（7）点击Webots仿真窗口采用如下的键盘进行操作。<br /><br />

**其中数字按键1~5可以让机器人进入自主模式实现在仿真中WAY*路点位置的自主机动：**<br />
**handcontrol2.py为手势操控，需自行安装python和google开源识别框架，启动时在采用的相机前方30~70cm处进行手势操控**

### 1.2 Mujoco下的仿真
>（1）在Ubuntu20.04系统中完成Mujoco的安装。<br />
>（2）解压缩mujoco仿真软件包，配置好Mujoco依赖的相关目录。<br />
>（3）在math_src文件中打开kin_math_pino.cpp，修改调用的URDF路径为自己的目录，在build目录中进行cmake ..和make。<br />
>（4）将build目录下生成运行./test，启动Mujoco软件画面<br />
>（5）在打开的仿真界面中采用同样的键盘进行操作，如果无法操作可以确认update_joy中key打印函数是否和宏定义一样。<br />

## 2.RoboGrasp通用抓取软件包
> 面向在场景中规则物体，如杯子、方块、易拉罐、纸团，通过头部相机点云可以完成对桌面的剔除分割，提供了基于几何聚类与神经网络两类方式的抓取算法，可以实现平面抓取与最优姿态抓取，结合模仿学习算法可在完成抓取后进一步进行复杂的作业技能执行。  <br />

### 2.1 几何聚类抓取
>（1）启动头部Realsense相机roslaunch realsense2_camera point.launch 。<br />
>（2）去除桌面点云roslaunch point_cloud_process get_table_top_points.launch 。<br />
>（3）在script中运行o3.py, 在线新窗口运行pca_new.py。<br />
>（4）修改机器人主控IP运行roslaunch robot_sim gpd_run.launch 发送抓取位姿<br />

### 2.2 点云神经网络抓取
>（1）启动头部Realsense相机roslaunch realsense2_camera point.launch 。<br />
>（2）去除桌面点云roslaunch point_cloud_process get_table_top_points.launch 。<br />
>（3）运行神经网络roslaunch gpd_ros tinyarm.launch，修改配置文件ros_eigen_params中相关路径否则读取不到参数 。<br />
>（4）修改机器人主控IP运行roslaunch robot_sim gpd_run.launch 发送抓取位姿<br />

## 3.RoboImitate具身模仿学习软件包
> 面机器人单臂或双臂复杂作业技能，如打开抽屉、缠绕、叠衣服等动作，采用传统的运动规划设计复杂度高，同时难以实现重规划，因此采用模仿学习通过不断示教，基于扩散神经网络生成具有重规划的动态作业行为。  <br />

### 3.1 数据采集与回放
>（1）启动相机roslaunch usb_cam test_img_view.launch 。<br />
>（2）启动中间件roslaunch webots_ros webots_ros_real.launch  。<br />
>（3）运行录制robot_record_episodes.py。<br />
>（4）回放robot_replay_episodes.py --id 5<br />

### 3.2 训练神经网络与推理
>（1）启动相机roslaunch usb_cam test_img_view.launch 。<br />
>（2）启动中间件roslaunch webots_ros webots_ros_real.launch  。<br />
>（3）运行imitate_learning.py训练网络。<br />
>（4）运行imitate_run.py进行推理<br />

## 4.RoboButler具身抓取平台
> 一个低成本、具身抓取平台，包括上肢与移动底盘，采用SCARA结构与直线导轨实现低成本，基于货架电机与舵机可以实现1hour内快速组装，机器人控制器采用Odroid-C4和STM32载板。  <br />
上肢组装资料：https://gvtdwawnc78.feishu.cn/wiki/DaeRwszMQi2pjAkKOGgcaRgZnpc?from=from_copylink<br />

更多问题请加入社区：QQ[RoboHome] 607339413 <br />
资料文档关注：知乎华北舵狗王https://zhuanlan.zhihu.com/p/699074039? <br />
视频资料关注：B站华北舵狗王



