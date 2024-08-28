#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include "sensor_msgs/JointState.h"
#include "tf2_ros/transform_listener.h"
#include <tf/transform_listener.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/Float32.h"
#include <iostream>

#include <sys/types.h>   
#include <sys/socket.h>   
#include <netinet/in.h>   
#include <arpa/inet.h>  
#include <std_srvs/Empty.h>
#include "comm.h"
#include "mems.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

using std::cout;
using std::endl;
using namespace std;

char ros_connect=0;
int  ros_loss_cnt=0;
int  arm_dof=6;
_Webots webots_mem;  
struct webots_msg//UDP实物样机发送的当前指令  相机图像启动usb_cam_all进行采集 仅录制从臂的期望角度与反馈角度
{
    float q_exp[2][7];
    float q[2][7];
    float dq[2][7];
    float tau[2][7];
    float cap_rate_exp[2];
    float cap_rate[2];
    float base_vel[3];
    float epos_exp[2][7];//new
    float epos[2][7];
    int save_data;
} _webots_msg;

void* Thread_UDP_ROBOT(void*)//UDP管理线程 Joint as slave
{
//----------------------UDP init----------------------------
	int sock_fd;  
    ros::NodeHandle n("~");
	sock_fd = socket(AF_INET, SOCK_DGRAM, 0);  
	if(sock_fd < 0)  
	{  
		exit(1);  
	}  
     
   /* 设置address */  
	struct sockaddr_in addr_serv;  
	int len;  
	memset(&addr_serv, 0, sizeof(addr_serv));  
	addr_serv.sin_family = AF_INET;  
	string UDP_IP="192.168.1.202";//------------<
    int SERV_PORT= 3333 ;//OUTTER SDK
    
    n.getParam("ip_addr", UDP_IP);
    n.getParam("udp_port", SERV_PORT);

	addr_serv.sin_addr.s_addr = inet_addr(UDP_IP.c_str());//机器人是客户端 软件主动发送
	addr_serv.sin_port = htons(SERV_PORT);  
	len = sizeof(addr_serv);  

    int recv_num=0,send_num=0;  
    int connect=0,loss_cnt=0;
    char send_buf[500]={0},recv_buf[sizeof(webots_msg)]={0};  
    cout<<"Robot UDP_IP:"<<UDP_IP<<" PORT:"<<SERV_PORT<<" DOF_ARM:"<<arm_dof<<endl;
  
    while (1)
    {
        memcpy(send_buf,&_webots_msg,sizeof(_webots_msg));//send joint command in python script
        send_num = sendto(sock_fd, send_buf, sizeof(send_buf), MSG_DONTWAIT, (struct sockaddr *)&addr_serv, len);
 
        if(send_num < 0)
        {
            perror("Robot sendto error:");
            exit(1);
        }

 	    recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), MSG_DONTWAIT, (struct sockaddr *)&addr_serv, (socklen_t *)&len); 
        if(recv_num >0)
        {
            if(!connect){
                connect=1;
                 printf("ROS::Memory Robot Real-Joint UDP Connected!!\n");
            }
            //printf("recv_num=%d\n",recv_num);
            memcpy(&_webots_msg,recv_buf,sizeof(_webots_msg));//通过UDP接收实际机器人的数据并存储到mem结构体用于发布ROS话题
            //-------------------Right Hands
            for(int i=0;i<arm_dof;i++){
                webots_mem.q_exp[0][i]=_webots_msg.q_exp[0][i];//right
                webots_mem.dq[0][i]=_webots_msg.dq[0][i];
                webots_mem.q[0][i]=_webots_msg.q[0][i];
            }
            webots_mem.cap_rate_exp[0]=_webots_msg.cap_rate_exp[0];  
            webots_mem.cap_rate[0]=_webots_msg.cap_rate[0];
            #if 0
                printf("exp[%.2f]: %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
                webots_mem.cap_rate_exp[0],
                _webots_msg.q_exp[0][0],_webots_msg.q_exp[0][1],_webots_msg.q_exp[0][2]
                ,_webots_msg.q_exp[0][3],_webots_msg.q_exp[0][4],_webots_msg.q_exp[0][5]
                ,_webots_msg.q_exp[0][6]);
                printf("now[%.2f]: %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
                webots_mem.cap_rate[0],
                _webots_msg.q[0][0],_webots_msg.q[0][1],_webots_msg.q[0][2]
                ,_webots_msg.q[0][3],_webots_msg.q[0][4],_webots_msg.q[0][5]
                ,_webots_msg.q[0][6]);
            #endif
            //--------------------Left Hands
            for(int i=0;i<arm_dof;i++){
                webots_mem.q_exp[1][i]=_webots_msg.q_exp[1][i];//left
                webots_mem.dq[1][i]=_webots_msg.dq[1][i];
                webots_mem.q[1][i]=_webots_msg.q[1][i];
            }
            webots_mem.cap_rate_exp[1]=_webots_msg.cap_rate_exp[1];  
            webots_mem.cap_rate[1]=_webots_msg.cap_rate[1];
            //base raw data
            webots_mem.base_vel[0]=_webots_msg.base_vel[0];//base
            webots_mem.base_vel[1]=_webots_msg.base_vel[1];
            webots_mem.base_vel[2]=_webots_msg.base_vel[2];

        }
        usleep(10*1000);
    }
    return 0;
}
 
void* Thread_ROS(void*)//内存管理线程 将mem中的暂存数据发布ROS话题并由模仿学习节点订阅
{
    ros::NodeHandle n("~");
    ros::Publisher joint_pub_r = n.advertise<sensor_msgs::JointState>("/puppet_right/joint_states", 1);
    ros::Publisher joint_pub_l = n.advertise<sensor_msgs::JointState>("/puppet_left/joint_states", 1);
    ros::Publisher joint_pub_base = n.advertise<sensor_msgs::JointState>("/puppet_base/joint_states", 1);

    ros::Publisher joint_pub_r_exp = n.advertise<sensor_msgs::JointState>("/puppet_right_exp/joint_states", 1);
    ros::Publisher joint_pub_l_exp = n.advertise<sensor_msgs::JointState>("/puppet_left_exp/joint_states", 1);

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 10);

    tf2_ros::StaticTransformBroadcaster tf_b;
    geometry_msgs::TransformStamped ts;

    int cout=1;
    float pos_n[3]={0};
    float vel_b[3]={0};
    float vel_n[3]={0};
	ros::Rate loop_rate(50);
	while (ros::ok())
	{  	
		ros::spinOnce();               // check for incoming messages

  /******************* 发布消息 ***************/
        string Name_Joint[7]={"X","Y","Z","ROL","PIT","YAW","Link6"};//------------<
        string Name_Cap="Cap";//------------<
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();//right
        joint_state.name.resize(arm_dof+1);//joint+1cap
        joint_state.position.resize(arm_dof+1);
        joint_state.velocity.resize(arm_dof+1);
        joint_state.effort.resize(arm_dof+1);
        for(int i=0;i<arm_dof;i++){//right
            joint_state.name[i] =Name_Joint[i];
            joint_state.position[i] = webots_mem.epos[0][i];       
            joint_state.velocity[i] = webots_mem.dq[0][i];
            joint_state.effort[i] = webots_mem.tau[0][i];  
        }         
        joint_state.name[arm_dof] ="CAP";
        joint_state.position[arm_dof] = webots_mem.cap_rate[0];
        joint_state.velocity[arm_dof] = 0;
        joint_state.effort[arm_dof] = 0;    
        joint_pub_r.publish(joint_state);
 

        joint_state.header.stamp = ros::Time::now();//left
        joint_state.name.resize(arm_dof+1);//joint+1cap
        joint_state.position.resize(arm_dof+1);
        joint_state.velocity.resize(arm_dof+1);
        joint_state.effort.resize(arm_dof+1);
        for(int i=0;i<arm_dof;i++){//left
            joint_state.name[i] =Name_Joint[i];
            joint_state.position[i] = webots_mem.epos[1][i];       
            joint_state.velocity[i] = webots_mem.dq[1][i];
            joint_state.effort[i] = webots_mem.tau[1][i];  
        }           
        joint_state.name[arm_dof] ="CAP";
        joint_state.position[arm_dof] = webots_mem.cap_rate[1];
        joint_state.velocity[arm_dof] = 0;
        joint_state.effort[arm_dof] = 0;    
        joint_pub_l.publish(joint_state);


        joint_state.header.stamp = ros::Time::now();//base机体速度
        joint_state.name.resize(2);
        joint_state.position.resize(2);
        joint_state.velocity.resize(2);
        joint_state.effort.resize(2);
        joint_state.name[0] ="X";
        joint_state.position[0] = 0;       
        joint_state.velocity[0] = webots_mem.base_vel[0];
        joint_state.effort[0] = 0;       
        joint_state.name[1] ="Yaw";
        joint_state.position[1] = 0;
        joint_state.velocity[1] = webots_mem.base_vel[2];
        joint_state.effort[1] = 0;    
        joint_pub_base.publish(joint_state);

 /*******************Command 发布消息 ***************/
 
        joint_state.header.stamp = ros::Time::now();//right
        joint_state.name.resize(arm_dof+1);//joint+1cap
        joint_state.position.resize(arm_dof+1);
        joint_state.velocity.resize(arm_dof+1);
        joint_state.effort.resize(arm_dof+1);
        for(int i=0;i<arm_dof;i++){//left
            joint_state.name[i] =Name_Joint[i];
            joint_state.position[i] = webots_mem.epos_exp[0][i];       
            joint_state.velocity[i] = webots_mem.dq[0][i];
            joint_state.effort[i] = webots_mem.tau[0][i];  
        }         
        joint_state.name[arm_dof] ="CAP";
        joint_state.position[arm_dof] = webots_mem.cap_rate_exp[0];
        joint_state.velocity[arm_dof] = 0;
        joint_state.effort[arm_dof] = 0;    
        joint_pub_r_exp.publish(joint_state);
 

        joint_state.header.stamp = ros::Time::now();//left
        joint_state.name.resize(arm_dof+1);//joint+1cap
        joint_state.position.resize(arm_dof+1);
        joint_state.velocity.resize(arm_dof+1);
        joint_state.effort.resize(arm_dof+1);
        for(int i=0;i<arm_dof;i++){//left
            joint_state.name[i] =Name_Joint[i];
            joint_state.position[i] = webots_mem.epos_exp[1][i];       
            joint_state.velocity[i] = webots_mem.dq[1][i];
            joint_state.effort[i] = webots_mem.tau[1][i];  
        }         
        joint_state.name[arm_dof] ="CAP";
        joint_state.position[arm_dof] = webots_mem.cap_rate_exp[1];
        joint_state.velocity[arm_dof] = 0;
        joint_state.effort[arm_dof] = 0;    
        joint_pub_l_exp.publish(joint_state);


        //-------------Odom里程计用于发布tf
        float dt=1/50.0;
        float v_scale=1.6*2;
        vel_b[0]=webots_mem.base_vel[0]*v_scale;
        vel_b[1]=webots_mem.base_vel[1]*v_scale;
        float yaw=webots_mem.att[2]/57.3;
        vel_n[0]=cos(yaw)*vel_b[0]-sin(yaw)*vel_b[1];
        vel_n[1]=cos(yaw)*vel_b[1]+sin(yaw)*vel_b[0];

        pos_n[0]+=vel_n[0]*dt;
        pos_n[1]+=vel_n[1]*dt;
        tf2::Quaternion qtn;
        qtn.setRPY(0,0,yaw);

        nav_msgs::Odometry odom_temp;
        
        odom_temp.header.frame_id="odom";
        odom_temp.child_frame_id="lidar_link";
        odom_temp.header.stamp=ros::Time::now();
        odom_temp.pose.pose.position.x=pos_n[0];
        odom_temp.pose.pose.position.y=pos_n[1];
        odom_temp.pose.pose.position.z=0.1;

        odom_temp.pose.pose.orientation.x=qtn.getX();
        odom_temp.pose.pose.orientation.y=qtn.getY();
        odom_temp.pose.pose.orientation.z=qtn.getZ();
        odom_temp.pose.pose.orientation.w=qtn.getW();

        odom_pub.publish(odom_temp);

        ts.header.seq=100;
        ts.header.frame_id="odom";
        ts.child_frame_id="lidar_link";
        ts.transform.translation.x=pos_n[0];
        ts.transform.translation.y=pos_n[1];
        ts.transform.translation.z=0.1;

        ts.transform.rotation.x=qtn.getX();
        ts.transform.rotation.y=qtn.getY();
        ts.transform.rotation.z=qtn.getZ();
        ts.transform.rotation.w=qtn.getW();

        tf_b.sendTransform(ts);
        
		loop_rate.sleep();
	}
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "webots_ros_real_ee");
	pthread_t tida, tidb,tidc;
    pthread_create(&tida, NULL, Thread_ROS, NULL);
    pthread_create(&tidc, NULL, Thread_UDP_ROBOT, NULL);
    pthread_join(tida, NULL);
    pthread_join(tidb, NULL);
    pthread_join(tidc, NULL);
}
