#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <gpd_ros/CloudIndexed.h>
#include <gpd_ros/CloudSamples.h>
#include <gpd_ros/GraspConfigList.h>


#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_msgs/Bool.h>

#include <dynamic_reconfigure/server.h>
#include <robot_sim/gpdConfig.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

#include <sys/types.h>   
#include <sys/socket.h>   
#include <netinet/in.h>   
#include <arpa/inet.h>  
#include <std_srvs/Empty.h>

#define math_pi 3.14159265

bool tf_flag = 0;
bool at_home = 0;

using namespace std;

#define max_velocity 0.5
#define max_acceleration 0.2

bool add_pi = false;
bool good_grasp = true;
bool bad_grasp = false;


struct remote_msg_end_grasp
{
   float arm_pos_exp_l_app[3];
   float arm_att_exp_l_app[3];

   float arm_pos_exp_l[3];
   float arm_att_exp_l[3];

   float arm_pos_exp_r_app[3];
   float arm_att_exp_r_app[3];

   float arm_pos_exp_r[3];
   float arm_att_exp_r[3];
} remote_msg_end_grasp;


float grasp_pos_app[6]={0};
float grasp_pos[6]={0};
int grasp_action=0;
int grasp_action_app=0;

void gpd_dynamic_callback(robot_sim::gpdConfig &config)
{
	add_pi = config.add_pi;
	good_grasp = config.good_grasp;
	bad_grasp = config.bad_grasp;
	std::cout<<"add_pi:"<<add_pi<<good_grasp<<bad_grasp<<std::endl;
}
 
string fcloud_rame_id = "camera_depth_optical_frame";//doghome
sensor_msgs::PointCloud2 seg_reslut; //分割后的点云转成ROS格式																  //收到点云后的标志位

gpd_ros::GraspConfig best_grasp; //最好的抓取点的msg
bool receive_grasp_flag = 0;	 //收到grasp消息

void graspCB(const gpd_ros::GraspConfigList &msg)
{
	cout << "receive grasp\n";
	if (!receive_grasp_flag&& msg.grasps.size())
	{
		best_grasp = msg.grasps[0];
		receive_grasp_flag = 1;
		printf("Best=%f %f %f\n",best_grasp.position.x, best_grasp.position.y, best_grasp.position.z);
	}
}


void printTf_grasp_app(tf::Transform tf) {
	
	//------------------------------------
	geometry_msgs::PoseStamped position_3d;
	position_3d.pose.position.x = tf.getOrigin().getX(); 
	position_3d.pose.position.y = tf.getOrigin().getY(); 
	position_3d.pose.position.z = tf.getOrigin().getZ();//odom_3d->pose.pose.position.z;
 
	position_3d.pose.orientation.x = tf.getRotation().x();
	position_3d.pose.orientation.y = tf.getRotation().y();
	position_3d.pose.orientation.z = tf.getRotation().z();
	position_3d.pose.orientation.w = tf.getRotation().w();

	position_3d.header.stamp = ros::Time::now();
	position_3d.header.frame_id = "map";

	tf::Quaternion quat;
	tf::quaternionMsgToTF(position_3d.pose.orientation, quat);

	double roll, pitch, yaw;//定义存储r\p\y的容器
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行
	roll*=57.3;pitch*=57.3;yaw*=57.3;
	//cout<<"origin:"<<roll<<" "<<pitch<<" "<<yaw<<endl;

	if(roll<-45)
		roll=90+roll;
	if(roll>45)
		roll=90-roll;

	if(fabs(yaw)<90){
		if(yaw>45)
			yaw=90-yaw;
		if(yaw<-45)
			yaw=90+yaw;
	}else{
		if(yaw>90)
			yaw=180-yaw;
		if(yaw<-90)
			yaw=180+yaw;
	}
	//cout<<position_3d<<endl;
	//cout<<"fix:"<<roll<<" "<<pitch<<" "<<yaw<<endl;

	//convert base
	float x=position_3d.pose.position.x;
	float y=position_3d.pose.position.y;
	float z=position_3d.pose.position.z;

	// cout<<"convert_h_app:"<<x<<" "<<y<<" "<<z<<endl;
	// cout<<"attitude_app:"<<roll<<" "<<pitch<<" "<<yaw<<endl;
	if(fabs(pitch)>45&&0){
		cout<<"pitch too big!!!!"<<endl;
	}else if(fabs(roll)>45&&0){
		cout<<"roll too big!!!!"<<endl;
	}else if(fabs(yaw)>90){
		cout<<"yaw too big!!!!"<<endl;
	}else{
		grasp_action_app=1;
        //receive_grasp_flag=1;
		grasp_pos_app[0]=x;
		grasp_pos_app[1]=y;
		grasp_pos_app[2]=z;
		grasp_pos_app[3]=roll;
		grasp_pos_app[4]=pitch;
		grasp_pos_app[5]=yaw;
	}

}


void printTf_grasp(tf::Transform tf) {
	
	//------------------------------------
	geometry_msgs::PoseStamped position_3d;
	position_3d.pose.position.x = tf.getOrigin().getX(); 
	position_3d.pose.position.y = tf.getOrigin().getY(); 
	position_3d.pose.position.z = tf.getOrigin().getZ();//odom_3d->pose.pose.position.z;
 
	position_3d.pose.orientation.x = tf.getRotation().x();
	position_3d.pose.orientation.y = tf.getRotation().y();
	position_3d.pose.orientation.z = tf.getRotation().z();
	position_3d.pose.orientation.w = tf.getRotation().w();

	position_3d.header.stamp = ros::Time::now();
	position_3d.header.frame_id = "map";

	tf::Quaternion quat;
	tf::quaternionMsgToTF(position_3d.pose.orientation, quat);

	double roll, pitch, yaw;//定义存储r\p\y的容器
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行
	roll*=57.3;pitch*=57.3;yaw*=57.3;
	//cout<<"origin:"<<roll<<" "<<pitch<<" "<<yaw<<endl;

	if(roll<-45)
		roll=90+roll;
	if(roll>45)
		roll=90-roll;

	if(fabs(yaw)<90){
		if(yaw>45)
			yaw=90-yaw;
		if(yaw<-45)
			yaw=90+yaw;
	}else{
		if(yaw>90)
			yaw=180-yaw;
		if(yaw<-90)
			yaw=180+yaw;
	}
	//cout<<position_3d<<endl;
	//cout<<"fix:"<<roll<<" "<<pitch<<" "<<yaw<<endl;

	//convert base
	float x=position_3d.pose.position.x;
	float y=position_3d.pose.position.y;
	float z=position_3d.pose.position.z;

	// cout<<"convert_h:"<<x<<" "<<y<<" "<<z<<endl;
	// cout<<"attitude:"<<roll<<" "<<pitch<<" "<<yaw<<endl;
	if(fabs(pitch)>45&&0){
		cout<<"pitch too big!!!!"<<endl;
	}else if(fabs(roll)>45&&0){
		cout<<"roll too big!!!!"<<endl;
	}else if(fabs(yaw)>90){
		cout<<"yaw too big!!!!"<<endl;
	}else{
		grasp_action=1;
        //receive_grasp_flag=1;
		grasp_pos[0]=x;
		grasp_pos[1]=y;
		grasp_pos[2]=z;
		grasp_pos[3]=roll;
		grasp_pos[4]=pitch;
		grasp_pos[5]=yaw;
	}
}


 void transformPoint_app(const tf::TransformListener &listener) {//监听TF   
    	try{
			tf::StampedTransform transform;
				//得到坐标odom和坐标base_link之间的关系
			listener.waitForTransform("arm_h", "grasp_tf_app", ros::Time(0), ros::Duration(0.005));
			listener.lookupTransform("arm_h", "grasp_tf_app",
									ros::Time(0), transform);
			printTf_grasp_app(transform);							
		}
		catch (tf::TransformException &ex) {
			//ROS_ERROR("%s",ex.what());
		}
}

 void transformPoint(const tf::TransformListener &listener) {//监听TF   
    	try{
			tf::StampedTransform transform;
				//得到坐标odom和坐标base_link之间的关系
			listener.waitForTransform("arm_h", "grasp_tf", ros::Time(0), ros::Duration(0.005));
			listener.lookupTransform("arm_h", "grasp_tf",
									ros::Time(0), transform);
			printTf_grasp(transform);							
		}
		catch (tf::TransformException &ex) {
			//ROS_ERROR("%s",ex.what());
		}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "tinymal_gdp_grasp");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10); //设置发送数据的频率

	ros::Subscriber grasps_sub_ = nh.subscribe("/detect_grasps/clustered_grasps", 10, graspCB); //GPD发布的grasp
	printf("GDP grasp server online!\n");
	dynamic_reconfigure::Server<robot_sim::gpdConfig> server;
	dynamic_reconfigure::Server<robot_sim::gpdConfig>::CallbackType callback;

	callback = boost::bind(&gpd_dynamic_callback, _1);
	server.setCallback(callback);

	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	tf::StampedTransform world2cam;
	bool tf_ok=false;
	//等待并读取桌面标签坐标系和Kinect坐标系之间的变换关系
	tf::TransformListener listener;

	while(!tf_ok){
		try{
			listener.lookupTransform("/camera_depth_optical_frame", "/camera_depth_optical_frame",
									ros::Time(0), world2cam);
			tf_ok=true;
		}
		catch (tf::TransformException &ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(0.1).sleep();
			continue;
		}
	}
	printf("Get TF world to camera!\n");
	/* 最好的那个grasp的pose变成ur爪子的坐标系有两个旋转 */
	tf::Transform grasp2urPose;
	tf::Quaternion grasp2urPose_Q;
	grasp2urPose_Q.setRPY(math_pi / 2, 0, -math_pi / 2);
	grasp2urPose.setRotation(grasp2urPose_Q);
	grasp2urPose.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

	int pub_tar=0;
	tf::TransformBroadcaster br,br1;
 
	tf::TransformListener listener_grasp_app(ros::Duration(5));//缓冲5s防止报错
	ros::Timer timer2=nh.createTimer(ros::Duration(0.01),boost::bind(&transformPoint_app, boost::ref(listener_grasp_app)));//0.01s 100Hz调用一次监听器

    tf::TransformListener listener_grasp(ros::Duration(5));//缓冲5s防止报错
	ros::Timer timer3=nh.createTimer(ros::Duration(0.01),boost::bind(&transformPoint, boost::ref(listener_grasp)));//0.01s 100Hz调用一次监听器


 	int sock_fdt;   
	sock_fdt = socket(AF_INET, SOCK_DGRAM, 0);  
	if(sock_fdt < 0)  
	{  
		ROS_ERROR("UPD TX ROS SDK Port Open Fail!!"); 
		exit(1);  
	} 
	struct sockaddr_in addr_servt;  
	int lent=0;  
	memset(&addr_servt, 0, sizeof(addr_servt));  
	addr_servt.sin_family = AF_INET;  
	string UDP_IP_="192.168.1.187";//doghome
	addr_servt.sin_addr.s_addr = inet_addr(UDP_IP_.c_str());  
	addr_servt.sin_port = htons(3334);  
	lent = sizeof(addr_servt);

	int recv_num=0,send_num=0;  
	char send_buf[500]={0},recv_buf[500]={0};  

	while (ros::ok())
	{
	 	if (receive_grasp_flag)
		{
			receive_grasp_flag = 0;
			good_grasp = 0;
			tf::Matrix3x3 orientation(
				best_grasp.approach.x, best_grasp.binormal.x, best_grasp.axis.x,
				best_grasp.approach.y, best_grasp.binormal.y, best_grasp.axis.y,
				best_grasp.approach.z, best_grasp.binormal.z, best_grasp.axis.z);
			double roll, pitch, yaw;
			tf::Vector3 Bottom(best_grasp.position.x, best_grasp.position.y, best_grasp.position.z);
			tf::Transform cam2grasp;
			cam2grasp.setOrigin(Bottom);
			tf::Quaternion cam2grasp_pose_Q;
			orientation.getRotation(cam2grasp_pose_Q); //将旋转矩阵转成四元数
			cam2grasp.setRotation(cam2grasp_pose_Q);   //设置cam2grasp的旋转

			tf::Transform cam2urPose, world2urPose;
			cam2urPose.mult(cam2grasp, grasp2urPose); //cam2grasp X grasp2urPose = cam2urPose;
			if (add_pi)
			{
				add_pi = 0;
				tf::Transform yaw_pi_rotation;
				tf::Quaternion yaw_pi_rotation_Q;
				yaw_pi_rotation_Q.setRPY(0, 0, math_pi);
				yaw_pi_rotation.setRotation(yaw_pi_rotation_Q);
				yaw_pi_rotation.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
				cam2urPose.mult(cam2urPose, yaw_pi_rotation);
			}
			world2urPose.mult(world2cam, cam2urPose); //world2cam X cam2urPose = world2urPose;
			tf::Matrix3x3(world2urPose.getRotation()).getRPY(roll, pitch, yaw);
			cout << roll * 180 / math_pi << "\t" << pitch * 180 / math_pi << "\t" << yaw * 180 / math_pi << "\n";
			if (yaw * 180 / math_pi > -90 && yaw * 180 / math_pi < 90)
			{
				std::cout << "dont need to add pi\n";
			}
			else
			{
				tf::Transform yaw_pi_rotation;
				tf::Quaternion yaw_pi_rotation_Q;
				yaw_pi_rotation_Q.setRPY(0, 0, math_pi);
				yaw_pi_rotation.setRotation(yaw_pi_rotation_Q);
				yaw_pi_rotation.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
				world2urPose.mult(world2urPose, yaw_pi_rotation);
			}

			tf::Transform above_10cm_of_best, add_10cm_to_Z;
			tf::Quaternion add_10cm_to_Z_Q;
			#if 0
				add_10cm_to_Z_Q.setRPY(0, 0, 0);
				add_10cm_to_Z.setRotation(add_10cm_to_Z_Q);
				add_10cm_to_Z.setOrigin(tf::Vector3(0.0, 0.0, 0.10));
				above_10cm_of_best.mult(world2urPose, add_10cm_to_Z);

				//广播world与海龟坐标系之间的tf数据  （数据，时间，父节点，子节点）
				br.sendTransform(tf::StampedTransform(above_10cm_of_best,ros::Time::now(),"camera_link","grasp_tf"));
			#else
				if (add_pi)
				{
					add_pi = 0;
					tf::Transform yaw_pi_rotation;
					tf::Quaternion yaw_pi_rotation_Q;
					yaw_pi_rotation_Q.setRPY(0, 0, math_pi);
					yaw_pi_rotation.setRotation(yaw_pi_rotation_Q);
					yaw_pi_rotation.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
					cam2grasp.mult(cam2grasp, yaw_pi_rotation);
				}

				add_10cm_to_Z_Q.setRPY(0, 0, 0);
				add_10cm_to_Z.setRotation(add_10cm_to_Z_Q);
				add_10cm_to_Z.setOrigin(tf::Vector3(-0.03, 0.0, 0.0));
				above_10cm_of_best.mult(cam2grasp, add_10cm_to_Z);

				//广播world与海龟坐标系之间的tf数据  （数据，时间，父节点，子节点）
				br.sendTransform(tf::StampedTransform(above_10cm_of_best,ros::Time::now(),"arm_h","grasp_tf_app"));

				br1.sendTransform(tf::StampedTransform(cam2grasp,ros::Time::now(),"arm_h","grasp_tf"));
			#endif
 
 		static int cnt_p=0;
		if(grasp_action&&grasp_action_app){	

            remote_msg_end_grasp.arm_pos_exp_r_app[0]=grasp_pos_app[0];
			remote_msg_end_grasp.arm_pos_exp_r_app[1]=grasp_pos_app[1];
			remote_msg_end_grasp.arm_pos_exp_r_app[2]=grasp_pos_app[2];
			remote_msg_end_grasp.arm_att_exp_r_app[0]=grasp_pos_app[3];
			remote_msg_end_grasp.arm_att_exp_r_app[1]=grasp_pos_app[4];
			remote_msg_end_grasp.arm_att_exp_r_app[2]=grasp_pos_app[5];


			remote_msg_end_grasp.arm_pos_exp_r[0]=grasp_pos[0];
			remote_msg_end_grasp.arm_pos_exp_r[1]=grasp_pos[1];
			remote_msg_end_grasp.arm_pos_exp_r[2]=grasp_pos[2];
			remote_msg_end_grasp.arm_att_exp_r[0]=grasp_pos[3];
			remote_msg_end_grasp.arm_att_exp_r[1]=grasp_pos[4];
			remote_msg_end_grasp.arm_att_exp_r[2]=grasp_pos[5];

			memcpy(send_buf,&remote_msg_end_grasp,sizeof(remote_msg_end_grasp));

			int send_num = sendto(sock_fdt, send_buf, sizeof(remote_msg_end_grasp), MSG_DONTWAIT, (struct sockaddr *)&addr_servt,lent);
			if(cnt_p++>5){cnt_p=0;
				cout<<"Grasp::send_num:"<<send_num<<endl;
				cout<<remote_msg_end_grasp.arm_pos_exp_r_app[2]<<endl;
				cout<<remote_msg_end_grasp.arm_pos_exp_r[2]<<endl;
			}
			grasp_action_app=grasp_action=0;

		}
		
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
    close(sock_fdt);
	return 0;
}
