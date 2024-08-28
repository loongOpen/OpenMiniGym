#include <Eigen/Dense>
#include <qpOASES.hpp>
#include "optimaize.h"
#include "locomotion_header.h"
#include "gait_math.h"
#include "adrc.h"
#include "convexMPC_interface.h"
#include "common_types.h"
#include "SolverMPC.h"
#include "cppTypes.h"
#include "PositionVelocityEstimator.h"
#include <string.h>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "spi_node.h"
#include "sys_time.h"
#include <sys/shm.h>

using namespace Eigen;
using namespace std;
using namespace qpOASES;

struct remote_msg
{
   char mode;
   float arm_pos_exp[2][3];
   float arm_att_exp[2][3];
   float cap_rate_exp[2];
   float base_vel_exp[3];
   char key[5];
} _remote_msg;

struct remote_msg_end
{
   float base_vel[3];
   float arm_pos_exp_l[3];
   float arm_att_exp_l[3];
   float cap_l;
   float arm_pos_exp_r[3];
   float arm_att_exp_r[3];
   float cap_r;
} _remote_msg_end;

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

struct remote_msg_record
{
   float mode;
   float record_rate;
   float episode_idx;
} _remote_msg_record;

struct remote_msg_imitate
{
   float mode;
   float record_rate;
   float episode_idx;
   float temp;
} _remote_msg_imitate;

struct remote_msg_joint
{
   float base_vel[3];
   float q_exp_l[6];
   float cap_l;
   float q_exp_r[6];
   float cap_r;

   float replay_rate;
} _remote_msg_joint;

struct robot_msg{
    float q_exp[2][7];
    float q[2][7];
    float dq[2][7];
    float tau[2][7];
    float cap_rate_exp[2];
    float cap_rate[2];
    float base_vel[3];

    int save_data;
} _robot_msg;

struct robot_msg_imt{
    float start_record;
    float reach_waypoint;
} _robot_msg_imit;

struct robot_record_msg
{
   float start_record;
   float reach_waypoint;
} _robot_record_msg;

struct remote_msg_waypoint
{
   float mode;
   float waypoint;
} _remote_msg_waypoint;

//-----------------------
struct remote_msg_skill
{
    int skill_id;
    int action_msg;
    END_POS pose_msg1;
    END_POS att_msg1;
    END_POS pose_msg2;
    END_POS att_msg2;
} _remote_msg_skill;

void Perror(const char *s)
{
    perror(s);
    exit(EXIT_FAILURE);
}

static void setnonblocking(int sockfd) {
    int flag = fcntl(sockfd, F_GETFL, 0);
    if (flag < 0) {
        Perror("fcntl F_GETFL fail");
        return;
    }
    if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0) {
        Perror("fcntl F_SETFL fail");
    }
}

void* Thread_UDP_SDK(void*)//SDK UDP通讯线程  作为as 服务器  feed back to ros
{
    static int cnt = 0;
    float sys_dt = 0;
    float loss_cnt_sdk=0;
    int flag = 0;
    float timer[5]={0};
    int i=0;

    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        perror("socket");
        exit(1);
    }
   setnonblocking(sock_fd);//非阻塞
   struct sockaddr_in addr_serv;
   int len;
   memset(&addr_serv, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
   addr_serv.sin_family = AF_INET;//使用IPV4地址
   addr_serv.sin_port = htons(3333);//端口
   addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
   len = sizeof(addr_serv);

   if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
   {
     perror("bind error:");
     exit(1);
   }

    int recv_num=0,send_num=0;
    int recording_flag=0;
    char send_buf[500]={0},recv_buf[500]={0};
    struct sockaddr_in addr_client;
    int cnt_p=0;
    int reg_wheel=0;
    printf("Thread UDP imitate ROS-FB\n");
    while (1)
    {
        //读取客户端 读取操控指令

        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        if(recv_num >0)
        {
            //------------------------发送机器人状态---------------------------------
            _robot_msg.q[0][0]=robotwb.arm_base_height;//0->right 1->left
            _robot_msg.q[0][1]=robotwb.arm_q[0][0];
            _robot_msg.q[0][2]=robotwb.arm_q[0][1];
            _robot_msg.q[0][3]=robotwb.arm_q[0][2];
            _robot_msg.q[0][4]=robotwb.arm_q[0][3];
            _robot_msg.q[0][5]=robotwb.arm_q[0][4];
            _robot_msg.q_exp[0][0]=robotwb.arm_base_height_exp;//0->right 1->left
            _robot_msg.q_exp[0][1]=robotwb.arm_q_exp[0][0]/57.3;
            _robot_msg.q_exp[0][2]=robotwb.arm_q_exp[0][1]/57.3;
            _robot_msg.q_exp[0][3]=robotwb.arm_q_exp[0][2]/57.3;
            _robot_msg.q_exp[0][4]=robotwb.arm_q_exp[0][3]/57.3;
            _robot_msg.q_exp[0][5]=robotwb.arm_q_exp[0][4]/57.3;

            _robot_msg.q[1][0]=robotwb.arm_base_height;
            _robot_msg.q[1][1]=robotwb.arm_q[0][0];
            _robot_msg.q[1][2]=robotwb.arm_q[0][1];
            _robot_msg.q[1][3]=robotwb.arm_q[0][2];
            _robot_msg.q[1][4]=robotwb.arm_q[0][3];
            _robot_msg.q[1][5]=robotwb.arm_q[0][4];
            _robot_msg.q_exp[1][0]=robotwb.arm_base_height_exp;//0->right 1->left
            _robot_msg.q_exp[1][1]=robotwb.arm_q_exp[1][0]/57.3;
            _robot_msg.q_exp[1][2]=robotwb.arm_q_exp[1][1]/57.3;
            _robot_msg.q_exp[1][3]=robotwb.arm_q_exp[1][2]/57.3;
            _robot_msg.q_exp[1][4]=robotwb.arm_q_exp[1][3]/57.3;
            _robot_msg.q_exp[1][5]=robotwb.arm_q_exp[1][4]/57.3;

            _robot_msg.cap_rate[0]=robotwb.cap_set;
            _robot_msg.cap_rate[1]=robotwb.cap_set;
            _robot_msg.cap_rate_exp[0]=robotwb.cap_set;
            _robot_msg.cap_rate_exp[1]=robotwb.cap_set;

            _robot_msg.base_vel[0]=robotwb.base_vel_b_w_fushion.x;
            _robot_msg.base_vel[1]=robotwb.base_vel_b_w_fushion.y;
            _robot_msg.base_vel[2]=robotwb.base_rate_fushion;
            memcpy(send_buf,&_robot_msg,sizeof(_robot_msg));
            send_num = sendto(sock_fd, send_buf, sizeof(_robot_msg), MSG_DONTWAIT, (struct sockaddr *)&addr_client, len);
//            printf("usb_send_cnt=%d\n",send_num);
            if(send_num < 0)
            {
                perror("OCU sendto error:");
                exit(1);
            }
        }
        usleep(5*1000);//200Hz
    }
    close(sock_fd);
    return 0;
}

void* Thread_UDP_SDK_IMIT(void*)//SDK UDP通讯线程  作为as 服务器 for get network commond
{
    static int cnt = 0;
    float sys_dt = 0;
    float loss_cnt_sdk=0;
    int flag = 0;
    float timer[5]={0};
    int i=0;

    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        perror("socket");
        exit(1);
    }
   setnonblocking(sock_fd);//非阻塞
   struct sockaddr_in addr_serv;
   int len;
   memset(&addr_serv, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
   addr_serv.sin_family = AF_INET;//使用IPV4地址
   addr_serv.sin_port = htons(3334);//端口<-----------------Python
   addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
   len = sizeof(addr_serv);

   if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
   {
     perror("bind error:");
     exit(1);
   }

    int recv_num=0,send_num=0;
    int recording_flag=0;
    char send_buf[500]={0},recv_buf[500]={0};
    struct sockaddr_in addr_client;
    int cnt_p=0;
    int reg_wheel=0;
    printf("Thread UDP imitate\n");
    while (1)
    {
        //读取客户端 读取操控指令
        loss_cnt_sdk+=0.005;
        _robot_record_msg.start_record=robotwb.start_record;
        _robot_record_msg.reach_waypoint=robotwb.reach_waypoint;
        robotwb.record_mode=_remote_msg_record.mode;

        if((loss_cnt_sdk>1&&(robotwb.arm_control_mode==98||robotwb.arm_control_mode==99||recording_flag==1))||(ocu.key_ud==-1&&robotwb.arm_control_mode!=0)){
            robotwb.arm_control_mode=ARM_M_H;//sdk
            robotwb.wheel_control_mode=0;//sdk
#if ONLY_ARM
            robotwb.head_control_mode=HEAD_LOOK_B;
#else
            robotwb.head_control_mode=HEAD_FREE;
#endif
            robotwb.head_att_exp[0]=robotwb.head_att_exp[1]=0;
            robotwb.arm_epos_b_exp.z=0;
            robotwb.arm_epos_e_exp.z=0;
            robotwb.arm_epos_n_exp.z=0;
            recording_flag=0;
            robotwb.start_record=robotwb.record_mode=0;
            robotwb.grasp_action=0;
            printf("Quit SDK mode!\n");
        }

        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        if(recv_num >0)
        {
            //printf("recv_num=%d\n",recv_num);
            if(recv_num==12){//recording
                memcpy(&_remote_msg_record,recv_buf,sizeof(_remote_msg_record));
                loss_cnt_sdk=0;
                if(recording_flag==0&&_remote_msg_record.mode==1&&_robot_record_msg.start_record==1)
                {
                    recording_flag=1;
                    printf("SDK Start Recording -------------------Hg5.[%.0f]!\n",_remote_msg_record.episode_idx);
                }

                if(recording_flag&&(_remote_msg_record.record_rate>0.02||robotwb.start_record==0))
                {
                    if(cnt_p++>25){cnt_p=0;
                    printf("SDK Comman Recording:%.1f%\n",_remote_msg_record.record_rate*100);
                    }
                    if(_remote_msg_record.record_rate<0.01||robotwb.start_record==0){
                        recording_flag=0;
                        _remote_msg_record.record_rate=0;
                        _robot_record_msg.start_record=robotwb.start_record=0;
                        printf("SDK Finish Recording!\n");
                    }
                }
                memcpy(send_buf,&_robot_record_msg,sizeof(_robot_record_msg));
                send_num = sendto(sock_fd, send_buf, sizeof(_robot_record_msg), MSG_DONTWAIT, (struct sockaddr *)&addr_client, len);
            }
            else if(recv_num==135){//epos replay----------------------------[1]
                loss_cnt_sdk=0;
                memcpy(&_remote_msg_end,recv_buf,sizeof(_remote_msg_end));
                if(robotwb.arm_control_mode!=98)
                {
                    robotwb.arm_control_mode=98;//sdk
                    robotwb.wheel_control_mode=98;//sdk
                    robotwb.head_control_mode=HEAD_LOOK_B;
                    printf("SDK epos mode enable!\n");
                }
                robotwb.arm_epos_h.x=_remote_msg_end.arm_pos_exp_r[0];
                robotwb.arm_epos_h.y=_remote_msg_end.arm_pos_exp_r[1];
                robotwb.arm_epos_h.z=_remote_msg_end.arm_pos_exp_r[2];
                robotwb.arm_att_b.x=_remote_msg_end.arm_att_exp_r[0];
                robotwb.arm_att_b.y=_remote_msg_end.arm_att_exp_r[1];
                robotwb.arm_att_b.z=_remote_msg_end.arm_att_exp_r[2];

                robotwb.cap_set=_remote_msg_end.cap_r;
                robotwb.base_vel_sdk[0]=_remote_msg_end.base_vel[0];
                robotwb.base_vel_sdk[2]=_remote_msg_end.base_vel[2];
                //printf("[%d]%f %f %f %f\n",rx_cnt++,_remote_msg.test[0],_remote_msg.test[1],_remote_msg.test[2],_remote_msg.test[3]);
                //
            }  
            else  if(recv_num==72){//joint replay---------------------------[2]
                memcpy(&_remote_msg_joint,recv_buf,sizeof(_remote_msg_joint));
                loss_cnt_sdk=0;
                if(robotwb.arm_control_mode!=99)
                {
                    robotwb.arm_control_mode=99;//sdk
                    robotwb.wheel_control_mode=99;//sdk
                    robotwb.head_control_mode=HEAD_LOOK_B;//HEAD_RECODE;//HEAD_LOOK_B;
                    printf("SDK joint mode enable!\n");
                }

                robotwb.arm_q_exp_sdk[0][0]=_remote_msg_joint.q_exp_r[0];//base_z
                robotwb.arm_q_exp_sdk[0][1]=_remote_msg_joint.q_exp_r[1]*57.3;
                robotwb.arm_q_exp_sdk[0][2]=_remote_msg_joint.q_exp_r[2]*57.3;
                robotwb.arm_q_exp_sdk[0][3]=_remote_msg_joint.q_exp_r[3]*57.3;
                robotwb.arm_q_exp_sdk[0][4]=_remote_msg_joint.q_exp_r[4]*57.3;
                robotwb.arm_q_exp_sdk[0][5]=_remote_msg_joint.q_exp_r[5]*57.3;
#if 0
                printf("%.3f %.3f %.3f %.3f %.3f %.3f \n",robotwb.arm_q_exp_sdk[0][0],
                        robotwb.arm_q_exp_sdk[0][1],
                        robotwb.arm_q_exp_sdk[0][2],
                        robotwb.arm_q_exp_sdk[0][3],
                        robotwb.arm_q_exp_sdk[0][4],
                        robotwb.arm_q_exp_sdk[0][5]);
#endif

                robotwb.cap_set_sdk[0]=_remote_msg_joint.cap_r;
                robotwb.base_vel_sdk[0]=_remote_msg_joint.base_vel[0];
                robotwb.base_vel_sdk[2]=_remote_msg_joint.base_vel[2];

                if(robotwb.arm_control_mode==99&&_remote_msg_joint.replay_rate>0.01&&_remote_msg_joint.replay_rate<0.99)
                {
                    if(cnt_p++>25){cnt_p=0;
                    printf("SDK Comman Replay:%.1f%\n",_remote_msg_joint.replay_rate*100);
                    }
                }
                if(_remote_msg_joint.replay_rate>0.98){
                    recording_flag=1;
                    _remote_msg_joint.replay_rate=0;
                    _robot_record_msg.start_record=robotwb.start_record=0;
                    robotwb.record_mode=0;
                    printf("SDK Finish Replay!\n");
                }
            }else if(recv_num==16){//imitate replay
                memcpy(&_remote_msg_imitate,recv_buf,sizeof(_remote_msg_imitate));

                if(recording_flag==0&&_remote_msg_imitate.mode==1&&_robot_record_msg.start_record==1)
                {
                    recording_flag=1;
                    printf("SDK Start Imiate -------------------Hg5.[%.0f]!\n",_remote_msg_record.episode_idx);
                }

                if(recording_flag&&(robotwb.start_record==0))
                {
                    if(robotwb.start_record==0){
                        recording_flag=0;
                        _remote_msg_record.record_rate=0;
                        _robot_record_msg.start_record=robotwb.start_record=0;
                        printf("SDK Finish Imiating!\n");
                    }
                }
                if(recording_flag==1)
                    loss_cnt_sdk=0;
                memcpy(send_buf,&_robot_record_msg,sizeof(_robot_record_msg));
                send_num = sendto(sock_fd, send_buf, sizeof(_robot_record_msg), MSG_DONTWAIT, (struct sockaddr *)&addr_client, len);
            }
            //-------------------------------------
            else if(recv_num==56){//skill sdk
                loss_cnt_sdk=0;
                memcpy(&_remote_msg_skill,recv_buf,sizeof(_remote_msg_skill));
                if(robotwb.arm_control_mode!=97&&robotwb.skill_lsm==1)
                {
                    robotwb.arm_control_mode=97;//sdk
                    robotwb.head_control_mode=HEAD_RECODE;

                    if(_remote_msg_skill.skill_id==1)//grasp
                        trigger_grasp_target(_remote_msg_skill.pose_msg1.x,//grasp
                                             _remote_msg_skill.pose_msg1.y,
                                             _remote_msg_skill.pose_msg1.z,
                                             _remote_msg_skill.att_msg1.x,
                                             _remote_msg_skill.att_msg1.y,
                                             _remote_msg_skill.att_msg1.z,
                                             _remote_msg_skill.pose_msg2.x,//app
                                             _remote_msg_skill.pose_msg2.y,
                                             _remote_msg_skill.pose_msg2.z,
                                             _remote_msg_skill.att_msg2.x,
                                             _remote_msg_skill.att_msg2.y,
                                             _remote_msg_skill.att_msg2.z);
                    if(_remote_msg_skill.skill_id==2)//put
                        trigger_put_pose(_remote_msg_skill.pose_msg1.x,
                                         _remote_msg_skill.pose_msg1.y,
                                         _remote_msg_skill.pose_msg1.z
                                         );
                    if(_remote_msg_skill.skill_id==3)//move
                        trigger_put_pose(robotwb.grasp_pose_skill.x+_remote_msg_skill.pose_msg1.x,
                                         robotwb.grasp_pose_skill.y+_remote_msg_skill.pose_msg1.y,
                                         robotwb.grasp_pose_skill.z+_remote_msg_skill.pose_msg1.z
                                         );
                }
            }
            //-----------------------------------
            else if(recv_num==96){//epos grasp pose----------------------------[1]
                loss_cnt_sdk=0;
                memcpy(&remote_msg_end_grasp,recv_buf,sizeof(remote_msg_end_grasp));
                if(robotwb.grasp_action==1){
                    robotwb.arm_epos_h_grasp.x=remote_msg_end_grasp.arm_pos_exp_r[0];
                    robotwb.arm_epos_h_grasp.y=remote_msg_end_grasp.arm_pos_exp_r[1];
                    robotwb.arm_epos_h_grasp.z=remote_msg_end_grasp.arm_pos_exp_r[2]+robotwb.arm_base_height;
                    robotwb.arm_att_b_grasp.x=remote_msg_end_grasp.arm_att_exp_r[0];
                    robotwb.arm_att_b_grasp.y=remote_msg_end_grasp.arm_att_exp_r[1];
                    robotwb.arm_att_b_grasp.z=remote_msg_end_grasp.arm_att_exp_r[2];

                    robotwb.arm_epos_h_grasp_app.x=remote_msg_end_grasp.arm_pos_exp_r_app[0];
                    robotwb.arm_epos_h_grasp_app.y=remote_msg_end_grasp.arm_pos_exp_r_app[1];
                    robotwb.arm_epos_h_grasp_app.z=remote_msg_end_grasp.arm_pos_exp_r_app[2]+robotwb.arm_base_height;
                    robotwb.arm_att_b_grasp_app.x=remote_msg_end_grasp.arm_att_exp_r_app[0];
                    robotwb.arm_att_b_grasp_app.y=remote_msg_end_grasp.arm_att_exp_r_app[1];
                    robotwb.arm_att_b_grasp_app.z=remote_msg_end_grasp.arm_att_exp_r_app[2];

                    robotwb.arm_control_mode=97;//sdk
                    robotwb.head_control_mode=HEAD_RECODE;
                    printf("SDK epos-grasp mode enable!\n");
                    printf("grasp_app pos:%f %f %f\n",
                           robotwb.arm_epos_h_grasp_app.x,
                           robotwb.arm_epos_h_grasp_app.y,
                           robotwb.arm_epos_h_grasp_app.z);
                    printf("grasp_app att:%f %f %f\n",
                           robotwb.arm_att_b_grasp_app.x,
                           robotwb.arm_att_b_grasp_app.y,
                           robotwb.arm_att_b_grasp_app.z);

                    printf("grasp pos:%f %f %f\n",
                           robotwb.arm_epos_h_grasp.x,
                           robotwb.arm_epos_h_grasp.y,
                           robotwb.arm_epos_h_grasp.z);
                    printf("grasp att:%f %f %f\n",
                           robotwb.arm_att_b_grasp.x,
                           robotwb.arm_att_b_grasp.y,
                           robotwb.arm_att_b_grasp.z);
                    robotwb.grasp_action=2;
                }
            }
            //------------------------------------------
            else if(recv_num==8&&0){//waypoint  doghome
                memcpy(&_remote_msg_waypoint,recv_buf,sizeof(_remote_msg_waypoint));
                int way_point=_remote_msg_waypoint.waypoint;
                if(way_point==0){
                    robotwb.wheel_control_mode=0;
                    robotwb.tar_waypoint=robotwb.way_point[0];
                    float dis=sqrt(pow(robotwb.tar_waypoint.x-robotwb.tar_waypoint_reg.x,2)+pow(robotwb.tar_waypoint.y-robotwb.tar_waypoint_reg.y,2));
                    if(dis>0.02){
                        robotwb.good_waypoint=1;
                        robotwb.tar_waypoint_reg=robotwb.tar_waypoint;
                    }
                }
                if(way_point==1){
                    robotwb.wheel_control_mode=1;
                    robotwb.tar_waypoint=robotwb.way_point[0];
                    float dis=sqrt(pow(robotwb.tar_waypoint.x-robotwb.tar_waypoint_reg.x,2)+pow(robotwb.tar_waypoint.y-robotwb.tar_waypoint_reg.y,2));
                    if(dis>0.05){
                        robotwb.good_waypoint=1;
                        robotwb.tar_waypoint_reg=robotwb.tar_waypoint;
                    }
                }
                if(way_point==2){
                    robotwb.wheel_control_mode=1;
                    robotwb.tar_waypoint=robotwb.way_point[1];
                    float dis=sqrt(pow(robotwb.tar_waypoint.x-robotwb.tar_waypoint_reg.x,2)+pow(robotwb.tar_waypoint.y-robotwb.tar_waypoint_reg.y,2));
                    if(dis>0.02){
                        robotwb.good_waypoint=1;
                        robotwb.tar_waypoint_reg=robotwb.tar_waypoint;
                    }
                }
                if(way_point==3){
                    robotwb.wheel_control_mode=1;
                    robotwb.tar_waypoint=robotwb.way_point[2];
                    float dis=sqrt(pow(robotwb.tar_waypoint.x-robotwb.tar_waypoint_reg.x,2)+pow(robotwb.tar_waypoint.y-robotwb.tar_waypoint_reg.y,2));
                    if(dis>0.02){
                        robotwb.good_waypoint=1;
                        robotwb.tar_waypoint_reg=robotwb.tar_waypoint;
                    }
                }
                if(way_point==4){
                    robotwb.wheel_control_mode=1;
                    robotwb.tar_waypoint=robotwb.way_point[3];
                    float dis=sqrt(pow(robotwb.tar_waypoint.x-robotwb.tar_waypoint_reg.x,2)+pow(robotwb.tar_waypoint.y-robotwb.tar_waypoint_reg.y,2));
                    if(dis>0.02){
                        robotwb.good_waypoint=1;
                        robotwb.tar_waypoint_reg=robotwb.tar_waypoint;
                    }
                }

                if(way_point==5){
                    robotwb.wheel_control_mode=1;
                    robotwb.tar_waypoint=robotwb.way_point[4];
                    float dis=sqrt(pow(robotwb.tar_waypoint.x-robotwb.tar_waypoint_reg.x,2)+pow(robotwb.tar_waypoint.y-robotwb.tar_waypoint_reg.y,2));
                    if(dis>0.02){
                        robotwb.good_waypoint=1;
                        robotwb.tar_waypoint_reg=robotwb.tar_waypoint;
                    }
                }

                if(robotwb.wheel_control_mode==1&&reg_wheel==0){
                    printf("SDK::Imitate Waypoint Mode!\n");
                }
                else if(robotwb.wheel_control_mode==0&&reg_wheel==1){
                    printf("SDK::Imitate RC Mode!\n");
                }
                reg_wheel=robotwb.wheel_control_mode;
                memcpy(send_buf,&_robot_record_msg,sizeof(_robot_record_msg));
                send_num = sendto(sock_fd, send_buf, sizeof(_robot_record_msg), MSG_DONTWAIT, (struct sockaddr *)&addr_client, len);
            }

            //------------------------发送机器人状态---------------------------------
//            robot_msg_imt.start_record
//            memcpy(send_buf,&_robot_msg,sizeof(robot_msg_imt));
//            send_num = sendto(sock_fd, send_buf, sizeof(robot_msg_imt), MSG_DONTWAIT, (struct sockaddr *)&addr_client, len);
//            printf("usb_send_cnt=%d\n",send_num);
            if(send_num < 0)
            {
                perror("OCU sendto error:");
                exit(1);
            }
        }
        usleep(5*1000);//200Hz
    }
    close(sock_fd);
    return 0;
}
//---------------------------------------------------------------------------------------------------------
void* Thread_5ms(void*)//
{
    static float init_cnt = 0;
    static int init_done = 0;
    int i = 0;
    static float timer[10] = { 0 };
    float dT=0.005;

    printf("Real RoboButler_5ms\n");
    while (1) {//5MS
        dT = Get_Cycle_T(10);
        updateJoy();

        locomotion_sfm(dT);

        state_estimator(dT);

        subscribe_webot_to_vmc(dT);

        body_servo_control(dT);

        usleep(5*1000);
    }//end while
}

void* Thread_1ms(void*)//
{
    static float init_cnt = 0;
    static int init_done = 0;
    int i = 0;
    static float timer[10] = { 0 };
    float dT=0.001;

    printf("Real RoboButler_1ms\n");
    while (1) {//
        dT = Get_Cycle_T(11);

        subscribe_imu_to_webot(&robotwb, dT);//赋值给robot结构体 IMU

        readAllMotorPos(&robotwb, dT);//读取角度

        estimate_end_state_new(dT);	//运动学正解
#if 1
        //grasp_mission(dT);
        grasp_mission_auto(dT);
#else
        skill_callback(dT);
#endif
        force_control_and_dis_pose(dT);//位力混控 站立版本

        set_motor_arm();
        set_motor_head();
        set_motor_w();
        set_motor_cap();
        usleep(1000);
    }//end while
}




int main(int argc, char **argv) {
  rbdl_test();
  vmc_param_init();

  pthread_t tid_servo,tid_nav,tid_1ms,tid_5ms,tid_hs,tid_hs1;
  pthread_create(&tid_servo, NULL, Thread_Mem_Servo, NULL);
  pthread_create(&tid_nav, NULL, Thread_Mem_Navigation, NULL);
  pthread_create(&tid_1ms, NULL, Thread_1ms, NULL);
  pthread_create(&tid_5ms, NULL, Thread_5ms, NULL);

  pthread_create(&tid_hs, NULL,  Thread_UDP_SDK, NULL);
  pthread_create(&tid_hs1, NULL, Thread_UDP_SDK_IMIT, NULL);
  pthread_join(tid_nav, NULL);
  pthread_join(tid_servo, NULL);
  pthread_join(tid_1ms, NULL);
  pthread_join(tid_5ms, NULL);
  pthread_join(tid_hs, NULL);
}

