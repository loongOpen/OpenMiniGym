#include <sys/shm.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <time.h>
#include "comm.h"
#include "spi_node.h"
#include "sys_time.h"
#include "locomotion_header.h"
#include "base_struct.h"
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
#include <pthread.h>
#include "include.h"
#include "locomotion_header.h"
#include "gait_math.h"
#include "adrc.h"
#include "base_struct.h"
_Webots webots_mem;

int mem_connect=0;
float mem_loss_cnt=0;

struct shareMemory
{
    int  flag=0;  //作为一个标志，非0：表示可读，0表示可写
    unsigned char szMsg[MEM_SIZE];
};
struct shareMemory shareMemory_spi,shareMemory_control;

unsigned char mem_read_buf[MEM_SIZE];
unsigned char mem_write_buf[MEM_SIZE];
long mem_read_cnt=0;
long mem_write_cnt=0;

static void setDataFloat_mem(float f, long *anal_cnt)
{
    int i = *(int *)&f;
    mem_write_buf[*anal_cnt+0] = ((i << 24) >> 24);
    mem_write_buf[*anal_cnt+1] = ((i << 16) >> 24);
    mem_write_buf[*anal_cnt+2] = ((i << 8) >> 24);
    mem_write_buf[*anal_cnt+3] = (i >> 24);

    *anal_cnt += 4;
}

static void setDataInt_mem(int i, long *anal_cnt)
{
    mem_write_buf[*anal_cnt+0] = ((i << 24) >> 24);
    mem_write_buf[*anal_cnt+1] = ((i << 16) >> 24);
    mem_write_buf[*anal_cnt+2] = ((i << 8) >> 24);
    mem_write_buf[*anal_cnt+3] = (i >> 24);

    *anal_cnt += 4;
}

static void setDataChar_mem(char f)
{
    mem_write_buf[mem_write_cnt++] = (f);
}

static float floatFromData_spi(unsigned char *data, long *anal_cnt)
{
    int i = 0x00;
    i |= (*(data + *anal_cnt + 3) << 24);
    i |= (*(data + *anal_cnt + 2) << 16);
    i |= (*(data + *anal_cnt + 1) << 8);
    i |= (*(data + *anal_cnt + 0));

    *anal_cnt += 4;
    return *(float *)&i;
}

static char charFromData_spi(unsigned char *data, long *anal_cnt)
{
    int temp = *anal_cnt;
    *anal_cnt += 1;
    return *(data + temp);
}


static void setDataFloat_spi_int(int f, long *anal_cnt)
{
    int16_t _temp;
    _temp=f;
    mem_write_buf[*anal_cnt]   = BYTE1(_temp);
    mem_write_buf[*anal_cnt+1] = BYTE0(_temp);
    //printf("%d %d %d\n",_temp,mem_write_buf[*anal_cnt],mem_write_buf[*anal_cnt+1] );
    *anal_cnt+=2;
}

//------------------------------内存管理--Hardware----------------------
void memory_read(void)
{//读取ROS command
    mem_read_cnt=0;
    //spi_rx.att[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    //spi_rx.att[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    //spi_rx.att[2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);

}

void memory_write(void)//写入Memeory to ROS
{
    mem_write_cnt=0;//MEM_SIZE/2;
    //------------------------写入内存

    setDataChar_mem (webots_mem.save_data);

    setDataFloat_mem(robotwb.arm_base_height_exp_flt,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q_exp_flt[0][0],&mem_write_cnt);//convet to rad
    setDataFloat_mem(robotwb.arm_q_exp_flt[0][1],&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q_exp_flt[0][2],&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q_exp_flt[0][3],&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q_exp_flt[0][4],&mem_write_cnt);
    setDataFloat_mem(robotwb.cap_set_flt,&mem_write_cnt);//set

    setDataFloat_mem(robotwb.arm_base_height,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q[0][0]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q[0][1]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q[0][2]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q[0][3]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q[0][4]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_base_dheight,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_dq[0][0]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_dq[0][1]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_dq[0][2]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_dq[0][3]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_dq[0][4]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.cap_set_flt,&mem_write_cnt);//fb
//
    setDataFloat_mem(robotwb.arm_base_height_exp_flt,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q_exp_flt[1][0],&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q_exp_flt[1][1],&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q_exp_flt[1][2],&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q_exp_flt[1][3],&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q_exp_flt[1][4],&mem_write_cnt);
    setDataFloat_mem(robotwb.cap_set_flt,&mem_write_cnt);//set

    setDataFloat_mem(robotwb.arm_base_height,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q[1][0]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q[1][1]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q[1][2]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q[1][3]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_q[1][4]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_base_dheight,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_dq[1][0]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_dq[1][1]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_dq[1][2]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_dq[1][3]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.arm_dq[1][4]*57.3,&mem_write_cnt);
    setDataFloat_mem(robotwb.cap_set_flt,&mem_write_cnt);//fb
#if 0
    setDataFloat_mem(robotwb.base_vel_b_w_fushion.x,&mem_write_cnt);
    setDataFloat_mem(robotwb.base_vel_b_w_fushion.y,&mem_write_cnt);
    setDataFloat_mem(robotwb.base_rate_fushion,&mem_write_cnt);
#else
    setDataFloat_mem(robotwb.base_vel_b_exp_rc_flt.x,&mem_write_cnt);
    setDataFloat_mem(robotwb.base_vel_b_exp_rc_flt.y,&mem_write_cnt);
    setDataFloat_mem(robotwb.base_rate_exp_rc_flt,&mem_write_cnt);
#endif
    for(int i=0;i<640;i++){
        for(int j=0;j<480;j++){
            setDataFloat_spi_int( webots_mem.pic_r[i][j],&mem_write_cnt);
            setDataFloat_spi_int( webots_mem.pic_g[i][j],&mem_write_cnt);
            setDataFloat_spi_int( webots_mem.pic_b[i][j],&mem_write_cnt);
        }
    }
    for(int i=0;i<640;i++){
        for(int j=0;j<480;j++){
            setDataFloat_spi_int( webots_mem.pic_r1[i][j],&mem_write_cnt);
            setDataFloat_spi_int( webots_mem.pic_g1[i][j],&mem_write_cnt);
            setDataFloat_spi_int( webots_mem.pic_b1[i][j],&mem_write_cnt);
        }
    }

    for(int i=0;i<640;i++){
        for(int j=0;j<480;j++){
            setDataFloat_spi_int( webots_mem.pic_r2[i][j],&mem_write_cnt);
            setDataFloat_spi_int( webots_mem.pic_g2[i][j],&mem_write_cnt);
            setDataFloat_spi_int( webots_mem.pic_b2[i][j],&mem_write_cnt);
        }
    }

    for(int i=0;i<360;i++){
            setDataFloat_spi_int( webots_mem.lidar_dis[i]*100,&mem_write_cnt);
    }

    webots_mem.att[0]=vmc_all.att[PITr];
    webots_mem.att[1]=vmc_all.att[ROLr];
    webots_mem.att[2]=vmc_all.att[YAWr];

    for(int i=0;i<3;i++){
            setDataFloat_spi_int( webots_mem.att[i]*100,&mem_write_cnt);
    }
}

void* Thread_Mem_Webots(void*)//内存管理线程
{
    float timer[5]={0};
    float sys_dt = 0.005;
    static int mem_init_cnt=0;
    int i=0;
    int link_cnt=0;
    int shmid_rx = shmget((key_t)MEM_SPI, sizeof(shareMemory_spi), 0666|IPC_CREAT); //失败返回-1，假设成功
    void *shm_rx = shmat(shmid_rx, 0, 0);
    shareMemory *pshm_rx = (shareMemory*)shm_rx;
    printf("Webots::Memory attached at %p\n",shm_rx);

    while(1)
    {
       //共享内存读取 to Servo task
        switch(pshm_rx->flag){
        case 0:
            if(!mem_connect){
            mem_init_cnt++;
            if(mem_init_cnt>2){
                printf("Webots::Memory ROS Link=%d!!!\n",link_cnt++);
                mem_connect=1;
            }
            }
            mem_loss_cnt=0;

            for(long k=0;k<MEM_SIZE;k++)
                mem_read_buf[k]=pshm_rx->szMsg[k];

            memory_read();
            pshm_rx->flag=1;
        break;
        case 2:
            memory_write();

            for(long k=0;k<MEM_SIZE;k++)
                pshm_rx->szMsg[k]=mem_write_buf[k];

             pshm_rx->flag=3;
        break;

        }

        mem_loss_cnt+=sys_dt;
        if(mem_loss_cnt>1&&mem_connect==1){
            mem_connect=0;
            mem_loss_cnt=0;
            mem_init_cnt=0;

            printf("Webots::Memery ROS Loss!!!\n");
        }

   #if 0
        if(pshm_rx->flag == 1)
        {
            if(!mem_connect){
            mem_init_cnt++;
            if(mem_init_cnt>2){
                printf("Webots::Memory ROS Link=%d!!!\n",link_cnt++);
                mem_connect=1;
            }
            }
            mem_loss_cnt=0;
#if EN_THREAD_LOCK
            pthread_mutex_lock(&lock);
#endif
            for(long k=0;k<MEM_SIZE;k++)
                mem_read_buf[k]=pshm_rx->szMsg[k];

            memory_read();
            memory_write();
#if EN_THREAD_LOCK
            pthread_mutex_unlock(&lock);
#endif
            for(long k=0;k<MEM_SIZE;k++)
                pshm_rx->szMsg[k]=mem_write_buf[k];

            pshm_rx->flag = 0;
        }

        mem_loss_cnt+=sys_dt;
        if(mem_loss_cnt>1&&mem_connect==1){
            mem_connect=0;
            mem_loss_cnt=0;
            mem_init_cnt=0;

            printf("Webots::Memery ROS Loss!!!\n");
        }
#endif
        //printf("%f\n",mem_loss_cnt);
        usleep(2*1000);
    }

    shmdt(shm_rx);  //失败返回-1，假设成功
    shmctl(shmid_rx, IPC_RMID, 0);  //失败返回-1，假设成功。仅在reader这里删除共享内存，保证读完最后一个消息
    return 0;
}



struct webots_msg
{

    float q_exp[2][6];
    float q[2][6];
    float dq[2][6];
    float tau[2][6];
    float cap_rate_exp[2];
    float cap_rate[2];
    float base_vel[3];

    int save_data;
} _webots_msg;


struct webots_msg_pic
{
    char fig_id;
    int16_t pic_s[640][480];
} _webots_msg_pic;

static void Perror(const char *s)
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

void* Thread_UDP_Webots1(void*)// as 服务器作为
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
   addr_serv.sin_port = htons(4000);//端口
   addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
   len = sizeof(addr_serv);

   if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
   {
     perror("bind error:");
     exit(1);
   }

    int recv_num=0,send_num=0;
    int recording_flag=0;
    printf("sizeof(webots_msg)=%d\n",sizeof(webots_msg));
    char send_buf[sizeof(_webots_msg)]={0},recv_buf[sizeof(_webots_msg)]={0};
    struct sockaddr_in addr_client;
    printf("Thread UDP Webots-Motion\n");
    while (1)
    {
        //读取客户端 读取操控指令
        loss_cnt_sdk+=0.005;
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        if(recv_num >0)
        {
            //printf("recv_num=%d\n",recv_num);

//            memcpy(&_remote_msg_end,recv_buf,sizeof(_remote_msg_end));
//            if(robotwb.arm_control_mode!=98)
//            {
//                robotwb.arm_control_mode=98;//sdk
//                robotwb.wheel_control_mode=98;//sdk
//                robotwb.head_control_mode=HEAD_LOOK_B;
//                printf("SDK epos mode enable!\n");
//            }


            //------------------------发送机器人状态---------------------------------
            //------------------------写入内存
            _webots_msg.save_data=webots_mem.save_data;

            _webots_msg.q_exp[0][0]=robotwb.arm_base_height_exp_flt ;
            _webots_msg.q_exp[0][1]=robotwb.arm_q_exp_flt[0][0];//convet to rad
            _webots_msg.q_exp[0][2]=robotwb.arm_q_exp_flt[0][1];
            _webots_msg.q_exp[0][3]=robotwb.arm_q_exp_flt[0][2];
            _webots_msg.q_exp[0][4]=robotwb.arm_q_exp_flt[0][3];
            _webots_msg.q_exp[0][5]=robotwb.arm_q_exp_flt[0][4];
            _webots_msg.cap_rate_exp[0]=robotwb.cap_set_flt;//set

            _webots_msg.q[0][0]=robotwb.arm_base_height;
            _webots_msg.q[0][1]=robotwb.arm_q[0][0]*57.3;
            _webots_msg.q[0][2]=robotwb.arm_q[0][1]*57.3;
            _webots_msg.q[0][3]=robotwb.arm_q[0][2]*57.3;
            _webots_msg.q[0][4]=robotwb.arm_q[0][3]*57.3;
            _webots_msg.q[0][5]=robotwb.arm_q[0][4]*57.3;
            _webots_msg.dq[0][0]=robotwb.arm_base_dheight;
            _webots_msg.dq[0][1]=robotwb.arm_dq[0][0]*57.3;
            _webots_msg.dq[0][2]=robotwb.arm_dq[0][1]*57.3;
            _webots_msg.dq[0][3]=robotwb.arm_dq[0][2]*57.3;
            _webots_msg.dq[0][4]=robotwb.arm_dq[0][3]*57.3;
            _webots_msg.dq[0][5]=robotwb.arm_dq[0][4]*57.3;
            _webots_msg.cap_rate[0]=robotwb.cap_set_flt;//fb
        //
            _webots_msg.q_exp[1][0]=robotwb.arm_base_height_exp_flt ;
            _webots_msg.q_exp[1][1]=robotwb.arm_q_exp_flt[1][0];//convet to rad
            _webots_msg.q_exp[1][2]=robotwb.arm_q_exp_flt[1][1];
            _webots_msg.q_exp[1][3]=robotwb.arm_q_exp_flt[1][2];
            _webots_msg.q_exp[1][4]=robotwb.arm_q_exp_flt[1][3];
            _webots_msg.q_exp[1][5]=robotwb.arm_q_exp_flt[1][4];
            _webots_msg.cap_rate_exp[1]=robotwb.cap_set_flt;//set

            _webots_msg.q[1][0]=robotwb.arm_base_height;
            _webots_msg.q[1][1]=robotwb.arm_q[1][0]*57.3;
            _webots_msg.q[1][2]=robotwb.arm_q[1][1]*57.3;
            _webots_msg.q[1][3]=robotwb.arm_q[1][2]*57.3;
            _webots_msg.q[1][4]=robotwb.arm_q[1][3]*57.3;
            _webots_msg.q[1][5]=robotwb.arm_q[1][4]*57.3;
            _webots_msg.dq[1][0]=robotwb.arm_base_dheight;
            _webots_msg.dq[1][1]=robotwb.arm_dq[1][0]*57.3;
            _webots_msg.dq[1][2]=robotwb.arm_dq[1][1]*57.3;
            _webots_msg.dq[1][3]=robotwb.arm_dq[1][2]*57.3;
            _webots_msg.dq[1][4]=robotwb.arm_dq[1][3]*57.3;
            _webots_msg.dq[1][5]=robotwb.arm_dq[1][4]*57.3;
            _webots_msg.cap_rate[1]=robotwb.cap_set_flt;//fb

            _webots_msg.base_vel[0]=robotwb.base_vel_b_exp_rc_flt.x;
            _webots_msg.base_vel[1]=robotwb.base_vel_b_exp_rc_flt.y;
            _webots_msg.base_vel[2]=robotwb.base_rate_exp_rc_flt;

            memcpy(send_buf,&_webots_msg,sizeof(_webots_msg));
            send_num = sendto(sock_fd, send_buf, sizeof(_webots_msg), MSG_DONTWAIT, (struct sockaddr *)&addr_client, len);
//            printf("usb_send_cnt=%d\n",send_num);
            if(send_num < 0)
            {
                perror("Webots sendto error:");
                exit(1);
            }
        }
        usleep(5*1000);
    }
    close(sock_fd);
    return 0;
}



void* Thread_UDP_Webots2(void*)// as 服务器作为
{
    static int cnt = 0;
    float sys_dt = 0;
    float loss_cnt_sdk=0;
    int flag = 0;
    float timer[5]={0};
    int i=0;

    int sock_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
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
   addr_serv.sin_port = htons(10001);//端口
   addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
   len = sizeof(addr_serv);

   if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
   {
     perror("bind error:");
     exit(1);
   }

    int recv_num=0,send_num=0;
    int recording_flag=0;
    printf("sizeof(webots_msg_pic)=%d\n",sizeof(webots_msg_pic));
    char send_buf[sizeof(_webots_msg_pic)]={0},recv_buf[sizeof(_webots_msg_pic)]={0};
    struct sockaddr_in addr_client;
    int send_cnt=0;
    printf("Thread UDP Webots-Camera\n");
    while (1)
    {
        //读取客户端 读取操控指令
        loss_cnt_sdk+=0.005;
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        if(recv_num >0)
        {
            //printf("recv_num=%d\n",recv_num);

//            memcpy(&_remote_msg_end,recv_buf,sizeof(_remote_msg_end));
//            if(robotwb.arm_control_mode!=98)
//            {
//                robotwb.arm_control_mode=98;//sdk
//                robotwb.wheel_control_mode=98;//sdk
//                robotwb.head_control_mode=HEAD_LOOK_B;
//                printf("SDK epos mode enable!\n");
//            }


            //------------------------发送机器人状态---------------------------------
            //------------------------写入内存

#if 1

            switch(send_cnt++){
            case 0:
                _webots_msg_pic.fig_id=0;
                for(int i=0;i<640;i++){
                    for(int j=0;j<480;j++){
                        _webots_msg_pic.pic_s[i][j]=(int16_t)webots_mem.pic_r[i][j];
                    }
                }
            break;
            case 1:
                _webots_msg_pic.fig_id=1;
                for(int i=0;i<640;i++){
                    for(int j=0;j<480;j++){
                        _webots_msg_pic.pic_s[i][j]=(int16_t)webots_mem.pic_g[i][j];
                    }
                }
            break;
            case 2:
                _webots_msg_pic.fig_id=2;
                for(int i=0;i<640;i++){
                    for(int j=0;j<480;j++){
                        _webots_msg_pic.pic_s[i][j]=(int16_t)webots_mem.pic_b[i][j];
                    }
                }
            break;

            case 3:
                _webots_msg_pic.fig_id=3;
                for(int i=0;i<640;i++){
                    for(int j=0;j<480;j++){
                        _webots_msg_pic.pic_s[i][j]=(int16_t)webots_mem.pic_r1[i][j];
                    }
                }
            break;
            case 4:
                _webots_msg_pic.fig_id=4;
                for(int i=0;i<640;i++){
                    for(int j=0;j<480;j++){
                        _webots_msg_pic.pic_s[i][j]=(int16_t)webots_mem.pic_g1[i][j];
                    }
                }
            break;
            case 5:
                _webots_msg_pic.fig_id=5;
                for(int i=0;i<640;i++){
                    for(int j=0;j<480;j++){
                        _webots_msg_pic.pic_s[i][j]=(int16_t)webots_mem.pic_b1[i][j];
                    }
                }
            break;


            case 6:
                _webots_msg_pic.fig_id=6;
                for(int i=0;i<640;i++){
                    for(int j=0;j<480;j++){
                        _webots_msg_pic.pic_s[i][j]=(int16_t)webots_mem.pic_r2[i][j];
                    }
                }
            break;
            case 7:
                _webots_msg_pic.fig_id=7;
                for(int i=0;i<640;i++){
                    for(int j=0;j<480;j++){
                        _webots_msg_pic.pic_s[i][j]=(int16_t)webots_mem.pic_g2[i][j];
                    }
                }
            break;
            case 8:
                _webots_msg_pic.fig_id=8;
                for(int i=0;i<640;i++){
                    for(int j=0;j<480;j++){
                        _webots_msg_pic.pic_s[i][j]=(int16_t)webots_mem.pic_b2[i][j];
                    }
                }
                send_cnt=0;
            break;
            }

#endif
            memcpy(send_buf,&_webots_msg_pic,sizeof(_webots_msg_pic));
            send_num = sendto(sock_fd, send_buf, sizeof(_webots_msg_pic), MSG_DONTWAIT, (struct sockaddr *)&addr_client, len);
//           memcpy(send_buf,&_webots_msg_pic,sizeof(_webots_msg_pic));
//           send_num = sendto(sock_fd, send_buf, sizeof(_webots_msg_pic), MSG_DONTWAIT, (struct sockaddr *)&addr_client, len);
           //printf("usb_send_cnt=%d\n",send_num);
            if(send_num < 0)
            {
                perror("Webots sendto error:");
                //exit(1);
            }
        }
        usleep(2*1000);
    }
    close(sock_fd);
    return 0;
}

