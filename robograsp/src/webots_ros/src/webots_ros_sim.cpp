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

#include <chrono>
#include <opencv2/opencv.hpp>


_Webots webots_mem;

using std::cout;
using std::endl;
using namespace std;

char ros_connect=0;
int  ros_loss_cnt=0;
 
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MEM_SPI 0001
#define MEM_SIZE (640*480*2*3*3+50*4+360*2)


//----------------------------------------------------------------------------
long mem_connect=0;
float mem_loss_cnt=0;
struct shareMemory
{
    int  flag=0;  //作为一个标志，非0：表示可读，0表示可写
    unsigned char szMsg[MEM_SIZE];
};
struct shareMemory shareMemory_spi;

unsigned char mem_write_buf[MEM_SIZE];
unsigned char mem_read_buf[MEM_SIZE];
long mem_write_cnt=0;
static void setDataFloat_mem(float f)
{
    int i = *(int *)&f;
    mem_write_buf[mem_write_cnt++] = ((i << 24) >> 24);
    mem_write_buf[mem_write_cnt++] = ((i << 16) >> 24);
    mem_write_buf[mem_write_cnt++] = ((i << 8) >> 24);
    mem_write_buf[mem_write_cnt++] = (i >> 24);
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

static float floatFromData_spi_int(unsigned char *data, long *anal_cnt,float size)
{
    float temp=0;
    temp=(float)((int16_t)(*(data + *anal_cnt + 0)<<8)|*(data + *anal_cnt + 1))/size;
    *anal_cnt += 2;
    return temp;
}

static char charFromData_spi(unsigned char *data, long *anal_cnt)
{
    int temp = *anal_cnt;
    *anal_cnt += 1;
    return *(data + temp);
}

static int intFromData_spi(unsigned char *data, long *anal_cnt)
{
    int i = 0x00;
    i |= (*(data + *anal_cnt + 3) << 24);
    i |= (*(data + *anal_cnt + 2) << 16);
    i |= (*(data + *anal_cnt + 1) << 8);
    i |= (*(data + *anal_cnt + 0));
    *anal_cnt += 4;
    return i;
}

static int16_t intFromData_spi_int(unsigned char *data, long *anal_cnt)
{
    int16_t temp=0;
    float size=1;
    temp=((int16_t)(*(data + *anal_cnt + 0)<<8)|*(data + *anal_cnt + 1))/size;
    *anal_cnt += 2;
    return temp;
}

void memory_write(void)//写入内存 to webots
{
    static float temp=0;
    mem_write_cnt=0;
    // setDataFloat_mem( spi_rx.att[0]);
    // setDataFloat_mem( spi_rx.att[1]);
    // setDataFloat_mem( spi_rx.att[2]);
     

}

void memory_read(void){//读取内存 from webots
	long mem_read_cnt=0;//MEM_SIZE/2;
 
    char temp = mem_read_buf[mem_read_cnt++];
 
    webots_mem.q_exp[0][0]=floatFromData_spi(mem_read_buf, &mem_read_cnt);//right
    webots_mem.q_exp[0][1]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q_exp[0][2]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q_exp[0][3]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q_exp[0][4]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q_exp[0][5]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.cap_rate_exp[0]=floatFromData_spi(mem_read_buf, &mem_read_cnt);  

    webots_mem.q[0][0]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q[0][1]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q[0][2]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q[0][3]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q[0][4]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q[0][5]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.dq[0][0]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.dq[0][1]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.dq[0][2]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.dq[0][3]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.dq[0][4]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.dq[0][5]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.cap_rate[0]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
//
    webots_mem.q_exp[1][0]=floatFromData_spi(mem_read_buf, &mem_read_cnt);//left unused
    webots_mem.q_exp[1][1]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q_exp[1][2]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q_exp[1][3]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q_exp[1][4]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q_exp[1][5]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.cap_rate_exp[1]=floatFromData_spi(mem_read_buf, &mem_read_cnt);

    webots_mem.q[1][0]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q[1][1]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q[1][2]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q[1][3]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q[1][4]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.q[1][5]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.dq[1][0]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.dq[1][1]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.dq[1][2]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.dq[1][3]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.dq[1][4]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.dq[1][5]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.cap_rate[1]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    
    webots_mem.base_vel[0]=floatFromData_spi(mem_read_buf, &mem_read_cnt);//base
    webots_mem.base_vel[1]=floatFromData_spi(mem_read_buf, &mem_read_cnt);
    webots_mem.base_vel[2]=floatFromData_spi(mem_read_buf, &mem_read_cnt);


	for(int i=0;i<640;i++){//head
        for(int j=0;j<480;j++){
            webots_mem.pic_r[i][j]=intFromData_spi_int(mem_read_buf, &mem_read_cnt);
            webots_mem.pic_g[i][j]=intFromData_spi_int(mem_read_buf, &mem_read_cnt);
            webots_mem.pic_b[i][j]=intFromData_spi_int(mem_read_buf, &mem_read_cnt);
        }
    }
    for(int i=0;i<640;i++){//right
        for(int j=0;j<480;j++){
            webots_mem.pic_rr[i][j]=intFromData_spi_int(mem_read_buf, &mem_read_cnt);
            webots_mem.pic_gr[i][j]=intFromData_spi_int(mem_read_buf, &mem_read_cnt);
            webots_mem.pic_br[i][j]=intFromData_spi_int(mem_read_buf, &mem_read_cnt);
        }
    }
    for(int i=0;i<640;i++){//left
        for(int j=0;j<480;j++){
            webots_mem.pic_rl[i][j]=intFromData_spi_int(mem_read_buf, &mem_read_cnt);
            webots_mem.pic_gl[i][j]=intFromData_spi_int(mem_read_buf, &mem_read_cnt);
            webots_mem.pic_bl[i][j]=intFromData_spi_int(mem_read_buf, &mem_read_cnt);
        }
    }

    for(int i=0;i<360;i++){
        webots_mem.lidar_dis[i]=intFromData_spi_int(mem_read_buf, &mem_read_cnt)/100.0;
    }

    for(int i=0;i<3;i++){
        webots_mem.att[i]=intFromData_spi_int(mem_read_buf, &mem_read_cnt)/100.0;
    }
    //printf("%f\n", webots_mem.att[2]);
}


pthread_mutex_t lock;

void* Thread_MEM(void*)//内存管理线程
{
    static int cnt = 0;
    static int mem_init_cnt=0;
    float sys_dt = 0.005;
    int i=0,memory_update=0;
    int flag = 0;
    int link_cnt=0;
    //共享内存
    int shmid_rx = shmget((key_t)MEM_SPI, sizeof(shareMemory_spi), 0666|IPC_CREAT); //失败返回-1，假设成功。
    //0666表示权限，与文件一样。如0644,它表示允许一个进程创建的共享内存被内存创建者所拥有的进程向共享内存读取和写入数据，同时其他用户创建的进程只能读取共享内存。
    void *shm_rx = shmat(shmid_rx, (void*)0, 0);  //失败返回-1，假设成功
    shareMemory *pshm_rx = (shareMemory*)shm_rx;
    pshm_rx->flag = 0;
    printf("ROS::Memory Webots attached at %p\n",shm_rx);

    while (1)
    {
        //共享内存写
        #if 1
        switch(pshm_rx->flag){
        case 1:
             if(!mem_connect){
            	mem_init_cnt++;
                if(mem_init_cnt>3){
                printf("ROS::Memery Webots Link=%d!!!\n",link_cnt++);
                mem_connect=1;
                }
            }
            mem_loss_cnt=0;

            memory_write();
            for(long k=0;k<MEM_SIZE;k++)
                pshm_rx->szMsg[k]=mem_write_buf[k];
                
            pshm_rx->flag=2;
        break;
        case 3:
            for(long k=0;k<MEM_SIZE;k++)
                mem_read_buf[k]=pshm_rx->szMsg[k];
            memory_read();
            pshm_rx->flag=0;
        break;

        }


        if(mem_loss_cnt>1&&mem_connect==1){
            mem_connect=0;
            mem_loss_cnt=0;
            mem_init_cnt=0;
            
            printf("ROS::Memery Webots Loss!!!\n");
        }
        #else
        if(pshm_rx->flag == 0)
        {
			//printf("%d %d\n",mem_init_cnt,mem_connect);
            if(!mem_connect){
            	mem_init_cnt++;
                if(mem_init_cnt>3){
                printf("ROS::Memery Webots Link=%d!!!\n",link_cnt++);
                mem_connect=1;
                }
            }
            mem_loss_cnt=0;
            //pthread_mutex_lock(&lock);
            memory_write();
            for(long k=0;k<MEM_SIZE;k++)
                pshm_rx->szMsg[k]=mem_write_buf[k];
            for(long k=0;k<MEM_SIZE;k++)
                mem_read_buf[k]=pshm_rx->szMsg[k];
            memory_read();
            //pthread_mutex_unlock(&lock);
            pshm_rx->flag = 1;
        }else{
            mem_loss_cnt+=sys_dt;
            //mem_init_cnt=0;
        }

        if(mem_loss_cnt>1&&mem_connect==1){
            mem_connect=0;
            mem_loss_cnt=0;
            mem_init_cnt=0;
            
            printf("ROS::Memery Webots Loss!!!\n");
        }
        #endif
        usleep(2*1000);
    }
    shmdt(shm_rx);  //失败返回-1，假设成功
    shmctl(shmid_rx, IPC_RMID, 0);  //
    return 0;
}

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
 
#include <iostream>
#include <fstream>
#pragma pack(push, 1) // 1字节对齐

// BMP文件头结构体
struct BMPFileHeader {
    uint16_t type; // 文件类型，必须为"BM"
    uint32_t size; // 文件大小，单位为字节
    uint16_t reserved1; // 保留字段，必须为0
    uint16_t reserved2; // 保留字段，必须为0
    uint32_t offset; // 像素数据起始位置，单位为字节
};

// BMP位图信息头结构体
struct BMPInfoHeader {
    uint32_t size; // 信息头大小，必须为40
    int32_t width; // 图像宽度，单位为像素
    int32_t height; // 图像高度，单位为像素
    uint16_t planes; // 颜色平面数，必须为1
    uint16_t bit_count; // 每个像素的位数，必须为24
    uint32_t compression; // 压缩方式，必须为0
    uint32_t size_image; // 像素数据大小，单位为字节
    int32_t x_pels_per_meter; // X方向像素数/米
    int32_t y_pels_per_meter; // Y方向像素数/米
    uint32_t clr_used; // 使用的颜色数，必须为0
    uint32_t clr_important; // 重要的颜色数，必须为0
};

#pragma pack(pop)

// 将RGB24格式像素数据封装为BMP图像
bool write_bmp(const char* filename, uint8_t* data, int32_t width, int32_t height) {
    BMPFileHeader file_header = { 0 };
    BMPInfoHeader info_header = { 0 };
    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs) {
        std::cerr << "Failed to create file: " << filename << std::endl;
        return false;
    }
    // BMP文件头
    file_header.type = 0x4D42; // BM
    file_header.size = sizeof(BMPFileHeader) + sizeof(BMPInfoHeader) + width * height * 3;
    file_header.offset = sizeof(BMPFileHeader) + sizeof(BMPInfoHeader);
    ofs.write(reinterpret_cast<char*>(&file_header), sizeof(file_header));
    // BMP位图信息头
    info_header.size = sizeof(BMPInfoHeader);
    info_header.width = width;
    info_header.height = height;
    info_header.planes = 1;
    info_header.bit_count = 24;
    info_header.size_image = width * height * 3;
    ofs.write(reinterpret_cast<char*>(&info_header), sizeof(info_header));
    // 像素数据
    int32_t row_size = ((width * 3 + 3) / 4) * 4; // 行字节数，必须为4的倍数
    uint8_t* row_data = new uint8_t[row_size];
    for (int32_t y = height - 1; y >= 0; --y) { // BMP图像的行是从下往上存储的
        for (int32_t x = 0; x < width; ++x) {
            row_data[x * 3 + 2] = data[(y * width + x) * 3 + 0]; // B
            row_data[x * 3 + 1] = data[(y * width + x) * 3 + 1]; // G
            row_data[x * 3 + 0] = data[(y * width + x) * 3 + 2]; // R
        }
        ofs.write(reinterpret_cast<char*>(row_data), row_size);
    }
    delete[] row_data;
    ofs.close();
    return true;
}

void Simluation_data(float dt){
      static int cnt_cap = 0;
      static int cap_num = 0;
      if (cnt_cap++ > 1&&1) {//记录相机
            cnt_cap = 0;

            int width=640;
            int height=480;
            //printf("%d %d\n",width,height);
            int pic_r[width][height];
            int pic_g[width][height];
            int pic_b[width][height];

            uint8_t* data = new uint8_t[width * height * 3]; // RGB24格式像素数据
            for (int32_t x = 0; x < width; ++x) {
               for (int32_t y = 0; y < height; ++y) {
                   int32_t index = (y * width + x) * 3;
                   data[index + 0] = webots_mem.pic_r[x][y]; // R
                   data[index + 1] = webots_mem.pic_g[x][y]; // G
                   data[index + 2] = webots_mem.pic_b[x][y]; // B
                }
            }

            //write_bmp("/home/tinymal/catkin_webots/Camera/head.bmp", data, 640, 480);
            //---
            int width1=640;
            int height1=480;
            //printf("%d %d\n",width,height);
            int pic_r1[width1][height1];
            int pic_g1[width1][height1];
            int pic_b1[width1][height1];

 
            uint8_t* data1 = new uint8_t[width1 * height1 * 3]; // RGB24格式像素数据
            for (int32_t x = 0; x < width1; ++x) {
               for (int32_t y = 0; y < height1; ++y) {
                   int32_t index = (y * width1 + x) * 3;
                   data1[index + 0] = webots_mem.pic_rr[x][y]; // R
                   data1[index + 1] = webots_mem.pic_gr[x][y]; // G
                   data1[index + 2] = webots_mem.pic_br[x][y]; // B
                }
            }

        	//write_bmp("/home/tinymal/catkin_webots/Camera/hand0.jpg", data1, 640, 480);

      }
   
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


void* Thread_UDP_WEBOTS1(void*)//UDP管理线程 Joint as slave
{
//----------------------UDP init----------------------------
	int SERV_PORT= 10000 ;//OUTTER SDK
  
	int sock_fd;  
 
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
	string UDP_IP="127.0.0.1";
	addr_serv.sin_addr.s_addr = inet_addr(UDP_IP.c_str());  
	addr_serv.sin_port = htons(SERV_PORT);  
	len = sizeof(addr_serv);  

    int recv_num=0,send_num=0;  
    int connect=0,loss_cnt=0;
    char send_buf[500]={0},recv_buf[sizeof(webots_msg)]={0};  
    printf("ROS::Memory Webots-Joint UDP attached at %d\n",addr_serv.sin_port );

    while (1)
    {
        //memcpy(send_buf,&_webots_msg,sizeof(_webots_msg));
        send_num = sendto(sock_fd, send_buf, sizeof(send_buf), MSG_DONTWAIT, (struct sockaddr *)&addr_serv, len);
 
        if(send_num < 0)
        {
            perror("Webots sendto error:");
            exit(1);
        }


 	    recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), MSG_DONTWAIT, (struct sockaddr *)&addr_serv, (socklen_t *)&len); 
        if(recv_num >0)
        {
            if(!connect){
                connect=1;
                 printf("ROS::Memory Webots-Joint UDP Connected!!\n");
            }
            //printf("recv_num=%d\n",recv_num);
            memcpy(recv_buf,&_webots_msg,sizeof(_webots_msg));

            webots_mem.q_exp[0][0]=_webots_msg.q_exp[0][0];//right
            webots_mem.q_exp[0][1]=_webots_msg.q_exp[0][1];
            webots_mem.q_exp[0][2]=_webots_msg.q_exp[0][2];
            webots_mem.q_exp[0][3]=_webots_msg.q_exp[0][3];
            webots_mem.q_exp[0][4]=_webots_msg.q_exp[0][4];
            webots_mem.q_exp[0][5]=_webots_msg.q_exp[0][5];
            webots_mem.cap_rate_exp[0]=_webots_msg.cap_rate_exp[0];  

            webots_mem.q[0][0]=_webots_msg.q_exp[0][0];
            webots_mem.q[0][1]=_webots_msg.q_exp[0][1];
            webots_mem.q[0][2]=_webots_msg.q_exp[0][2];
            webots_mem.q[0][3]=_webots_msg.q_exp[0][3];
            webots_mem.q[0][4]=_webots_msg.q_exp[0][4];
            webots_mem.q[0][5]=_webots_msg.q_exp[0][5];
            webots_mem.dq[0][0]=_webots_msg.dq[0][0];
            webots_mem.dq[0][1]=_webots_msg.dq[0][1];
            webots_mem.dq[0][2]=_webots_msg.dq[0][2];
            webots_mem.dq[0][3]=_webots_msg.dq[0][3];
            webots_mem.dq[0][4]=_webots_msg.dq[0][4];
            webots_mem.dq[0][5]=_webots_msg.dq[0][5];
            webots_mem.cap_rate[0]=_webots_msg.cap_rate[0];
            //
            webots_mem.q_exp[1][0]=_webots_msg.q_exp[1][0];//right
            webots_mem.q_exp[1][1]=_webots_msg.q_exp[1][1];
            webots_mem.q_exp[1][2]=_webots_msg.q_exp[1][2];
            webots_mem.q_exp[1][3]=_webots_msg.q_exp[1][3];
            webots_mem.q_exp[1][4]=_webots_msg.q_exp[1][4];
            webots_mem.q_exp[1][5]=_webots_msg.q_exp[1][5];
            webots_mem.cap_rate_exp[1]=_webots_msg.cap_rate_exp[1];  

            webots_mem.q[1][0]=_webots_msg.q_exp[1][0];
            webots_mem.q[1][1]=_webots_msg.q_exp[1][1];
            webots_mem.q[1][2]=_webots_msg.q_exp[1][2];
            webots_mem.q[1][3]=_webots_msg.q_exp[1][3];
            webots_mem.q[1][4]=_webots_msg.q_exp[1][4];
            webots_mem.q[1][5]=_webots_msg.q_exp[1][5];
            webots_mem.dq[1][0]=_webots_msg.dq[1][0];
            webots_mem.dq[1][1]=_webots_msg.dq[1][1];
            webots_mem.dq[1][2]=_webots_msg.dq[1][2];
            webots_mem.dq[1][3]=_webots_msg.dq[1][3];
            webots_mem.dq[1][4]=_webots_msg.dq[1][4];
            webots_mem.dq[1][5]=_webots_msg.dq[1][5];
            webots_mem.cap_rate[1]=_webots_msg.cap_rate[1];

            webots_mem.base_vel[0]=_webots_msg.base_vel[0];//base
            webots_mem.base_vel[1]=_webots_msg.base_vel[1];
            webots_mem.base_vel[2]=_webots_msg.base_vel[2];

        }

        usleep(10*1000);
    }
    return 0;
}

struct webots_msg_pic
{
    char fig_id;
    int pic_s[640][480];
} _webots_msg_pic;

void* Thread_UDP_WEBOTS2(void*)//UDP管理线程 Camera as slave
{
//----------------------UDP init----------------------------
	int SERV_PORT= 10001 ;//OUTTER SDK
  
	int sock_fd;  
 
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
	string UDP_IP="127.0.0.1";
	addr_serv.sin_addr.s_addr = inet_addr(UDP_IP.c_str());  
	addr_serv.sin_port = htons(SERV_PORT);  
	len = sizeof(addr_serv);  

    int recv_num=0,send_num=0;  
    int connect=0,loss_cnt=0;
    char send_buf[500]={0},recv_buf[sizeof(_webots_msg_pic)]={0};  
    printf("ROS::Memory Webots-Camera UDP attached at %d\n",addr_serv.sin_port );

    while (1)
    {
        //memcpy(send_buf,&_webots_msg,sizeof(_webots_msg));
        send_num = sendto(sock_fd, send_buf, sizeof(send_buf), MSG_DONTWAIT, (struct sockaddr *)&addr_serv, len);
 
        if(send_num < 0)
        {
            perror("Webots Camera sendto error:");
            exit(1);
        }


 	    recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), MSG_DONTWAIT, (struct sockaddr *)&addr_serv, (socklen_t *)&len); 
        
        if(recv_num >0)
        {
            if(!connect){
                connect=1;
                 printf("ROS::Memory Webots-Camera UDP Connected!!\n");
            }
            printf("recv_num=%d\n",recv_num);
            memcpy(recv_buf,&_webots_msg_pic,sizeof(_webots_msg_pic));

            switch(_webots_msg_pic.fig_id){
                case 0:
                    for(int i=0;i<640;i++){//head
                        for(int j=0;j<480;j++){
                            webots_mem.pic_r[i][j]= _webots_msg_pic.pic_s[i][j];
                        }
                    }
                break;
                case 1:
                    for(int i=0;i<640;i++){//head
                        for(int j=0;j<480;j++){
                            webots_mem.pic_g[i][j]= _webots_msg_pic.pic_s[i][j];
                        }
                    }
                break;
                case 2:
                    for(int i=0;i<640;i++){//head
                        for(int j=0;j<480;j++){
                            webots_mem.pic_b[i][j]= _webots_msg_pic.pic_s[i][j];
                        }
                    }
                break;              

                case 3:
                    for(int i=0;i<640;i++){//head
                        for(int j=0;j<480;j++){
                            webots_mem.pic_rr[i][j]= _webots_msg_pic.pic_s[i][j];
                        }
                    }
                break;
                case 4:
                    for(int i=0;i<640;i++){//head
                        for(int j=0;j<480;j++){
                            webots_mem.pic_gr[i][j]= _webots_msg_pic.pic_s[i][j];
                        }
                    }
                break;
                case 5:
                    for(int i=0;i<640;i++){//head
                        for(int j=0;j<480;j++){
                            webots_mem.pic_br[i][j]= _webots_msg_pic.pic_s[i][j];
                        }
                    }
                break;     

                case 6:
                    for(int i=0;i<640;i++){//head
                        for(int j=0;j<480;j++){
                            webots_mem.pic_rl[i][j]= _webots_msg_pic.pic_s[i][j];
                        }
                    }
                break;
                case 7:
                    for(int i=0;i<640;i++){//head
                        for(int j=0;j<480;j++){
                            webots_mem.pic_gl[i][j]= _webots_msg_pic.pic_s[i][j];
                        }
                    }
                break;
                case 8:
                    for(int i=0;i<640;i++){//head
                        for(int j=0;j<480;j++){
                            webots_mem.pic_bl[i][j]= _webots_msg_pic.pic_s[i][j];
                        }
                    }
                break;                    
            }
        }

        usleep(10*1000);
    }
    return 0;

}

void* Thread_ROS(void*)//内存管理线程
{

	ros::NodeHandle n;
 
    // 创建一个图像发布者
    ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("/cam_high/image_raw", 10);
	ros::Publisher image_pubr = n.advertise<sensor_msgs::Image>("/cam_right_wrist/image_raw", 10);
	ros::Publisher image_publ = n.advertise<sensor_msgs::Image>("/cam_left_wrist/image_raw", 10);

    ros::Publisher joint_pub_r = n.advertise<sensor_msgs::JointState>("/puppet_right/joint_states", 1);
    ros::Publisher joint_pub_l = n.advertise<sensor_msgs::JointState>("/puppet_left/joint_states", 1);
    ros::Publisher joint_pub_base = n.advertise<sensor_msgs::JointState>("/puppet_base/joint_states", 1);

    ros::Publisher joint_pub_r_exp = n.advertise<sensor_msgs::JointState>("/puppet_right_exp/joint_states", 1);
    ros::Publisher joint_pub_l_exp = n.advertise<sensor_msgs::JointState>("/puppet_left_exp/joint_states", 1);
    //ros::Publisher joint_pub_base_exp = n.advertise<sensor_msgs::JointState>("/puppet_base_exp/joint_states", 1);

    ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("/laser", 10);

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 10);

    tf2_ros::StaticTransformBroadcaster tf_b;
    geometry_msgs::TransformStamped ts;

    int laser_num=360;
    double laser_f=25;
    double laser_dis[laser_num];
    double intensite[laser_num];
    int cout=1;

    float pos_n[3]={0};
    float vel_b[3];
    float vel_n[3];
	ros::Rate loop_rate(50);
	while (ros::ok())
	{  	
		ros::spinOnce();               // check for incoming messages
        //Simluation_data(0.5);
 
  /******************* 发布消息 ***************/
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();//right
        joint_state.name.resize(7);
        joint_state.position.resize(7);
        joint_state.velocity.resize(7);
        joint_state.effort.resize(7);
        joint_state.name[0] ="Link0";
        joint_state.position[0] = webots_mem.q[0][0];       
        joint_state.velocity[0] = webots_mem.dq[0][0];
        joint_state.effort[0] = webots_mem.tau[0][0];       
        joint_state.name[1] ="Link1";
        joint_state.position[1] = webots_mem.q[0][1];
        joint_state.velocity[1] = webots_mem.dq[0][1];
        joint_state.effort[1] = webots_mem.tau[0][1];    
        joint_state.name[2] ="Link2";
        joint_state.position[2] = webots_mem.q[0][2];
        joint_state.velocity[2] = webots_mem.dq[0][2];
        joint_state.effort[2] = webots_mem.tau[0][2];    
        joint_state.name[3] ="Link3";
        joint_state.position[3] = webots_mem.q[0][3];
        joint_state.velocity[3] = webots_mem.dq[0][3];
        joint_state.effort[3] = webots_mem.tau[0][3];    
        joint_state.name[4] ="Link4";
        joint_state.position[4] = webots_mem.q[0][4];
        joint_state.velocity[4] = webots_mem.dq[0][4];
        joint_state.effort[4] = webots_mem.tau[0][4];    
        joint_state.name[5] ="Link5";
        joint_state.position[5] = webots_mem.q[0][5];
        joint_state.velocity[5] = webots_mem.dq[0][5];
        joint_state.effort[5] = webots_mem.tau[0][5];    
        joint_state.name[6] ="CAP";
        joint_state.position[6] = webots_mem.cap_rate[0];
        joint_state.velocity[6] = 0;
        joint_state.effort[6] = 0;    
        joint_pub_r.publish(joint_state);
 

        joint_state.header.stamp = ros::Time::now();//left
        joint_state.name.resize(7);
        joint_state.position.resize(7);
        joint_state.velocity.resize(7);
        joint_state.effort.resize(7);
        joint_state.name[0] ="Link0";
        joint_state.position[0] = webots_mem.q[1][0];       
        joint_state.velocity[0] = webots_mem.dq[1][0];
        joint_state.effort[0] = webots_mem.tau[1][0];       
        joint_state.name[1] ="Link1";
        joint_state.position[1] = webots_mem.q[1][1];
        joint_state.velocity[1] = webots_mem.dq[1][1];
        joint_state.effort[1] = webots_mem.tau[1][1];    
        joint_state.name[2] ="Link2";
        joint_state.position[2] = webots_mem.q[1][2];
        joint_state.velocity[2] = webots_mem.dq[1][2];
        joint_state.effort[2] = webots_mem.tau[1][2];    
        joint_state.name[3] ="Link3";
        joint_state.position[3] = webots_mem.q[1][3];
        joint_state.velocity[3] = webots_mem.dq[1][3];
        joint_state.effort[3] = webots_mem.tau[1][3];    
        joint_state.name[4] ="Link4";
        joint_state.position[4] = webots_mem.q[1][4];
        joint_state.velocity[4] = webots_mem.dq[1][4];
        joint_state.effort[4] = webots_mem.tau[1][4];    
        joint_state.name[5] ="Link5";
        joint_state.position[5] = webots_mem.q[1][5];
        joint_state.velocity[5] = webots_mem.dq[1][5];
        joint_state.effort[5] = webots_mem.tau[1][5];    
        joint_state.name[6] ="CAP";
        joint_state.position[6] = webots_mem.cap_rate[1];
        joint_state.velocity[6] = 0;
        joint_state.effort[6] = 0;    
        joint_pub_l.publish(joint_state);


        joint_state.header.stamp = ros::Time::now();//base
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
 
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(7);
        joint_state.position.resize(7);
        joint_state.velocity.resize(7);
        joint_state.effort.resize(7);
        joint_state.name[0] ="Link0";
        joint_state.position[0] = webots_mem.q_exp[0][0];       
        joint_state.velocity[0] = webots_mem.dq[0][0];
        joint_state.effort[0] = webots_mem.tau[0][0];       
        joint_state.name[1] ="Link1";
        joint_state.position[1] = webots_mem.q_exp[0][1];
        joint_state.velocity[1] = webots_mem.dq[0][1];
        joint_state.effort[1] = webots_mem.tau[0][1];    
        joint_state.name[2] ="Link2";
        joint_state.position[2] = webots_mem.q_exp[0][2];
        joint_state.velocity[2] = webots_mem.dq[0][2];
        joint_state.effort[2] = webots_mem.tau[0][2];    
        joint_state.name[3] ="Link3";
        joint_state.position[3] = webots_mem.q_exp[0][3];
        joint_state.velocity[3] = webots_mem.dq[0][3];
        joint_state.effort[3] = webots_mem.tau[0][3];    
        joint_state.name[4] ="Link4";
        joint_state.position[4] = webots_mem.q_exp[0][4];
        joint_state.velocity[4] = webots_mem.dq[0][4];
        joint_state.effort[4] = webots_mem.tau[0][4];    
        joint_state.name[5] ="Link5";
        joint_state.position[5] = webots_mem.q_exp[0][5];
        joint_state.velocity[5] = webots_mem.dq[0][5];
        joint_state.effort[5] = webots_mem.tau[0][5];    
        joint_state.name[6] ="CAP";
        joint_state.position[6] = webots_mem.cap_rate[0];
        joint_state.velocity[6] = 0;
        joint_state.effort[6] = 0;    
        joint_pub_r_exp.publish(joint_state);
 

        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(7);
        joint_state.position.resize(7);
        joint_state.velocity.resize(7);
        joint_state.effort.resize(7);
        joint_state.name[0] ="Link0";
        joint_state.position[0] = webots_mem.q_exp[1][0];       
        joint_state.velocity[0] = webots_mem.dq[1][0];
        joint_state.effort[0] = webots_mem.tau[1][0];       
        joint_state.name[1] ="Link1";
        joint_state.position[1] = webots_mem.q_exp[1][1];
        joint_state.velocity[1] = webots_mem.dq[1][1];
        joint_state.effort[1] = webots_mem.tau[1][1];    
        joint_state.name[2] ="Link2";
        joint_state.position[2] = webots_mem.q_exp[1][2];
        joint_state.velocity[2] = webots_mem.dq[1][2];
        joint_state.effort[2] = webots_mem.tau[1][2];    
        joint_state.name[3] ="Link3";
        joint_state.position[3] = webots_mem.q_exp[1][3];
        joint_state.velocity[3] = webots_mem.dq[1][3];
        joint_state.effort[3] = webots_mem.tau[1][3];    
        joint_state.name[4] ="Link4";
        joint_state.position[4] = webots_mem.q_exp[1][4];
        joint_state.velocity[4] = webots_mem.dq[1][4];
        joint_state.effort[4] = webots_mem.tau[1][4];    
        joint_state.name[5] ="Link5";
        joint_state.position[5] = webots_mem.q_exp[1][5];
        joint_state.velocity[5] = webots_mem.dq[1][5];
        joint_state.effort[5] = webots_mem.tau[1][5];    
        joint_state.name[6] ="CAP";
        joint_state.position[6] = webots_mem.cap_rate[1];
        joint_state.velocity[6] = 0;
        joint_state.effort[6] = 0;    
        joint_pub_l_exp.publish(joint_state);


 

		int width = 640, height = 480;
		Mat image(height, width, CV_8UC3);

		// 遍历每个像素点并设置其值为蓝色（BGR格式）head
		for (int y = 0; y < height; ++y) {
			for (int x = 0; x < width; ++x) {
				 
				image.at<Vec3b>(y, x)[0] = webots_mem.pic_r[x][y];      // B
				image.at<Vec3b>(y, x)[1] = webots_mem.pic_g[x][y];      // G
				image.at<Vec3b>(y, x)[2] = webots_mem.pic_b[x][y];      // R
			}
		}
		 
		// 创建一个CvBridge对象
		cv_bridge::CvImage cv_image;
		// 将OpenCV图像转换为ROS图像消息
		cv::Mat imageResize;
		// cv::resize(image, imageResize, cv::Size(), 0.5, 0.5);
		// cv_image.image = imageResize;
		cv_image.image = image;
		cv_image.encoding = "bgr8";
		cv_image.header.stamp = ros::Time::now();
		auto msg = cv_image.toImageMsg();
		image_pub.publish(msg);

		///---------------------right hand
 		// 遍历每个像素点并设置其值为蓝色（BGR格式） 
		for (int y = 0; y < height; ++y) {
			for (int x = 0; x < width; ++x) {
				 
				image.at<Vec3b>(y, x)[0] = webots_mem.pic_rr[x][y];   // B
				image.at<Vec3b>(y, x)[1] = webots_mem.pic_gr[x][y];      // G
				image.at<Vec3b>(y, x)[2] = webots_mem.pic_br[x][y];      // R
			}
		}

		cv_image.image = image;
		cv_image.encoding = "bgr8";
		cv_image.header.stamp = ros::Time::now();
		msg = cv_image.toImageMsg();
		image_pubr.publish(msg);
        //left
		for (int y = 0; y < height; ++y) {
			for (int x = 0; x < width; ++x) {
				 
				image.at<Vec3b>(y, x)[0] = webots_mem.pic_rl[x][y];   // B
				image.at<Vec3b>(y, x)[1] = webots_mem.pic_gl[x][y];      // G
				image.at<Vec3b>(y, x)[2] = webots_mem.pic_bl[x][y];      // R
			}
		}
	    cv_image.image = image;
		cv_image.encoding = "bgr8";
		cv_image.header.stamp = ros::Time::now();
		msg = cv_image.toImageMsg();
		image_publ.publish(msg);


        //----------laser
        sensor_msgs::LaserScan scan;
        scan.header.stamp=ros::Time::now();
        scan.header.frame_id="lidar_link";
        scan.angle_min=-1.57;
        scan.angle_max=1.57;
        scan.angle_increment=3.14/laser_num;
        scan.time_increment=(1/laser_f)/laser_num;
        scan.range_min=0.05;
        scan.range_max=8;

        scan.ranges.resize(laser_num);
        scan.intensities.resize(laser_num);
        for(int i=0;i<laser_num;i++){
            scan.ranges[i]=webots_mem.lidar_dis[i];
            if(scan.ranges[i]<scan.range_min)
                scan.intensities[i]=0;
            else
                scan.intensities[i]=1;
        }
        laser_pub.publish(scan);
        

        //-------------Odom
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
    ros::init(argc, argv, "webots_ros");
	pthread_t tida, tidb,tidc;
    pthread_create(&tida, NULL, Thread_ROS, NULL);
    #if 1
    pthread_create(&tidb, NULL, Thread_MEM, NULL);
    #else
    pthread_create(&tidb, NULL, Thread_UDP_WEBOTS1, NULL);
    pthread_create(&tidc, NULL, Thread_UDP_WEBOTS2, NULL);
    #endif
    pthread_join(tida, NULL);
    pthread_join(tidb, NULL);
    pthread_join(tidc, NULL);
}
