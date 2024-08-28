
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include "wbInterface.h"
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

#if RUN_WEBOTS
#include <webots/Robot.hpp>
#include <webots/supervisor.h>
#define EN_GAIT 1

using namespace webots;
using namespace Eigen;
using namespace std;
using namespace qpOASES;
Robot *robotm = new Robot();
_SIM_DATA _sim_webots;


#include <iostream>
#include <fstream>
#include "spi_node.h"
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

      update_lidar();
      static int cnt_cap = 0;
      static int cap_num = 0;
      if (cnt_cap++ > 1&&1) {//记录相机
            cnt_cap = 0;

            int width=wb_camera_get_width(RGB_F);
            int height=wb_camera_get_height(RGB_F);
            //printf("%d %d\n",width,height);
            int pic_r[width][height];
            int pic_g[width][height];
            int pic_b[width][height];

            const unsigned char *image;
            image = wb_camera_get_image(RGB_F);
            for (int i = 0; i < width; i++) {
                for (int j = 0; j < height; j++) {
                    webots_mem.pic_r[i][j]=pic_r[i][j] = wb_camera_image_get_red(image, width, i, j);
                    webots_mem.pic_g[i][j]=pic_g[i][j] = wb_camera_image_get_green(image, width, i, j);
                    webots_mem.pic_b[i][j]=pic_b[i][j] = wb_camera_image_get_blue(image, width, i, j);
                }
            }
            uint8_t* data = new uint8_t[width * height * 3]; // RGB24格式像素数据
            for (int32_t x = 0; x < width; ++x) {
               for (int32_t y = 0; y < height; ++y) {
                   int32_t index = (y * width + x) * 3;
                   data[index + 0] = pic_r[x][y]; // R
                   data[index + 1] = pic_g[x][y]; // G
                   data[index + 2] = pic_b[x][y]; // B
                }
            }

            //write_bmp("/home/tinymal/文档/tinystrech/controllers/tinyscretch/Camera/head.bmp", data, 640, 480);
            //---
            int width1=wb_camera_get_width(RGB_H0);
            int height1=wb_camera_get_height(RGB_H0);
            //printf("%d %d\n",width,height);
            int pic_r1[width1][height1];
            int pic_g1[width1][height1];
            int pic_b1[width1][height1];

            const unsigned char *image1;
            image1 = wb_camera_get_image(RGB_H0);
            for (int i = 0; i < width1; i++) {
                for (int j = 0; j < height1; j++) {
                      webots_mem.pic_r1[i][j]=pic_r1[i][j] = wb_camera_image_get_red(image1, width1, i, j);
                      webots_mem.pic_g1[i][j]=pic_g1[i][j] = wb_camera_image_get_green(image1, width1, i, j);
                      webots_mem.pic_b1[i][j]=pic_b1[i][j] = wb_camera_image_get_blue(image1, width1, i, j);
                }
            }
            uint8_t* data1 = new uint8_t[width1 * height1 * 3]; // RGB24格式像素数据
            for (int32_t x = 0; x < width1; ++x) {
               for (int32_t y = 0; y < height1; ++y) {
                   int32_t index = (y * width1 + x) * 3;
                   data1[index + 0] = pic_r1[x][y]; // R
                   data1[index + 1] = pic_g1[x][y]; // G
                   data1[index + 2] = pic_b1[x][y]; // B
                }
            }

            //write_bmp("/home/tinymal/文档/tinystrech/controllers/tinyscretch/Camera/hand0.bmp", data1, 640, 480);

            //2
            int width2=wb_range_finder_get_width(RANGE_F);
            int height2=wb_range_finder_get_height(RANGE_F);

            int pic_r2[640][480];
            int pic_g2[640][480];
            int pic_b2[640][480];

            const float *image2;
            image2 = wb_range_finder_get_range_image(RANGE_F);

            for (int i = 0; i < 640; i++) {
                for (int j = 0; j < 480; j++) {
                      webots_mem.pic_r2[i][j]=pic_r2[i][j] = wb_range_finder_image_get_depth(image2, width2, i, j)/8*1000;
                      webots_mem.pic_g2[i][j]=pic_g2[i][j] = wb_range_finder_image_get_depth(image2, width2, i, j)/8*1000;
                      webots_mem.pic_b2[i][j]=pic_b2[i][j] = wb_range_finder_image_get_depth(image2, width2, i, j)/8*1000;
                }
            }
            uint8_t* data2 = new uint8_t[640 * 480 * 3]; // RGB24格式像素数据
            for (int32_t x = 0; x < 640; x++) {
               for (int32_t y = 0; y < 480; y++) {
                   int32_t index = (y * 640 + x) * 3;
                   data2[index + 0] =   webots_mem.pic_r2[x][y]; // R
                   data2[index + 1] =   webots_mem.pic_g2[x][y]; // G
                   data2[index + 2] =   webots_mem.pic_b2[x][y]; // B
                }
            }

            //write_bmp("/home/tinymal/文档/tinystrech/controllers/tinyscretch/Camera/hand1.bmp", data2, 640, 480);


      }
#if 0
      WbNodeRef flag[20];
      WbFieldRef flag_t[20];
      flag[0] = wb_supervisor_node_get_from_def("can0");
      flag_t[0] = wb_supervisor_node_get_field(flag[0], "translation");
      const double *flag_translation;
      flag_translation = wb_supervisor_node_get_position(flag[0]);
      _sim_webots.can_pos[0].x=flag_translation[0];
      _sim_webots.can_pos[0].y=flag_translation[2];
      _sim_webots.can_pos[0].z=flag_translation[1];
      //printf("cube %f %f %f\n",_sim_webots.can_pos[0].x,_sim_webots.can_pos[0].y,_sim_webots.can_pos[0].z);

      flag[0] = wb_supervisor_node_get_from_def("base_cog");
      flag_t[0] = wb_supervisor_node_get_field(flag[0], "translation");
      flag_translation = wb_supervisor_node_get_position(flag[0]);
      _sim_webots.base_pos.x=flag_translation[0];
      _sim_webots.base_pos.y=flag_translation[2];
      _sim_webots.base_pos.z=flag_translation[1];
      //printf("base %f %f %f\n",_sim_webots.base_pos.x,_sim_webots.base_pos.y,_sim_webots.base_pos.z);

      flag[0] = wb_supervisor_node_get_from_def("chair0");
      flag_t[0] = wb_supervisor_node_get_field(flag[0], "translation");
      flag_translation = wb_supervisor_node_get_position(flag[0]);
      _sim_webots.chair_pos[0].x=flag_translation[0];
      _sim_webots.chair_pos[0].y=flag_translation[2];
      _sim_webots.chair_pos[0].z=flag_translation[1];
#endif
      WbNodeRef flag[20];
      WbFieldRef flag_t[20];
      flag[0] = wb_supervisor_node_get_from_def("Sense");
      flag_t[0] = wb_supervisor_node_get_field(flag[0], "translation");
      const double *flag_translation;
      flag_translation = wb_supervisor_node_get_position(flag[0]);
      _sim_webots.base_pos.x=flag_translation[0];
      _sim_webots.base_pos.y=flag_translation[2];
      _sim_webots.base_pos.z=flag_translation[1];
      //printf("base_pos %f %f %f\n",_sim_webots.base_pos.x,_sim_webots.base_pos.y,_sim_webots.base_pos.z);
}

void* Thread_main(void*)//OCU UDP通讯线程  as master
{
    static float init_cnt = 0;
    static int init_done = 0;
    int i = 0;
    static float timer[10] = { 0 };
    float dT = TIME_STEP / 1000.;

    printf("Thread TinySctrech\n");
    while (wb_robot_step(TIME_STEP) != -1) {//5MS
        init_cnt += dT;

        Simluation_data(dT);

        if (1) {
            updateJoy();

            locomotion_sfm(dT);

            subscribe_imu_to_webot(&robotwb, dT);//赋值给robot结构体

            state_estimator(dT);//估计质心状态

            readAllMotorPos(&robotwb, dT);//读取角度

            estimate_end_state_new(dT);	//运动学正解

            state_estimator(dT);

            body_servo_control(dT);

            force_control_and_dis_pose(dT);//位力混控 站立版本

            set_motor_arm();
            set_motor_head();
            set_motor_w();
            set_motor_cap();
        }


  #if !EN_GAIT
  #if 1//验证坐标系和角度反馈
      #if 1
        END_POS end_pos;
        end_pos.x=0.45;
        end_pos.y=0;
        end_pos.z=0.15;//base =0.1
        END_POS end_att;
        end_att.x=0;
        end_att.y=0/57.3;//pit
        end_att.z=0/57.3;
        float q_out[6];
        ik_pino(end_pos,end_att,q_out);
        robotwb.arm_base_height_exp= q_out[0];
        robotwb.arm_q_exp[0][0] = q_out[1]*57.3;//
        robotwb.arm_q_exp[0][1] = q_out[2]*57.3;//
        robotwb.arm_q_exp[0][2] = q_out[3]*57.3;//
        robotwb.arm_q_exp[0][3] = q_out[4]*57.3;//
        robotwb.arm_q_exp[0][4] = q_out[5]*57.3;//
      #else
          robotwb.arm_q_exp[0][0] = 45;//
          robotwb.arm_q_exp[0][1] = 89;//
          robotwb.arm_q_exp[0][2] = 45;//
          robotwb.arm_q_exp[0][3] = -89;//
          robotwb.arm_q_exp[0][4] = 45;//
      #endif
          robotwb.head_att_exp[0] = -15;
          robotwb.head_att_exp[1] = -45;

          robotwb.wheel_dq_exp[0] = 100;
          robotwb.wheel_dq_exp[1] = 100;
          robotwb.wheel_dq_exp[2] = 100;
          robotwb.wheel_dq_exp[3] = 100;

          set_motor_arm();
          set_motor_head();
          set_motor_w();
  #endif
      //
      #if 0//验证运动学和Jacobi
        static float timer=0, timer1=0;
        static int test_state = 0;
        float q_inv[3];
        int m_id = 0;
          switch (test_state)
          {
          case 0:
              timer += 0.001*TIME_STEP;

              for (m_id = 0; m_id < 4; m_id++) {
                  robotwb.Leg[m_id].tar_sita[0] = -25;//大腿
                  robotwb.Leg[m_id].tar_sita[1] = 90;//小腿
                  robotwb.Leg[m_id].tar_sita[2] = 5 * robotwb.Leg[m_id].flag_rl;//侧展
                  robotwb.Leg[m_id].tar_sita[3] = 5 * robotwb.Leg[m_id].flag_rl;//航向 +顺时针
                  robotwb.Leg[m_id].tar_sita[4] = 42;//足端  +向下压
                  set_motor_q(m_id); robotwb.Leg[m_id].epos_h_reg = robotwb.Leg[m_id].tar_epos_h;
                  for (int j = 0; j < 5; j++)
                      robotwb.Leg[m_id].tar_sita_d[j] = 0;
              }
              if (timer > 1) { timer = 0; test_state++;

              printf("ss\n"); }
          break;
          case 1:
              m_id = 0;
              timer1 += 0.01;
              for (m_id = 0; m_id < 4; m_id++) {

                  robotwb.Leg[m_id].tar_espd_h.x = 5;
                  //robotwb.Leg[m_id].tar_espd_h.y = 10;
                  espd_to_neg_dq(m_id, 0.001*TIME_STEP);
                  for (int j = 0; j < 3; j++)
                      robotwb.Leg[m_id].tar_sita[j] += robotwb.Leg[m_id].tar_sita_d[j] * 0.001*TIME_STEP;
                  if (m_id == 1&&0) {
                      printf("id=%d %f %f %f\n", m_id, robotwb.Leg[m_id].tar_sita[0], robotwb.Leg[m_id].tar_sita[1], robotwb.Leg[m_id].tar_sita[2]);
                      printf("id=%d %f %f %f\n", m_id, robotwb.Leg[m_id].tar_sita_d[0], robotwb.Leg[m_id].tar_sita_d[1], robotwb.Leg[m_id].tar_sita_d[2]);
                  }
                  set_motor_q(m_id);
              }
          break;
          }
      #endif

        #if 0//验证Inv运动学
          static float timer = 0;
          static int test_state = 0;
          int m_id = 0;
          float q_inv[3];
          static float sin_temp = 0;
          static float timer1 = 0;
          static float x_ff=0;
          switch (test_state)
          {
          case 0:
              timer += 0.001*TIME_STEP;
              robotwb.arm_base_height_exp = 0.12;
              robotwb.arm_q_exp[0][0] = 45;//
              robotwb.arm_q_exp[0][1] = 89;//
              robotwb.arm_q_exp[0][2] = 45;//
              robotwb.arm_q_exp[0][3] = -89;//
              robotwb.arm_q_exp[0][4] = 45;//

              robotwb.head_att_exp[0] = -15;
              robotwb.head_att_exp[1] = -45;

              robotwb.wheel_dq_exp[0] = 0;
              robotwb.wheel_dq_exp[1] = 0;
              robotwb.wheel_dq_exp[2] = 0;
              robotwb.wheel_dq_exp[3] = 0;
              END_POS end_pos;
              end_pos.x=0.1;
              end_pos.y=-0.15;
              end_pos.z=0;//base =0.1
              END_POS end_att;
              end_att.x=0;
              end_att.y=0/57.3;//pit
              end_att.z=0/57.3;
              float q_out[6];
              ik_pino(end_pos,end_att,q_out);
              robotwb.arm_base_height_exp= q_out[0];
              robotwb.arm_q_exp[0][0] = q_out[1]*57.3;//
              robotwb.arm_q_exp[0][1] = q_out[2]*57.3;//
              robotwb.arm_q_exp[0][2] = q_out[3]*57.3;//
              robotwb.arm_q_exp[0][3] = q_out[4]*57.3;//
              robotwb.arm_q_exp[0][4] = q_out[5]*57.3;//

              set_motor_arm();
              set_motor_head();
              set_motor_w();
              if (timer > 1) { timer = 0; test_state++; printf("ss\n"); }
              break;
          case 1:
              m_id = 0;
              timer += 0.01;
              timer1 += 0.01*30;

              END_POS end_pos1;
              robotwb.wheel_dq_exp[0] = sind(timer1*1.5)*100.05;
              robotwb.wheel_dq_exp[1] = sind(timer1*1.5)*100.05;
              robotwb.wheel_dq_exp[2] = sind(timer1*1.5)*100.05;
              robotwb.wheel_dq_exp[3] = sind(timer1*1.5)*100.05;
              x_ff+=robotwb.wheel_dq_exp[0]*0.00005*0.08;
              end_pos1.x=0.45+sind(timer1)*0.05*0-x_ff;
              end_pos1.y=-0.125+sind(timer1)*0.1*0;
              end_pos1.z=0.1+0.08;//base =0.1
              END_POS end_att1;
              end_att1.x=(sind(timer1)*10)/57.3;
              end_att1.y=(sind(timer1)*25)/57.3*1;//pit
              end_att1.z=(sind(timer1)*15)/57.3;

              printf("exp_att=%f %f %f\n",end_att1.x*57.3,end_att1.y*57.3,end_att1.z*57.3);
              float q_out1[6];
              ik_pino(end_pos1,end_att1,q_out1);
              robotwb.arm_base_height_exp= q_out1[0];
              robotwb.arm_q_exp[0][0] = q_out1[1]*57.3;//
              robotwb.arm_q_exp[0][1] = q_out1[2]*57.3;//
              robotwb.arm_q_exp[0][2] = q_out1[3]*57.3;//
              robotwb.arm_q_exp[0][3] = q_out1[4]*57.3;//
              robotwb.arm_q_exp[0][4] = q_out1[5]*57.3;//

              float q_in[6];
              END_POS end_pos_now,end_att_now;
              q_in[0]=robotwb.arm_base_height;
              q_in[1]=robotwb.arm_q[0][0];
              q_in[2]=robotwb.arm_q[0][1];
              q_in[3]=robotwb.arm_q[0][2];
              q_in[4]=robotwb.arm_q[0][3];
              q_in[5]=robotwb.arm_q[0][4];
              fk_pino(q_in, &end_pos_now, &end_att_now);

              //printf("exp=%f %f %f\n",end_pos1.x,end_pos1.y,end_pos1.z);
              //printf("now=%f %f %f\n",end_pos_now.x,end_pos_now.y,end_pos_now.z);
              set_motor_arm();
              set_motor_head();
              set_motor_w();
              break;
          }

        #endif

        #if 0//验证力矩 力矩与角速度+方向一致
            static float timer = 0;
            static int test_state = 0;
            int m_id = 0;
            switch (test_state)
            {
            case 0:
                timer += 0.001*TIME_STEP;

                for (m_id = 0; m_id < 4; m_id++) {
  #if 1//标准站立
                    robotwb.Leg[m_id].tar_sita[0] = 45;//大腿
                    robotwb.Leg[m_id].tar_sita[1] = 90;//小腿
                    robotwb.Leg[m_id].tar_sita[2] = 5 * robotwb.Leg[m_id].flag_rl;//侧展
                    robotwb.Leg[m_id].tar_sita[3] = 5 * robotwb.Leg[m_id].flag_rl;//航向 +顺时针
                    robotwb.Leg[m_id].tar_sita[4] = 42;//足端  +向下抬
  #endif
                    set_motor_q(m_id);
                    for (int j = 0; j < 5; j++)
                        robotwb.Leg[m_id].tar_sita_d[j] = 0;
                }
                if (timer > 1) { timer = 0; test_state++; }
                break;
            case 1:
                m_id = 0;
                for (m_id = 0; m_id < 4; m_id++) {
                    robotwb.Leg[m_id].taod[0] = -0.65 * 0;//大腿
                    robotwb.Leg[m_id].taod[1] = -0.45 * 0;//小腿
                    robotwb.Leg[m_id].taod[2] = 1.5 * 0;//侧展
                    robotwb.Leg[m_id].taod[3] = 0.5 * 0;//航向
                    robotwb.Leg[m_id].taod[4] = -0.5 * 0;//足端
                    set_motor_t(m_id);
                }
                break;
            }
        #endif
  #endif
    }//end while

      delete robotm;
}
#endif

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

struct robot_msg
{
   float arm_q_l[6];
   float arm_q_r[6];
   float cap_rate[2];
   float base_vel[3];
} _robot_msg;

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

void* Thread_UDP_SDK(void*)//OCU UDP通讯线程  作为 服务器
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
    printf("Thread UDP imitate\n");
    while (1)
    {
        //读取客户端 读取操控指令
        loss_cnt_sdk+=0.005;
        _robot_record_msg.start_record=robotwb.start_record;
        _robot_record_msg.reach_waypoint=robotwb.reach_waypoint;
        robotwb.record_mode=_remote_msg_record.mode;

        if(loss_cnt_sdk>1&&(robotwb.arm_control_mode==98||robotwb.arm_control_mode==99||recording_flag==1)){
            robotwb.arm_control_mode=ARM_M_H;//sdk
            robotwb.wheel_control_mode=0;//sdk
            robotwb.head_control_mode=HEAD_FREE;
            robotwb.head_att_exp[0]=robotwb.head_att_exp[1]=0;
            robotwb.arm_epos_b_exp.z=0;
            robotwb.arm_epos_e_exp.z=0;
            robotwb.arm_epos_n_exp.z=0;
            recording_flag=0;
            robotwb.start_record=robotwb.record_mode=0;
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

                if(recording_flag&&(_remote_msg_record.record_rate>0.01||robotwb.start_record==0))
                {
                    if(cnt_p++>25){cnt_p=0;
                    printf("SDK Comman Recording:%.1f%\n",_remote_msg_record.record_rate*100);
                    }
                    if(_remote_msg_record.record_rate>0.98||robotwb.start_record==0){
                        recording_flag=0;
                        _remote_msg_record.record_rate=0;
                        _robot_record_msg.start_record=robotwb.start_record=0;
                        printf("SDK Finish Recording!\n");
                    }
                }
                memcpy(send_buf,&_robot_record_msg,sizeof(_robot_record_msg));
                send_num = sendto(sock_fd, send_buf, sizeof(_robot_record_msg), MSG_DONTWAIT, (struct sockaddr *)&addr_client, len);
            }
            else if(recv_num==135){//epos replay--unused now
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
            }else  if(recv_num==72){//joint replay
                memcpy(&_remote_msg_joint,recv_buf,sizeof(_remote_msg_joint));
                loss_cnt_sdk=0;
                if(robotwb.arm_control_mode!=99)
                {
                    robotwb.arm_control_mode=99;//sdk
                    robotwb.wheel_control_mode=99;//sdk
                    robotwb.head_control_mode=HEAD_LOOK_B;
                    printf("SDK joint mode enable!\n");
                }

                robotwb.arm_q_exp_sdk[0][0]=_remote_msg_joint.q_exp_r[0];//base_z
                robotwb.arm_q_exp_sdk[0][1]=_remote_msg_joint.q_exp_r[1];
                robotwb.arm_q_exp_sdk[0][2]=_remote_msg_joint.q_exp_r[2];
                robotwb.arm_q_exp_sdk[0][3]=_remote_msg_joint.q_exp_r[3];
                robotwb.arm_q_exp_sdk[0][4]=_remote_msg_joint.q_exp_r[4];
                robotwb.arm_q_exp_sdk[0][5]=_remote_msg_joint.q_exp_r[5];

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
            }else if(recv_num==8){//wwaypont
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
            _robot_msg.arm_q_r[0]=robotwb.arm_base_height;
            _robot_msg.arm_q_r[1]=robotwb.arm_q[0][0];
            _robot_msg.arm_q_r[2]=robotwb.arm_q[0][1];
            _robot_msg.arm_q_r[3]=robotwb.arm_q[0][2];
            _robot_msg.arm_q_r[4]=robotwb.arm_q[0][3];
            _robot_msg.arm_q_r[5]=robotwb.arm_q[0][4];

            _robot_msg.arm_q_l[0]=robotwb.arm_base_height;
            _robot_msg.arm_q_l[1]=robotwb.arm_q[0][0];
            _robot_msg.arm_q_l[2]=robotwb.arm_q[0][1];
            _robot_msg.arm_q_l[3]=robotwb.arm_q[0][2];
            _robot_msg.arm_q_l[4]=robotwb.arm_q[0][3];
            _robot_msg.arm_q_l[5]=robotwb.arm_q[0][4];

            _robot_msg.cap_rate[0]=robotwb.cap_set;
            _robot_msg.cap_rate[1]=robotwb.cap_set;
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
        usleep(5*1000);
    }
    close(sock_fd);
    return 0;
}


int main(int argc, char **argv) {
 
#if RUN_WEBOTS
  webots_device_init();
#endif
  vmc_param_init();

  record_thread(0, 0.005);
  record_thread(1, 0.005);
  recorder(0.005);
#if RUN_WEBOTS
  pthread_t tida,tidb,tidc,tidd;
  pthread_create(&tida, NULL, Thread_UDP_SDK, NULL);
  pthread_create(&tidb, NULL, Thread_main, NULL);
#if 1
  pthread_create(&tidc, NULL, Thread_Mem_Webots, NULL);
#else
  pthread_create(&tidc, NULL, Thread_UDP_Webots1, NULL);
  pthread_create(&tidd, NULL, Thread_UDP_Webots2, NULL);
#endif
  pthread_join(tida, NULL);
  pthread_join(tidb, NULL);
  pthread_join(tidc, NULL);
  pthread_join(tidd, NULL);
#endif
}

