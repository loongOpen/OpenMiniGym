#ifndef _SPI_NODE_H_
#define _SPI_NODE_H_

#define SPI_BUF_SIZE 255
#define CAN_T_DIV 500.0
#define CAN_I_DIV 100.0
#define CAN_F_DIV 100.0
#define CAN_POS_DIV 50.0
#define CAN_DPOS_DIV 20.0
#define CAN_GAIN_DIV_P 500.0
#define CAN_GAIN_DIV_I 10000.0
#define CAN_GAIN_DIV_D 1000.0


#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


typedef struct
{
    float x;
    float y;
    float z;
    float zz;
}END_POS;
 
typedef struct{
    int pic_r[640][480];
    int pic_g[640][480];
    int pic_b[640][480];

    int pic_rr[640][480];
    int pic_gr[640][480];
    int pic_br[640][480];

    int pic_rl[640][480];
    int pic_gl[640][480];
    int pic_bl[640][480];

    float q_exp[2][7];
    float cap_rate_exp[2];

    float q[2][7];
    float dq[2][7];
    float tau[2][7];
    float cap_rate[2];
    float base_vel[3];
    float lidar_dis[360];
    int save_data;

    float att[3];
    float epos[2][6];
    float epos_exp[2][6];
}_Webots;

extern _Webots webots_mem;

#endif
