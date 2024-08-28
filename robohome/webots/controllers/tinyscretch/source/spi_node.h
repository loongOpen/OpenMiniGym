#ifndef _SPI_NODE_H_
#define _SPI_NODE_H_
#include "base_struct.h"
extern int mem_connect;
extern int mem_connect_c;

typedef struct{
    int pic_r[640][480];
    int pic_g[640][480];
    int pic_b[640][480];

    int pic_r1[640][480];
    int pic_g1[640][480];
    int pic_b1[640][480];

    int pic_r2[640][480];
    int pic_g2[640][480];
    int pic_b2[640][480];

    float q[2][6];
    float dq[2][6];
    float tau[2][6];
    float cap_rate[2];
    float base_vel[3];

    float lidar_dis[360];
    float att[3];
    int save_data;
}_Webots;

extern _Webots webots_mem;
#define MEM_SPI 0001
#define MEM_SIZE (640*480*2*3*3+50*4+360*2)

extern pthread_mutex_t lock;
void* Thread_Mem_Webots(void*);//内存管理线程
#endif
