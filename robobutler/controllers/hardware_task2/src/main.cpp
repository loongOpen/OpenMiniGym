#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include "comm.h"
#include "spi_node.h"
#include "spi.h"
#include "sys_time.h"
#include <pthread.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/parse.h>

YAML::Node config_hardware=YAML::LoadFile("/home/odroid/Corgi/Param/param_hardware.yaml");

// ps -ef | grep hardware_task
// kill －9 324
_MEMS mems;

#define SPI_TEST 0

#define USE_USB 0
#define USE_SERIAL 0

#define EN_SPI_BIG 1
#define CAN_LINK_COMM_VER1 0
#define CAN_LINK_COMM_VER2 1//3 BLDC Param DIV
#if EN_SPI_BIG
#if CAN_LINK_COMM_VER1
    #define SPI_SEND_MAX  85
#else
    #define SPI_SEND_MAX  120+30+20//equal to stm32 spi send cnt
#endif
#else
#define SPI_SEND_MAX  40//40
#endif
#define EN_MULTI_THREAD 1
#define NO_THREAD 0
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MEM_SPI 0001
#define MEM_SIZE 2048
#define EN_DBG_PRINT 0

#if NO_THREAD&&!EN_MULTI_THREAD
static uint32_t speed = 150000*4;//3Mhz 死机
#define DELAY_SPI 250//us
#else
static uint32_t speed = 20000000;//odroid 20M  upbord 2M  raspberrypi 5M
#define DELAY_SPI 250//us
#endif
float spi_loss_cnt = 0;
int spi_connect = 0;
float mems_usb_loss_cnt = 0;
int mems_usb_connect = 0;

using namespace std;

_SPI_RX spi_rx;
_SPI_TX spi_tx;

uint8_t spi_tx_buf[SPI_BUF_SIZE] = {0};
uint8_t spi_rx_buf[SPI_BUF_SIZE] = {0};
int spi_tx_cnt_show=0;
int spi_tx_cnt = 0;
int spi_rx_cnt = 0;

uint8_t usb_tx_buf[SPI_BUF_SIZE] = {0};
uint8_t usb_rx_buf[SPI_BUF_SIZE] = {0};
int usb_tx_cnt = 0;
int usb_rx_cnt = 0;

uint8_t tx[SPI_BUF_SIZE] = {};
uint8_t rx[ARRAY_SIZE(tx)] = {};

int mem_connect=0;
int brain_connect_c=0;
float mem_loss_cnt=0;
struct shareMemory
{
    int  flag=0;  //作为一个标志，非0：表示可读，0表示可写
    unsigned char szMsg[MEM_SIZE];
};
struct shareMemory shareMemory_spi;

static void setDataInt_spi(int i)
{
    spi_tx_buf[spi_tx_cnt++] = ((i << 24) >> 24);
    spi_tx_buf[spi_tx_cnt++] = ((i << 16) >> 24);
    spi_tx_buf[spi_tx_cnt++] = ((i << 8) >> 24);
    spi_tx_buf[spi_tx_cnt++] = (i >> 24);
}

static void setDataFloat_spi(float f)
{
    int i = *(int *)&f;
    spi_tx_buf[spi_tx_cnt++] = ((i << 24) >> 24);
    spi_tx_buf[spi_tx_cnt++] = ((i << 16) >> 24);
    spi_tx_buf[spi_tx_cnt++] = ((i << 8) >> 24);
    spi_tx_buf[spi_tx_cnt++] = (i >> 24);
}

static void setDataFloat_spi_int(float f,float size)
{
    int16_t _temp;
    _temp=f*size;
    spi_tx_buf[spi_tx_cnt++] = BYTE1(_temp);
    spi_tx_buf[spi_tx_cnt++] = BYTE0(_temp);
}

static float floatFromData_spi(unsigned char *data, int *anal_cnt)
{
    int i = 0x00;
    i |= (*(data + *anal_cnt + 3) << 24);
    i |= (*(data + *anal_cnt + 2) << 16);
    i |= (*(data + *anal_cnt + 1) << 8);
    i |= (*(data + *anal_cnt + 0));

    *anal_cnt += 4;
    return *(float *)&i;
}

static float floatFromData_spi_int(unsigned char *data, int *anal_cnt,float size)
{
    float temp=0;
    temp=(float)((int16_t)(*(data + *anal_cnt + 0)<<8)|*(data + *anal_cnt + 1))/size;
    *anal_cnt += 2;
    return temp;
}

static char charFromData_spi(unsigned char *data, int *anal_cnt)
{
    int temp = *anal_cnt;
    *anal_cnt += 1;
    return *(data + temp);
}

static int intFromData_spi(unsigned char *data, int *anal_cnt)
{
    int i = 0x00;
    i |= (*(data + *anal_cnt + 3) << 24);
    i |= (*(data + *anal_cnt + 2) << 16);
    i |= (*(data + *anal_cnt + 1) << 8);
    i |= (*(data + *anal_cnt + 0));
    *anal_cnt += 4;
    return i;
}

float To_180_degrees(float x)
{
    return (x>180?(x-360):(x<-180?(x+360):x));
}

void can_board_send(char sel)//发送到单片机
{
    int i;
    static float t_temp = 0;
    char id = 0;
    char sum_t = 0, _cnt = 0;
    static char bldc_id_sel=0;
    spi_tx_cnt = 0;

    spi_tx_buf[spi_tx_cnt++] = 0xFE;
    spi_tx_buf[spi_tx_cnt++] = 0xFC;
    spi_tx_buf[spi_tx_cnt++] = sel;
    spi_tx_buf[spi_tx_cnt++] = 0;

    switch (sel)
    {
    case 44:// 3BLDC param div---------------------------------send to butler car
        for (int id = 0; id < 4; id++)
        {
            setDataFloat_spi_int(spi_tx.q_set[id][0],CAN_DPOS_DIV_0);//set for dq in butler
            setDataFloat_spi_int(spi_tx.q_set[id][1],CAN_POS_DIV);
            setDataFloat_spi_int(spi_tx.q_set[id][2],CAN_POS_DIV);
            setDataFloat_spi_int(spi_tx.tau_ff[id][0],CAN_T_DIV);
            setDataFloat_spi_int(spi_tx.tau_ff[id][1],CAN_T_DIV);
            setDataFloat_spi_int(spi_tx.tau_ff[id][2],CAN_T_DIV);
            setDataFloat_spi_int(spi_tx.q_reset[id][0],CAN_POS_DIV);
            setDataFloat_spi_int(spi_tx.q_reset[id][1],CAN_POS_DIV);
            setDataFloat_spi_int(spi_tx.q_reset[id][2],CAN_POS_DIV);
            spi_tx_buf[spi_tx_cnt++] =spi_tx.param_sel[id];//chose sw or st param
        }
            setDataFloat_spi_int(spi_tx.t_to_i,1000);
#if 1//--------------------param PVT
            setDataFloat_spi_int(spi_tx.kp,CAN_GAIN_DIV_P);
            setDataFloat_spi_int(spi_tx.ki,CAN_GAIN_DIV_I);
            setDataFloat_spi_int(spi_tx.kd,CAN_GAIN_DIV_D);

            setDataFloat_spi_int(spi_tx.kp_sw_d[bldc_id_sel],CAN_GAIN_DIV_P);
            setDataFloat_spi_int(spi_tx.ki_sw_d[bldc_id_sel],CAN_GAIN_DIV_I);
            setDataFloat_spi_int(spi_tx.kd_sw_d[bldc_id_sel],CAN_GAIN_DIV_D);

            setDataFloat_spi_int(spi_tx.kp_st_d[bldc_id_sel],CAN_GAIN_DIV_P);
            setDataFloat_spi_int(spi_tx.ki_st_d[bldc_id_sel],CAN_GAIN_DIV_I);
            setDataFloat_spi_int(spi_tx.kd_st_d[bldc_id_sel],CAN_GAIN_DIV_D);
#endif
            //printf("bldc_id_sel=%d %f %f %f\n",bldc_id_sel,spi_tx.kp_sw_d[bldc_id_sel],spi_tx.ki_sw_d[bldc_id_sel],spi_tx.kd_sw_d[bldc_id_sel]);

            spi_tx_buf[spi_tx_cnt++] = (spi_tx.max_i);
            spi_tx_buf[spi_tx_cnt++] = spi_tx.en_motor*100+spi_tx.reser_q*10+spi_tx.reset_err;
            spi_tx_buf[spi_tx_cnt++] = bldc_id_sel*100+spi_tx.led_enable[0]*10+spi_tx.led_enable[1];//bldc id sel
            spi_tx_buf[spi_tx_cnt++] = spi_tx.led_side_enable[0];
            spi_tx_buf[spi_tx_cnt++] = spi_tx.led_side_enable[1];//bldc id sel
//OCU param
            setDataFloat_spi_int( mems.imu_att.z,CAN_POS_DIV);
            spi_tx_buf[spi_tx_cnt++] =  mems.Acc_CALIBRATE*100+mems.Gyro_CALIBRATE*10+mems.Mag_CALIBRATE;
            spi_tx_buf[spi_tx_cnt++] =  spi_tx.beep_state;//normal end

            //0 末端俯仰 1末端横滚 2夹持 0云台俯仰 1云台航向
#if 0
            spi_tx.arm_head3.att_set[0]=14;
            spi_tx.arm_head3.att_set[1]=15;
            spi_tx.arm_head3.att_set[2]=15;
            spi_tx.arm_head3.head_set[0]=16;
            spi_tx.arm_head3.head_set[1]=45;
            spi_tx.arm_head3.head_set[2]=3;
            spi_tx.arm_head3.cap=0.5;
#endif
            setDataFloat_spi_int(spi_tx.arm_head3.att_set[0],100);//hand
            setDataFloat_spi_int(spi_tx.arm_head3.att_set[1],100);//
            setDataFloat_spi_int(spi_tx.arm_head3.att_set[2],100);
            setDataFloat_spi_int(spi_tx.arm_head3.head_set[0],100);//head
            setDataFloat_spi_int(spi_tx.arm_head3.head_set[1],100);//
            setDataFloat_spi_int(spi_tx.arm_head3.head_set[2],100);
            setDataFloat_spi_int(spi_tx.arm_head3.cap,100);//cap
            spi_tx_buf[spi_tx_cnt++] = spi_tx.arm_cmd_s.power*10+spi_tx.arm_cmd_s.mode;
#if 0//test for wheel
            spi_tx.w_dq_set[0]=0.72;
            spi_tx.w_dq_set[1]=0.72;
            spi_tx.w_dq_set[2]=0.72;
            spi_tx.w_dq_set[3]=0.72;
#endif
            setDataFloat_spi_int(spi_tx.w_dq_set[0],100);
            setDataFloat_spi_int(spi_tx.w_dq_set[1],100);
            setDataFloat_spi_int(spi_tx.w_dq_set[2],100);
            setDataFloat_spi_int(spi_tx.w_dq_set[3],100);
            setDataFloat_spi_int(spi_tx.w_tau_set[0],CAN_T_DIV);
            setDataFloat_spi_int(spi_tx.w_tau_set[1],CAN_T_DIV);
            setDataFloat_spi_int(spi_tx.w_tau_set[2],CAN_T_DIV);
            setDataFloat_spi_int(spi_tx.w_tau_set[3],CAN_T_DIV);

            spi_tx_buf[spi_tx_cnt++] =mem_connect;
            spi_tx_buf[spi_tx_cnt++] =brain_connect_c;
            bldc_id_sel++;
            if(bldc_id_sel>2)
                bldc_id_sel=0;
    break;

    case 50://发送OCU配置
        //--------------------Param From OCU-------------
        setDataFloat_spi_int( mems.imu_pos.x,1000);
        setDataFloat_spi_int( mems.imu_pos.y,1000);
        setDataFloat_spi_int( mems.imu_pos.z,1000);
        setDataFloat_spi_int( mems.imu_att.x,CAN_POS_DIV);
        setDataFloat_spi_int( mems.imu_att.y,CAN_POS_DIV);
        setDataFloat_spi_int( mems.imu_att.z,CAN_POS_DIV);
        setDataFloat_spi_int( mems.gps_pos.x,1000);
        setDataFloat_spi_int( mems.gps_pos.y,1000);
        setDataFloat_spi_int( mems.gps_pos.z,1000);
        spi_tx_buf[spi_tx_cnt++] =  mems.Acc_CALIBRATE;
        spi_tx_buf[spi_tx_cnt++] =  mems.Gyro_CALIBRATE;
        spi_tx_buf[spi_tx_cnt++] =  mems.Mag_CALIBRATE;

    break;

    default:
        for (int id = 0; id < 4; id++)
        {
            setDataFloat_spi(0);
            setDataFloat_spi(0);
            setDataFloat_spi(0);
            setDataFloat_spi(0);
        }
        break;
    }

    spi_tx_buf[3] = (spi_tx_cnt)-4;
    for (i = 0; i < spi_tx_cnt; i++)
        sum_t += spi_tx_buf[i];
    spi_tx_buf[spi_tx_cnt++] = sum_t;

    if(spi_tx_cnt>SPI_SEND_MAX)
       printf("spi_tx_cnt=%d over flow!!!\n",spi_tx_cnt);
    spi_tx_cnt_show=spi_tx_cnt;
}

float deadw(float x,float zoom)
{
    float t;
    if(x>0)
    {
        t = x - zoom;
        if(t<0)
        {
            t = 0;
        }
    }
    else
    {
        t = x + zoom;
        if(t>0)
        {
            t = 0;
        }
    }
  return (t);
}

void DigitalLPFw(float in, float* out, float cutoff_freq, float dt) {
      float input_reg=in;
    if (cutoff_freq <= 0.0f || dt <= 0.0f) {
        *out = input_reg;
    }
    float rc = 1.0f/(2*3.1415926*cutoff_freq);
    float alpha = (dt/(dt+rc));
    *out += (input_reg - *out) * alpha;
}


int slave_rx(uint8_t *data_buf, int num)//接收解码--------------from stm32 feedback
{
    static int cnt_p = 0;
    static int cnt_err_sum=0;
    uint8_t id;
    uint8_t sum = 0;
    uint8_t i;
    uint8_t temp;
    int anal_cnt = 4;
    for (i = 0; i < (num - 1); i++)
        sum += *(data_buf + i);
    if (!(sum == *(data_buf + num - 1))){
        printf("spi sum err=%d sum_cal=0x%X sum=0x%X !!\n",cnt_err_sum++,sum,*(data_buf + num - 1));
        return 0;
    }
    if (!(*(data_buf) == 0xFF && *(data_buf + 1) == 0xFB)){
        printf("spi head err!!\n");
        return 0;
    }
    if (*(data_buf + 2) == 30) //
    {
        spi_loss_cnt = 0;
        if (spi_connect==0)
        {
            printf("Hardware::Hardware SPI-STM32 Link3-Sbus Yunzhuo!!!=%d!!!\n",spi_connect);
            spi_connect = 1;
        }

        spi_rx.att[0] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.att[1] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.att[2] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        //printf("att0=%f att1=%f att2=%f dt=%f\n",spi_rx.att[0],spi_rx.att[1],spi_rx.att[2], Get_Cycle_T(0));
        spi_rx.att_rate[0] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.att_rate[1] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.att_rate[2] = floatFromData_spi(spi_rx_buf, &anal_cnt);

        spi_rx.acc_b[0] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.acc_b[1] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.acc_b[2] = floatFromData_spi(spi_rx_buf, &anal_cnt);

        for (int i = 0; i < 4; i++)
        {
            spi_rx.dq[i][0] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_DPOS_DIV_0);//rad
            spi_rx.q[i][1] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_POS_DIV);
            spi_rx.q[i][2] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_POS_DIV);
            spi_rx.tau[i][0] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_T_DIV);
            spi_rx.tau[i][1] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_T_DIV);
            spi_rx.tau[i][2] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_T_DIV);

            temp= charFromData_spi(spi_rx_buf, &anal_cnt);
            spi_rx.connect[i] = temp/100;
            spi_rx.connect_motor[i][0] = (temp-spi_rx.connect[i] *100)/10;
            spi_rx.ready[i][0] = temp%10;
            temp= charFromData_spi(spi_rx_buf, &anal_cnt);
            spi_rx.connect[i] = temp/100;
            spi_rx.connect_motor[i][1] = (temp-spi_rx.connect[i] *100)/10;
            spi_rx.ready[i][1] = temp%10;
            temp= charFromData_spi(spi_rx_buf, &anal_cnt);
            spi_rx.connect[i] = temp/100;
            spi_rx.connect_motor[i][2] = (temp-spi_rx.connect[i] *100)/10;
            spi_rx.ready[i][2] = temp%10;
            //printf("%d %d\n", spi_rx.connect_motor[i][2], spi_rx.ready[i][2] );
        }
        float spi_dt = Get_Cycle_T(15);
#if 1//calculate z_height
        static float dq_flt=0;
        DigitalLPFw(spi_rx.dq[0][0], &dq_flt, 50, spi_dt);
        //printf("q_connect:%d%d%d%d\n",spi_rx.connect[0],spi_rx.connect[1],spi_rx.connect[2],spi_rx.w_connect[3]);
        //printf("q: %.3f %.3f %.3f %.3f\n",spi_rx.q[0][0],spi_rx.dq[0][0],spi_rx.q[0][2]);
        //printf("%f %f\n",dq_flt,spi_rx.q[0][0]);
        spi_rx.q[0][0]+=deadw(dq_flt,0.5)*spi_dt;
#endif
        //wheel
        spi_rx.w_dq_now[0] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);
        spi_rx.w_dq_now[1] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);
        spi_rx.w_dq_now[2] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);
        spi_rx.w_dq_now[3] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);

        spi_rx.w_tau_now[0] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_T_DIV);
        spi_rx.w_tau_now[1] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_T_DIV);
        spi_rx.w_tau_now[2] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_T_DIV);
        spi_rx.w_tau_now[3] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_T_DIV);

#if 0
    printf("w_connect:%d%d%d%d\n",spi_rx.w_connect[0],spi_rx.w_connect[1],spi_rx.w_connect[2],spi_rx.w_connect[3]);
    printf("wd: %.3f %.3f %.3f %.3f\n",spi_rx.w_dq_now[0],spi_rx.w_dq_now[1],spi_rx.w_dq_now[2],spi_rx.w_dq_now[3]);
    printf("tau:%.3f %.3f %.3f %.3f\n",spi_rx.w_tau_now[0],spi_rx.w_tau_now[1],spi_rx.w_tau_now[2],spi_rx.w_tau_now[3]);
#endif
        temp= charFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.w_connect[0] = (temp-spi_rx.connect[i] *100)/10;
        spi_rx.w_connect[1] = temp%10;
        temp= charFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.w_connect[2] = (temp-spi_rx.connect[i] *100)/10;
        spi_rx.w_connect[3] = temp%10;

        //Sbus from STM32
        spi_rx.ocu.sbus_ch[0] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);
        spi_rx.ocu.sbus_ch[1] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);
        spi_rx.ocu.sbus_ch[2] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);
        spi_rx.ocu.sbus_ch[3] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);
        spi_rx.ocu.sbus_ch[4] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);
        spi_rx.ocu.sbus_ch[5] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);

        temp= charFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.ocu.sbus_conncect = temp/100;
        spi_rx.ocu.sbus_aux[0] = (temp-int(temp/100) *100)/10;
        spi_rx.ocu.sbus_aux[1] = temp%10;

        temp= charFromData_spi(spi_rx_buf, &anal_cnt);
       // spi_rx.ocu.sbus_aux[0] = temp/100;
        spi_rx.ocu.sbus_aux[2] = (temp-int(temp/100) *100)/10;
        spi_rx.ocu.sbus_aux[3] = temp%10;

        temp= charFromData_spi(spi_rx_buf, &anal_cnt);
       // spi_rx.ocu.sbus_aux[0] = temp/100;
        spi_rx.ocu.sbus_aux[4] = (temp-int(temp/100) *100)/10;
        spi_rx.ocu.sbus_aux[5] = temp%10;

        spi_rx.cap_touch = floatFromData_spi(spi_rx_buf, &anal_cnt);
        //printf("spi_rx.cap_touch=%f\n",spi_rx.cap_touch);
        spi_rx.bat_v[0] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);
        spi_rx.bat_v[0] =spi_rx.bat_v[1] =spi_rx.bat_v[2];
    }
    return 1;
}

void transfer(int fd, int sel)//发送
{
    static uint8_t state, rx_cnt;
    static uint8_t _data_len2 = 0, _data_cnt2 = 0;
    int ret;
    uint8_t data = 0;

    can_board_send(sel);
    ret=SPIDataRW(0,spi_tx_buf,rx,SPI_SEND_MAX); //向总线中写入7个数据

    if (ret < 1){
       printf("SPI Reopen!\n");
       SPISetup(0,speed);//printf("can't send spi message\n");
    }
    else
    {
        //printf("ret=%d\n",ret);
        for (int i = 0; i < SPI_SEND_MAX; i++)
        {
            data = rx[i];
            if (state == 0 && data == 0xFF)
            {
                state = 1;
                spi_rx_buf[0] = data;
            }
            else if (state == 1 && data == 0xFB)
            {
                state = 2;
                spi_rx_buf[1] = data;
            }
            else if (state == 2 && data > 0 && data < 0XF1)
            {
                state = 3;
                spi_rx_buf[2] = data;
            }
            else if (state == 3 && data < SPI_BUF_SIZE)
            {
                state = 4;
                spi_rx_buf[3] = data;
                _data_len2 = data;
                _data_cnt2 = 0;
            }
            else if (state == 4 && _data_len2 > 0)
            {
                _data_len2--;
                spi_rx_buf[4 + _data_cnt2++] = data;
                if (_data_len2 == 0)
                    state = 5;
            }
            else if (state == 5)
            {
                state = 0;
                spi_rx_buf[4 + _data_cnt2] = data;
                spi_rx_cnt = 4;
                slave_rx(spi_rx_buf, _data_cnt2 + 5);
            }
            else
                state = 0;
            //printf("%02x ",rx[i]);
        }
        //printf("\n");
    }
    //printf("after2\n");
}

//===========================================================Share Memeory form here========================================================================================

unsigned char mem_write_buf[MEM_SIZE];
unsigned char mem_read_buf[MEM_SIZE];
int mem_write_cnt=0;
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

void memory_write(void)//写入内存 to control_task feedback
{
    static float temp=0;
    mem_write_cnt=0;
    setDataFloat_mem( spi_rx.att[0]);
    setDataFloat_mem( spi_rx.att[1]);
    setDataFloat_mem( spi_rx.att[2]);
    setDataFloat_mem( spi_rx.att_rate[0]);
    setDataFloat_mem( spi_rx.att_rate[1]);
    setDataFloat_mem( spi_rx.att_rate[2]);
    setDataFloat_mem( spi_rx.acc_b[0]);
    setDataFloat_mem( spi_rx.acc_b[1]);
    setDataFloat_mem( spi_rx.acc_b[2]);
    setDataFloat_mem( spi_rx.acc_n[0]);
    setDataFloat_mem( spi_rx.acc_n[1]);
    setDataFloat_mem( spi_rx.acc_n[2]);
    //-----------MEMS USB
    setDataChar_mem (mems_usb_connect);
    setDataFloat_mem( To_180_degrees(spi_rx.att_usb[0]-spi_rx.att_usb_bias[0])*mems_usb_connect);
    setDataFloat_mem( To_180_degrees(spi_rx.att_usb[1]-spi_rx.att_usb_bias[1])*mems_usb_connect);
    setDataFloat_mem( To_180_degrees(spi_rx.att_usb[2]-spi_rx.att_usb_bias[2])*mems_usb_connect);
    setDataFloat_mem( (spi_rx.att_rate_usb[0]-spi_rx.att_rate_usb_bias[0])*mems_usb_connect);
    setDataFloat_mem( (spi_rx.att_rate_usb[1]-spi_rx.att_rate_usb_bias[1])*mems_usb_connect);
    setDataFloat_mem( (spi_rx.att_rate_usb[2]-spi_rx.att_rate_usb_bias[2])*mems_usb_connect);
    setDataFloat_mem( spi_rx.acc_b_usb[0]*mems_usb_connect);
    setDataFloat_mem( spi_rx.acc_b_usb[1]*mems_usb_connect);
    setDataFloat_mem( spi_rx.acc_b_usb[2]*mems_usb_connect);

    for (int i = 0; i < 4; i++)
    {
        setDataFloat_mem(spi_rx.q[i][0]);
        setDataFloat_mem(spi_rx.q[i][1]);
        setDataFloat_mem(spi_rx.q[i][2]);
        setDataFloat_mem(spi_rx.tau[i][0]);
        setDataFloat_mem(spi_rx.tau[i][1]);
        setDataFloat_mem(spi_rx.tau[i][2]);

        setDataFloat_mem(spi_rx.bat_v[i]);

        setDataChar_mem(spi_rx.connect[i]*100+spi_rx.connect_motor[i][0]*10+spi_rx.ready[i][0]);
        setDataChar_mem(spi_rx.connect[i]*100+spi_rx.connect_motor[i][1]*10+spi_rx.ready[i][1]);
        setDataChar_mem(spi_rx.connect[i]*100+spi_rx.connect_motor[i][2]*10+spi_rx.ready[i][2]);
    }
    //printf("%f %f %f\n",spi_rx.tau[0][0],spi_rx.tau[0][1],spi_rx.tau[0][2]);
    //SBUS
    setDataChar_mem( spi_rx.ocu.sbus_conncect *100+ spi_rx.ocu.sbus_aux[0]*10+spi_rx.ocu.sbus_aux[1]);
    setDataChar_mem( spi_rx.ocu.sbus_aux[2]*10+spi_rx.ocu.sbus_aux[3]);
    setDataChar_mem( spi_rx.ocu.sbus_aux[4]*10+spi_rx.ocu.sbus_aux[5]);

    setDataFloat_mem(spi_rx.ocu.sbus_ch[0]);
    setDataFloat_mem(spi_rx.ocu.sbus_ch[1]);
    setDataFloat_mem(spi_rx.ocu.sbus_ch[2]);
    setDataFloat_mem(spi_rx.ocu.sbus_ch[3]);
    setDataFloat_mem(spi_rx.ocu.sbus_ch[4]);
    setDataFloat_mem(spi_rx.ocu.sbus_ch[5]);

    //AOA UWB
    setDataFloat_mem(spi_rx.aoa.angle);
    setDataFloat_mem(spi_rx.aoa.dis);
    setDataFloat_mem(spi_rx.aoa.rssi);

    //wheel
    setDataFloat_mem(spi_rx.w_dq_now[0]);
    setDataFloat_mem(spi_rx.w_dq_now[1]);
    setDataFloat_mem(spi_rx.w_dq_now[2]);
    setDataFloat_mem(spi_rx.w_dq_now[3]);

    setDataFloat_mem(spi_rx.w_tau_now[0]);
    setDataFloat_mem(spi_rx.w_tau_now[1]);
    setDataFloat_mem(spi_rx.w_tau_now[2]);
    setDataFloat_mem(spi_rx.w_tau_now[3]);

    setDataFloat_mem (spi_rx.cap_touch );

#if 0
    printf("w_connect:%d%d%d%d\n", 1,1,1,1);
    printf("wd: %.3f %.3f %.3f %.3f\n",spi_rx.w_dq_now[0],spi_rx.w_dq_now[1],spi_rx.w_dq_now[2],spi_rx.w_dq_now[3]);
    printf("tau:%.3f %.3f %.3f %.3f\n",spi_rx.w_tau_now[0],spi_rx.w_tau_now[1],spi_rx.w_tau_now[2],spi_rx.w_tau_now[3]);
#endif
    setDataChar_mem(spi_rx.w_connect[0]*10+spi_rx.w_connect[1]);
    setDataChar_mem(spi_rx.w_connect[2]*10+spi_rx.w_connect[3]);
}

void memory_read(void){//读取内存 from control_task--action
int mem_read_cnt=MEM_SIZE/2;
float test1,test2;
for (int i = 0; i < 4; i++)
{
    spi_tx.q_set[i][0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);//dp for z-height
    spi_tx.q_set[i][1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.q_set[i][2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.q_reset[i][0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.q_reset[i][1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.q_reset[i][2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.tau_ff[i][0] = floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_tx.tau_ff[i][1] = floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_tx.tau_ff[i][2] = floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;

    spi_tx.param_sel[i]=mem_read_buf[mem_read_cnt++];//id isolate
}
    spi_tx.t_to_i= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.max_i= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    spi_tx.kp= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    spi_tx.kp_sw= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki_sw= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd_sw= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    spi_tx.kp_st= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki_st= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd_st= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    //bldc 0---PVT param
    spi_tx.kp_sw_d[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki_sw_d[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd_sw_d[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    spi_tx.kp_st_d[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki_st_d[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd_st_d[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    //bldc 1
    spi_tx.kp_sw_d[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki_sw_d[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd_sw_d[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    spi_tx.kp_st_d[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki_st_d[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd_st_d[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    //bldc 2
    spi_tx.kp_sw_d[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki_sw_d[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd_sw_d[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    spi_tx.kp_st_d[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki_st_d[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd_st_d[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    spi_tx.en_motor= mem_read_buf[mem_read_cnt++]*mem_connect;
    spi_tx.reser_q= mem_read_buf[mem_read_cnt++];
    spi_tx.reset_err= mem_read_buf[mem_read_cnt++];
   // printf("spi_tx.en_motor=%d, %f %f %f\n",spi_tx.en_motor+10*mem_connect,spi_tx.q_set[0][0],spi_tx.q_set[0][1],spi_tx.q_set[0][2]);
    spi_tx.led_enable[0]= mem_read_buf[mem_read_cnt]/10;//front led
    spi_tx.led_enable[1]= mem_read_buf[mem_read_cnt++]%10;

    spi_tx.led_side_enable[0]= mem_read_buf[mem_read_cnt++];//side led
    spi_tx.led_side_enable[1]= mem_read_buf[mem_read_cnt++];

    //-----------------------OCU param-------------------
    mems.imu_pos.x= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.imu_pos.y= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.imu_pos.z= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.imu_att.x= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.imu_att.y= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.imu_att.z= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.gps_pos.x= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.gps_pos.y= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.gps_pos.z= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.Acc_CALIBRATE=charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    mems.Gyro_CALIBRATE=charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    mems.Mag_CALIBRATE=charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_tx.beep_state=charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;

    //---arm & head exp_q
    spi_tx.arm_head3.att_set[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.arm_head3.att_set[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.arm_head3.att_set[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.arm_head3.head_set[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.arm_head3.head_set[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.arm_head3.head_set[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.arm_head3.power= charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_tx.arm_head3.cap= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    //---wheel control
    spi_tx.w_dq_set[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_tx.w_dq_set[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_tx.w_dq_set[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_tx.w_dq_set[3]= floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;

    spi_tx.w_tau_set[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_tx.w_tau_set[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_tx.w_tau_set[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_tx.w_tau_set[3]= floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;

    brain_connect_c= charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
#if 0
    printf("w_dq_exp:%.3f %.3f %.3f %.3f\n",spi_tx.w_dq_set[0],spi_tx.w_dq_set[1],spi_tx.w_dq_set[2],spi_tx.w_dq_set[3]);
#endif
}

pthread_mutex_t lock;
void* Thread_Mem(void*)//内存管理线程
{
    static int cnt = 0;
    static int mem_init_cnt=0;
    float sys_dt = 0;
    int i=0,memory_update=0;
    int flag = 0;
    int link_cnt=0;
    //共享内存
    int shmid_rx = shmget((key_t)MEM_SPI, sizeof(shareMemory_spi), 0666|IPC_CREAT); //失败返回-1，假设成功。
    //0666表示权限，与文件一样。如0644,它表示允许一个进程创建的共享内存被内存创建者所拥有的进程向共享内存读取和写入数据，同时其他用户创建的进程只能读取共享内存。
    void *shm_rx = shmat(shmid_rx, (void*)0, 0);  //失败返回-1，假设成功
    shareMemory *pshm_rx = (shareMemory*)shm_rx;
    pshm_rx->flag = 0;
    printf("Hardware::Memory Hardware attached at %p\n",shm_rx);

    while (1)
    {
        //共享内存写
        sys_dt = Get_Cycle_T(17);
        if(pshm_rx->flag == 0)
        {
            if(!mem_connect){
                mem_init_cnt++;
                if(mem_init_cnt>3){
                printf("Hardware::Memery Control Link=%d!!!\n",link_cnt++);
                mem_connect=1;
                }
            }
            mem_loss_cnt=0;
            //pthread_mutex_lock(&lock);
            memory_write();
            for(int k=0;k<MEM_SIZE/2-1;k++)
                pshm_rx->szMsg[k]=mem_write_buf[k];
            for(int k=MEM_SIZE/2;k<MEM_SIZE;k++)
                mem_read_buf[k]=pshm_rx->szMsg[k];
            memory_read();
            //pthread_mutex_unlock(&lock);
            pshm_rx->flag = 1;
        }else{
            mem_loss_cnt+=sys_dt;
            mem_init_cnt=0;
        }
        //mem_loss_cnt+=sys_dt;
        //printf("mem_loss_cnt=%f sys_dt=%f\n",mem_loss_cnt,sys_dt);
        if(mem_loss_cnt>1&&mem_connect==1){
            mem_connect=0;
            mem_loss_cnt=0;
            mem_init_cnt=0;
            for (int i = 0; i < 4; i++){
            spi_tx.q_set[i][0] = spi_rx.q[i][0];
            spi_tx.q_set[i][1] = spi_rx.q[i][1];
            spi_tx.q_set[i][2] = spi_rx.q[i][2];
            spi_tx.tau_ff[i][0] = 0;
            spi_tx.tau_ff[i][1] = 0;
            spi_tx.tau_ff[i][2] = 0;
            }
            spi_tx.kp= spi_tx.ki= spi_tx.kd= spi_tx.en_motor= 0;
            spi_tx.kp_sw= spi_tx.ki_sw= spi_tx.kd_sw= spi_tx.en_motor= 0;
            spi_tx.kp_st= spi_tx.ki_st= spi_tx.kd_st= spi_tx.en_motor= 0;
            printf("Hardware::Memery Control Loss!!!\n");
        }

        usleep(500);
    }
    shmdt(shm_rx);  //失败返回-1，假设成功
    shmctl(shmid_rx, IPC_RMID, 0);  //
    return 0;
}

void* Thread_SPI(void*)//内存管理线程
{
    static float timer_spi1 = 0,timer_spi2=0;
    static int cnt = 0;
    static int timer_1s=0;
    static int timer_1m=0;
    static int timer_1h=0;
    static float timer_cnt=0;
    float sys_dt = 0;
    int i=0,memory_update=0;
    int flag = 0;
    int fd = 0;
    char buf_mem[MEM_SIZE]={1,2};
    Cycle_Time_Init();
    fd=SPISetup(0,speed); //初始化SPI通道0，并设置为最大速度500000hz
    if(fd==-1)
        printf("init spi failed!\n");

    while (1)
    {
        sys_dt = Get_Cycle_T(15);
        timer_cnt+=sys_dt;
        if(timer_cnt>1){
            timer_cnt=0;
            timer_1s++;
            if(timer_1s>60){timer_1s=0;
                timer_1m++;
            }
            if(timer_1m>60){timer_1m=0;
                timer_1h++;}

            printf("Hardware::SPI Still Online at hour-%d min-%d sec-%d spi_cnt=%d\n",timer_1h,timer_1m,timer_1s,spi_tx_cnt_show);
        }

        spi_loss_cnt += sys_dt;

        if (spi_loss_cnt > 1.5&& spi_connect==1)
        {
            spi_loss_cnt = 0;
            spi_connect = 0;
            printf("Hardware::Hardware SPI-STM32 Loss!!!\n");
        }
        //-------SPI CAN发送
        timer_spi1+= sys_dt;
        timer_spi2+= sys_dt;

        transfer(fd, 44);//3 bldc div

        usleep(DELAY_SPI);
    }
    close(fd);
    return 0;
}


#if NO_THREAD&&!EN_MULTI_THREAD
int Thread_ALL(void)
#else
void* Thread_ALL(void*)
#endif
{
    static float timer_spi1 = 0,timer_spi2=0;
    static int mem_init_cnt=0;
    float sys_dt = 0;
    int i=0,memory_update=0;
    int flag = 0;
    int link_cnt=0;
    int fd = 0;
    char buf_mem[MEM_SIZE]={1,2};
    Cycle_Time_Init();

    fd=SPISetup(0,speed); //初始化SPI通道0，并设置为最大速度500000hz
    if(fd==-1)
        printf("init spi failed!\n");
    //while(1);
    //共享内存
    int shmid_rx = shmget((key_t)MEM_SPI, sizeof(shareMemory_spi), 0666|IPC_CREAT); //失败返回-1，假设成功。
    //0666表示权限，与文件一样。如0644,它表示允许一个进程创建的共享内存被内存创建者所拥有的进程向共享内存读取和写入数据，同时其他用户创建的进程只能读取共享内存。
    void *shm_rx = shmat(shmid_rx, (void*)0, 0);  //失败返回-1，假设成功
    shareMemory *pshm_rx = (shareMemory*)shm_rx;
    pshm_rx->flag = 0;
    printf("Hardware Butler::Memory Hardware attached at %p\n",shm_rx);
    while (1)
    {
        sys_dt = Get_Cycle_T(15);
        spi_loss_cnt += sys_dt;
        if (spi_loss_cnt > 1.5&& spi_connect==1)
        {
            spi_loss_cnt = 0;
            spi_connect = 0;
            printf("Hardware::Hardware SPI-STM32 Loss!!!\n");
        }
        //-------SPI CAN发送
        timer_spi1+= sys_dt;
        timer_spi2+= sys_dt;

        transfer(fd, 44);

        //共享内存写
        if(pshm_rx->flag == 0)
        {
            if(!mem_connect){
                mem_init_cnt++;
                if(mem_init_cnt>3){
                printf("Hardware::Memery Control Link=%d!!!\n",link_cnt++);
                mem_connect=1;
                }
            }
            mem_loss_cnt=0;
            memory_write();
            for(int k=0;k<MEM_SIZE/2-1;k++)
                pshm_rx->szMsg[k]=mem_write_buf[k];
            for(int k=MEM_SIZE/2;k<MEM_SIZE;k++)
                mem_read_buf[k]=pshm_rx->szMsg[k];
            memory_read();
            pshm_rx->flag = 1;
        }else
            mem_loss_cnt+=sys_dt;
        mem_loss_cnt+=sys_dt;
        printf("mem_loss_cnt=%f\n",mem_loss_cnt);
        if(mem_loss_cnt>1&&mem_connect==1){
            mem_connect=0;
            mem_loss_cnt=0;
            mem_init_cnt=0;
            for (int i = 0; i < 4; i++){
            spi_tx.q_set[i][0] = spi_rx.q[i][0];
            spi_tx.q_set[i][1] = spi_rx.q[i][1];
            spi_tx.q_set[i][2] = spi_rx.q[i][2];
            spi_tx.tau_ff[i][0] = 0;
            spi_tx.tau_ff[i][1] = 0;
            spi_tx.tau_ff[i][2] = 0;
            }
            spi_tx.kp= spi_tx.ki= spi_tx.kd= spi_tx.en_motor= 0;
            spi_tx.kp_st= spi_tx.ki_st= spi_tx.kd_st= spi_tx.en_motor= 0;
            spi_tx.kp_sw= spi_tx.ki_sw= spi_tx.kd_sw= spi_tx.en_motor= 0;
            printf("Hardware::Memery Control Loss!!!\n");
        }
        usleep(200);
    }
    close(fd);
    shmdt(shm_rx);  //失败返回-1，假设成功
    shmctl(shmid_rx, IPC_RMID, 0);  //
    return 0;
}

int main(int argc, char *argv[])
{
    int use_usb_imu=config_hardware["mems_param"]["imu_usb_enable"].as<float>();//
#if NO_THREAD&&!EN_MULTI_THREAD&&!USE_USB
    Thread_ALL();
#else
    pthread_t tida, tidb,tidc;
    pthread_mutex_init(&lock, NULL);
    #if EN_MULTI_THREAD&&!USE_USB
    pthread_create(&tida, NULL, Thread_Mem, NULL);
    pthread_create(&tida, NULL, Thread_SPI, NULL);
    pthread_join(tida, NULL);
    pthread_join(tidb, NULL);
    if(use_usb_imu)
        pthread_join(tidc, NULL);
    #else
#if !USE_USB
    pthread_create(&tida, NULL, Thread_ALL, NULL);
    pthread_join(tida, NULL);
#else
    pthread_create(&tidb, NULL, Thread_Mem, NULL);
    pthread_create(&tidc, NULL, Thread_USB, NULL);
#endif
#if EN_MULTI_THREAD
    pthread_join(tidb, NULL);
    pthread_join(tidc, NULL);
#endif
    #endif
#endif
    return 1;
}
