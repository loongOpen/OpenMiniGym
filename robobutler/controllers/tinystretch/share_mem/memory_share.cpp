#include <sys/shm.h>
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
#include "include.h"
#include "gait_math.h"
#include "locomotion_header.h"

// ps -ef | grep control_task   ps a全部查看
// ps -ef | grep hardware_task
// kill －9 324  ps aux | grep "MOCO_ML"
// sudo nano /etc/rc.local

_SPI_RX spi_rx;
_SPI_TX spi_tx;

_NAV_RX nav_rx;
_NAV_TX nav_tx;
_MEMS mems;

int mem_connect=0;
float mem_loss_cnt=0;
int mem_connect_c=0;
float mem_loss_cnt_c=0;
struct shareMemory
{
    int  flag=0;  //作为一个标志，非0：表示可读，0表示可写
    unsigned char szMsg[MEM_SIZE];
};
struct shareMemory shareMemory_spi,shareMemory_control;

unsigned char mem_read_buf[MEM_SIZE];
unsigned char mem_write_buf[MEM_SIZE];
int mem_read_cnt=0;
int mem_write_cnt=0;

unsigned char mem_read_buf_c[MEM_SIZE];
unsigned char mem_write_buf_c[MEM_SIZE];
int mem_read_cnt_c=0;
int mem_write_cnt_c=0;
int brain_connect_c=0;
void sbus_mode_anal(){
    int i;
    static int cnt_zero_cal=0;
    static int cnt_mems_cal=0;
    static int cnt_temp=0;

#if SBUS_YUNZHUO
    int zero_cal_sel=1;
    int ctrl_mode_sel=0;
    int motor_init_sel=4;//height sw
    int auto_forward_sel=3;
    int sbus_gait_sel=5;
    int smart_mode_sel=2;

    if(cnt_temp++>100){
        cnt_temp=0;
        //        //yunzhuo
        //        int key_a_yun,key_b_yun,key_c_yun,key_d_yun;
        //        int sel_e_yun,sel_f_yun;
        //        float rc_g_yun,rc_h_yun;
        ocu.key_a_yun=spi_rx.ocu.sbus_aux[2];
        ocu.key_b_yun=spi_rx.ocu.sbus_aux[3];
        ocu.key_c_yun=spi_rx.ocu.sbus_aux[4];
        ocu.key_d_yun=spi_rx.ocu.sbus_aux[5];
        ocu.sel_e_yun=spi_rx.ocu.sbus_aux[0];
        ocu.sel_f_yun=spi_rx.ocu.sbus_aux[1];
        ocu.rc_g_yun=spi_rx.ocu.sbus_ch[4];
        ocu.rc_h_yun=spi_rx.ocu.sbus_ch[5];

        if(ocu.sbus_aux_reg[0]!=ocu.sel_f_yun)
            cnt_zero_cal+=10;
        else
            cnt_zero_cal-=5;
        cnt_zero_cal=LIMIT(cnt_zero_cal,0,100);
        if(cnt_zero_cal>30&&vmc_all.gait_mode==IDLE)
        {
            ocu.sbus_cal_motor_zero=1;
        }else
            ocu.sbus_cal_motor_zero=0;

        //mems cal
        if(ocu.sbus_aux_reg[1]!=ocu.sel_e_yun&&vmc_all.gait_mode==IDLE)
            cnt_mems_cal+=10;
        else
            cnt_mems_cal-=5;
        cnt_mems_cal=LIMIT(cnt_mems_cal,0,100);
        if(cnt_mems_cal>30)
        {
            ocu.sbus_cal_mems=1;
            robotwb.beep_state=BEEP_BLDC_GAIT_SWITCH;
            printf("SBUS::Cal IMU Commond!\n");
        }else
            ocu.sbus_cal_mems=0;

        if(spi_rx.ocu.sbus_ch[0]<-0.3&&spi_rx.ocu.sbus_ch[3]<-0.3&&
           spi_rx.ocu.sbus_ch[1]<-0.3&&spi_rx.ocu.sbus_ch[2]>0.3)
            ocu.sbus_power_off=1;
        else
            ocu.sbus_power_off=0;

        if(spi_rx.ocu.sbus_ch[0]<-0.47&&spi_rx.ocu.sbus_ch[1]<-0.47&&
           spi_rx.ocu.sbus_ch[2]<-0.45&&spi_rx.ocu.sbus_ch[3]>0.47)
            ocu.sbus_power_switch=1;
        else
            ocu.sbus_power_switch=0;

        if(ocu.sbus_aux_reg[2]!=ocu.key_c_yun)
            ocu.sbus_motor_init=1;
        else
            ocu.sbus_motor_init=0;

        ocu.sbus_height_switch=ocu.key_c_yun;//swing height
        ocu.sbus_control_mode=ocu.sel_e_yun;
        ocu.sbus_smart_en=ocu.key_b_yun;
        ocu.sbus_gait_sel=ocu.sel_f_yun;

        ocu.sbus_spd_rate[Xr]=LIMIT(ocu.rc_g_yun+0.5,0,1);
        ocu.sbus_spd_rate[Zr]=ocu.sbus_ch[0];//spd


        ocu.sbus_aux_reg[0]=ocu.sel_f_yun;
        ocu.sbus_aux_reg[1]=ocu.sel_e_yun;
        ocu.sbus_aux_reg[2]=ocu.key_c_yun;
        #if 0

        printf("connect=%d a0=%d a1=%d a2=%d a3=%d a4=%d a5=%d\n",
          spi_rx.ocu.sbus_conncect ,  spi_rx.ocu.sbus_aux[0],spi_rx.ocu.sbus_aux[1]
                , spi_rx.ocu.sbus_aux[2],spi_rx.ocu.sbus_aux[3]
                , spi_rx.ocu.sbus_aux[4],spi_rx.ocu.sbus_aux[5]);
        printf("rc0=%f rc1=%f rc2=%f rc3=%f rc4=%f rc5=%f\n",
          spi_rx.ocu.sbus_ch[0],spi_rx.ocu.sbus_ch[1]
                ,spi_rx.ocu.sbus_ch[2],spi_rx.ocu.sbus_ch[3]
                ,spi_rx.ocu.sbus_ch[4],spi_rx.ocu.sbus_ch[5]);
        #endif
    }
#else
    int zero_cal_sel=3;
    int motor_init_sel=0;
    int ctrl_mode_sel=1;
    int sbus_gait_sel=2;
    int smart_mode_sel=3;

    if(cnt_temp++>100){
        cnt_temp=0;
        if(ocu.sbus_aux_reg[zero_cal_sel]!=ocu.sbus_aux[zero_cal_sel])
            cnt_zero_cal+=10;
        else
            cnt_zero_cal-=5;
        cnt_zero_cal=LIMIT(cnt_zero_cal,0,100);
        if(cnt_zero_cal>40)
        {
            ocu.sbus_cal_motor_zero=1;
        }else
            ocu.sbus_cal_motor_zero=0;

        if(spi_rx.ocu.sbus_ch[0]<-0.47&&spi_rx.ocu.sbus_ch[3]<-0.47&&
           spi_rx.ocu.sbus_ch[1]<-0.47&&spi_rx.ocu.sbus_ch[2]>0.47)
            ocu.sbus_power_off=1;
        else
            ocu.sbus_power_off=0;

        if(spi_rx.ocu.sbus_ch[0]<-0.47&&spi_rx.ocu.sbus_ch[1]<-0.47&&
           spi_rx.ocu.sbus_ch[2]<-0.45&&spi_rx.ocu.sbus_ch[3]>0.47)
            ocu.sbus_power_switch=1;
        else
            ocu.sbus_power_switch=0;

        if(ocu.sbus_aux_reg[motor_init_sel]!=ocu.sbus_aux[motor_init_sel])
            ocu.sbus_motor_init=1;
        else
            ocu.sbus_motor_init=0;

        ocu.sbus_height_switch=ocu.sbus_aux[motor_init_sel];
        ocu.sbus_control_mode=ocu.sbus_aux[ctrl_mode_sel];
        ocu.sbus_smart_en=ocu.sbus_aux[smart_mode_sel];
        ocu.sbus_gait_sel=ocu.sbus_aux[sbus_gait_sel];
        ocu.sbus_height_rate=LIMIT(ocu.sbus_ch[0]+0.5,0,1);

        for(i=0;i<4;i++)
          ocu.sbus_aux_reg[i]= ocu.sbus_aux[i];
        #if 0
//        printf("connect=%d a0=%d a1=%d a2=%d a3=%d\n",
//          spi_rx.ocu.sbus_conncect ,  spi_rx.ocu.sbus_aux[0],spi_rx.ocu.sbus_aux[1], spi_rx.ocu.sbus_aux[2],spi_rx.ocu.sbus_aux[3] );
//        printf("rc0=%f rc1=%f rc2=%f rc3=%f\n",
//          spi_rx.ocu.sbus_ch[0],spi_rx.ocu.sbus_ch[1],spi_rx.ocu.sbus_ch[2],spi_rx.ocu.sbus_ch[3]);

        printf("connect=%d a0=%d a1=%d a2=%d a3=%d a4=%d a5=%d\n",
          spi_rx.ocu.sbus_conncect ,  spi_rx.ocu.sbus_aux[0],spi_rx.ocu.sbus_aux[1]
                , spi_rx.ocu.sbus_aux[2],spi_rx.ocu.sbus_aux[3]
                , spi_rx.ocu.sbus_aux[4],spi_rx.ocu.sbus_aux[5]);
        printf("rc0=%f rc1=%f rc2=%f rc3=%f rc4=%f rc5=%f\n",
          spi_rx.ocu.sbus_ch[0],spi_rx.ocu.sbus_ch[1]
                ,spi_rx.ocu.sbus_ch[2],spi_rx.ocu.sbus_ch[3]
                ,spi_rx.ocu.sbus_ch[4],spi_rx.ocu.sbus_ch[5]);
        #endif
    }
    #endif
}


static void setDataFloat_mem(float f, int *anal_cnt)
{
    int i = *(int *)&f;
    mem_write_buf[*anal_cnt+0] = ((i << 24) >> 24);
    mem_write_buf[*anal_cnt+1] = ((i << 16) >> 24);
    mem_write_buf[*anal_cnt+2] = ((i << 8) >> 24);
    mem_write_buf[*anal_cnt+3] = (i >> 24);

    *anal_cnt += 4;
}

static void setDataFloat_mem_c(float f, int *anal_cnt)
{
    int i = *(int *)&f;
    mem_write_buf_c[*anal_cnt+0] = ((i << 24) >> 24);
    mem_write_buf_c[*anal_cnt+1] = ((i << 16) >> 24);
    mem_write_buf_c[*anal_cnt+2] = ((i << 8) >> 24);
    mem_write_buf_c[*anal_cnt+3] = (i >> 24);

    *anal_cnt += 4;
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

static char charFromData_spi(unsigned char *data, int *anal_cnt)
{
    int temp = *anal_cnt;
    *anal_cnt += 1;
    return *(data + temp);
}


int id_swap[4]={1,3,0,2};
void memory_write_c(int st_mem)//写入内存 NAV  写入里程计 状态估计 OCU状态
{
int i;
static float temp=0;
mem_write_cnt_c=st_mem;
//------------------------数据转化------------------------
for(i=0;i<4;i++){//记录无刷控制数据 上传给Record 上位机 jilu
    nav_rx.is_ground[i]=vmc[id_swap[i]].ground;
    nav_rx.is_touch[i]=vmc[id_swap[i]].is_touch;
    nav_rx.force_en_flag[i]=stand_force_enable_flag[id_swap[i]];
    nav_rx.leg_state[i]=vmc[id_swap[i]].param.trig_state;
    nav_rx.leg_connect[i]=leg_motor[id_swap[i]].connect;
    nav_rx.motor_connect[i][0]=leg_motor[id_swap[i]].connect_motor[0];
    nav_rx.motor_connect[i][1]=leg_motor[id_swap[i]].connect_motor[1];
    nav_rx.motor_connect[i][2]=leg_motor[id_swap[i]].connect_motor[2];
    nav_rx.motor_ready[i][0]=leg_motor[id_swap[i]].ready[0];
    nav_rx.motor_ready[i][1]=leg_motor[id_swap[i]].ready[1];
    nav_rx.motor_ready[i][2]=leg_motor[id_swap[i]].ready[2];
    nav_rx.epos_n_tar[i].x=vmc[id_swap[i]].param.tar_epos_h.x;//末端位置 包含阻抗
    nav_rx.epos_n_tar[i].y=vmc[id_swap[i]].param.tar_epos_h.y;//末端位置 包含阻抗
    nav_rx.epos_n_tar[i].z=vmc[id_swap[i]].param.tar_epos_h.z;//末端位置 包含阻抗
    nav_rx.epos_n_now[i].x=vmc[id_swap[i]].epos.x;
    nav_rx.epos_n_now[i].y=vmc[id_swap[i]].epos.y;
    nav_rx.epos_n_now[i].z=vmc[id_swap[i]].epos.z;

    nav_rx.depos_n_tar[i].x=robotwb.Leg[id_swap[i]].force_imp_spd_h.x;
    nav_rx.depos_n_tar[i].y=robotwb.Leg[id_swap[i]].force_imp_spd_h.y;
    nav_rx.depos_n_tar[i].z=robotwb.Leg[id_swap[i]].force_imp_spd_h.z;
    nav_rx.depos_n_now[i].x=vmc[id_swap[i]].epos_spdd_n.x;
    nav_rx.depos_n_now[i].y=vmc[id_swap[i]].epos_spdd_n.y;
    nav_rx.depos_n_now[i].z=vmc[id_swap[i]].epos_spdd_n.z;
    if(i==0||i==1){
        nav_rx.sita_now[i][0]=robotwb.arm_base_height*10;
        nav_rx.sita_now[i][1]=robotwb.arm_q[0][0]*57.3;//spi_rx.w_dq_now[id_swap[i]];//vmc[id_swap[i]].sita2;
        nav_rx.sita_now[i][2]=robotwb.arm_q[0][1]*57.3;//vmc[id_swap[i]].sita3;
    }else{
        nav_rx.sita_now[i][0]=robotwb.arm_q[0][2]*57.3;
        nav_rx.sita_now[i][1]=robotwb.arm_q[0][3]*57.3;//spi_rx.w_dq_now[id_swap[i]];//vmc[id_swap[i]].sita2;
        nav_rx.sita_now[i][2]=robotwb.arm_q[0][4]*57.3;//vmc[id_swap[i]].sita3;
    }
    nav_rx.sita_tar[i][0]=vmc[id_swap[i]].tar_sita1;
    nav_rx.sita_tar[i][1]=vmc[id_swap[i]].tar_sita2;
    nav_rx.sita_tar[i][2]=vmc[id_swap[i]].tar_sita3;

    nav_rx.GRF_n_tar[i].x=robotwb.Leg[id_swap[i]].tar_force_dis_n.x;
    nav_rx.GRF_n_tar[i].y=robotwb.Leg[id_swap[i]].tar_force_dis_n.y;
    nav_rx.GRF_n_tar[i].z=robotwb.Leg[id_swap[i]].tar_force_dis_n.z;
    nav_rx.GRF_n_now[i].x=robotwb.Leg[id_swap[i]].force_est_n_output.x;
    nav_rx.GRF_n_now[i].y=robotwb.Leg[id_swap[i]].force_est_n_output.y;
    nav_rx.GRF_n_now[i].z=robotwb.Leg[id_swap[i]].force_est_n_output.z;

    nav_rx.tau_tar[i][0]=robotwb.Leg[id_swap[i]].taod[0];
    nav_rx.tau_tar[i][1]=robotwb.Leg[id_swap[i]].taod[1];
    nav_rx.tau_tar[i][2]=robotwb.Leg[id_swap[i]].taod[2];
    nav_rx.tau_now[i][0]=robotwb.Leg[id_swap[i]].taom[0]-robotwb.Leg[id_swap[i]].taod_mess[0];
    nav_rx.tau_now[i][1]=robotwb.Leg[id_swap[i]].taom[1]-robotwb.Leg[id_swap[i]].taod_mess[1];
    nav_rx.tau_now[i][2]=robotwb.Leg[id_swap[i]].taom[2]-robotwb.Leg[id_swap[i]].taod_mess[2];
    }
if(vmc_all.gait_mode==0||1){
    nav_rx.att_now[0]=robotwb.now_att.pitch;
    nav_rx.att_now[1]=robotwb.now_att.roll;
    nav_rx.att_now[2]=robotwb.now_att.yaw;
}else{
    nav_rx.att_now[0]=robotwb.Leg[0].tar_force_dis_n.z;
    nav_rx.att_now[1]=0;
    nav_rx.att_now[2]=robotwb.Leg[0].force_est_n_output.z;

}
    nav_rx.datt_now[0]=robotwb.now_rate.pitch;
    nav_rx.datt_now[1]=robotwb.now_rate.roll;
    nav_rx.datt_now[2]=robotwb.now_rate.yaw;

    nav_rx.acc_n_now[0]=robotwb.cog_acc_n.x;
    nav_rx.acc_n_now[1]=robotwb.cog_acc_n.y;
    nav_rx.acc_n_now[2]=robotwb.cog_acc_n.z;

    nav_rx.att_tar[0]=robotwb.exp_att.pitch;
    nav_rx.att_tar[1]=robotwb.exp_att.roll;
    nav_rx.att_tar[2]=robotwb.exp_att.yaw;

    nav_rx.com_n_tar[0]=robotwb.exp_pos_n.x;
    nav_rx.com_n_tar[1]=robotwb.exp_pos_n.y;
    nav_rx.com_n_tar[2]=robotwb.exp_pos_n.z;

    nav_rx.com_n_now[0]=vmc_all.pos_n.x;
    nav_rx.com_n_now[1]=vmc_all.pos_n.y;
    nav_rx.com_n_now[2]=vmc_all.pos_n.z;

    nav_rx.dcom_n_tar[0]=robotwb.exp_spd_n.x;
    nav_rx.dcom_n_tar[1]=robotwb.exp_spd_n.y;
    nav_rx.dcom_n_tar[2]=robotwb.exp_spd_n.z;
 if(vmc_all.gait_mode==TROT){
    nav_rx.dcom_n_now[0]=vmc_all.spd_n.x;
    nav_rx.dcom_n_now[1]=vmc_all.spd_n.y;
    nav_rx.dcom_n_now[2]=vmc_all.spd_n.z;
 }else{
     nav_rx.dcom_n_now[0]=0;
     nav_rx.dcom_n_now[1]=0;
     nav_rx.dcom_n_now[2]=0;
  }
    nav_rx.ground_att_now[0]=vmc_all.ground_att_est[0];
    nav_rx.ground_att_now[1]=vmc_all.ground_att_est[1];
    nav_rx.ground_att_now[2]=vmc_all.ground_att_est[2];

    nav_rx.gait_state=vmc_all.param.robot_mode;
    nav_rx.rc_mode=sdk.sdk_mode;
    //--------------------自定义记录
     nav_rx.temp_record[0]=vmc[0].tar_sita1;
     nav_rx.temp_record[1]=vmc[0].tar_sita2;

     nav_rx.temp_record[6]= robotwb.Leg[0].espd_norm_b;

//-------------------------写入内存（主要是计数用）-------------------------
for (i=0;i<4;i++)
{
    mem_write_buf_c[mem_write_cnt_c++] = nav_rx.is_touch[i];
    mem_write_buf_c[mem_write_cnt_c++] = nav_rx.is_ground[i];
    mem_write_buf_c[mem_write_cnt_c++] = nav_rx.force_en_flag[i];
    mem_write_buf_c[mem_write_cnt_c++] = nav_rx.leg_state[i];
    mem_write_buf_c[mem_write_cnt_c++] = nav_rx.leg_connect[i]*100+nav_rx.motor_connect[i][0]*10+nav_rx.motor_ready[i][0];
    mem_write_buf_c[mem_write_cnt_c++] = nav_rx.leg_connect[i]*100+nav_rx.motor_connect[i][1]*10+nav_rx.motor_ready[i][1];
    mem_write_buf_c[mem_write_cnt_c++] = nav_rx.leg_connect[i]*100+nav_rx.motor_connect[i][2]*10+nav_rx.motor_ready[i][2];
//    mem_write_buf_c[mem_write_cnt_c++] = spi_rx.w_connect[i]*100+spi_rx.w_connect[i]*10+spi_rx.w_connect[i];
//    mem_write_buf_c[mem_write_cnt_c++] = spi_rx.w_connect[i]*100+spi_rx.w_connect[i]*10+spi_rx.w_connect[i];
//    mem_write_buf_c[mem_write_cnt_c++] = spi_rx.w_connect[i]*100+spi_rx.w_connect[i]*10+spi_rx.w_connect[i];
    setDataFloat_mem_c( nav_rx.epos_n_tar[i].x,&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.epos_n_tar[i].y,&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.epos_n_tar[i].z,&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.epos_n_now[i].x,&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.epos_n_now[i].y,&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.epos_n_now[i].z,&mem_write_cnt_c);

    setDataFloat_mem_c( nav_rx.depos_n_tar[i].x,&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.depos_n_tar[i].y,&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.depos_n_tar[i].z,&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.depos_n_now[i].x,&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.depos_n_now[i].y,&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.depos_n_now[i].z,&mem_write_cnt_c);

    setDataFloat_mem_c( nav_rx.sita_tar[i][0],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.sita_tar[i][1],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.sita_tar[i][2],&mem_write_cnt_c);

    setDataFloat_mem_c( nav_rx.sita_now[i][0],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.sita_now[i][1],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.sita_now[i][2],&mem_write_cnt_c);

    setDataFloat_mem_c( nav_rx.tau_tar[i][0],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.tau_tar[i][1],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.tau_tar[i][2],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.tau_now[i][0],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.tau_now[i][1],&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.tau_now[i][2],&mem_write_cnt_c);

    setDataFloat_mem_c( nav_rx.GRF_n_tar[i].x,&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.GRF_n_tar[i].y,&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.GRF_n_tar[i].z,&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.GRF_n_now[i].x,&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.GRF_n_now[i].y,&mem_write_cnt_c);
    setDataFloat_mem_c( nav_rx.GRF_n_now[i].z,&mem_write_cnt_c);
}
setDataFloat_mem_c( nav_rx.com_n_tar[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.com_n_tar[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.com_n_tar[2],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.com_n_now[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.com_n_now[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.com_n_now[2],&mem_write_cnt_c);

nav_rx.acc_n_now[0]=robotwb.cog_acc_b.x;
nav_rx.acc_n_now[1]=robotwb.cog_acc_b.y;
nav_rx.acc_n_now[2]=robotwb.cog_acc_b.z;
setDataFloat_mem_c( nav_rx.acc_n_now[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.acc_n_now[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.acc_n_now[2],&mem_write_cnt_c);

setDataFloat_mem_c( nav_rx.dcom_n_tar[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.dcom_n_tar[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.dcom_n_tar[2],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.dcom_n_now[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.dcom_n_now[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.dcom_n_now[2],&mem_write_cnt_c);

setDataFloat_mem_c( nav_rx.ground_att_now[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.ground_att_now[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.ground_att_now[2],&mem_write_cnt_c);

setDataFloat_mem_c( nav_rx.att_now[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.att_now[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.att_now[2],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.datt_now[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.datt_now[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.datt_now[2],&mem_write_cnt_c);

setDataFloat_mem_c( nav_rx.att_tar[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.att_tar[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.att_tar[2],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.datt_tar[0],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.datt_tar[1],&mem_write_cnt_c);
setDataFloat_mem_c( nav_rx.datt_tar[2],&mem_write_cnt_c);

mem_write_buf_c[mem_write_cnt_c++] = nav_rx.gait_state;
mem_write_buf_c[mem_write_cnt_c++] = nav_rx.rc_mode;

for (i=0;i<10;i++)
    setDataFloat_mem_c( nav_rx.temp_record[i],&mem_write_cnt_c);
}


//-------------------------内存管理-OCU----------------------------
void memory_read_c(int mem_st){//读取内存 OCU  读取遥控  模式等等 from UDP thread
char temp;
char i=0;
mem_read_cnt_c=mem_st;

nav_tx.connect = charFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.ocu_mode = charFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

nav_tx.rc_spd_b[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;
nav_tx.rc_spd_b[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;
nav_tx.rc_spd_b[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;

nav_tx.rc_rate_b[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;
nav_tx.rc_rate_b[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;
nav_tx.rc_rate_b[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;

nav_tx.key_ud = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_lr = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_x = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_y = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_a = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_b = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_ll = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_rr = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_st = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
nav_tx.key_back = (charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)-1)*mem_connect_c;
//printf("connect=%d up=%d x=%d b=%d a=%d\n",nav_tx.connect ,
//    nav_tx.key_ud,nav_tx.key_x,nav_tx.key_b ,nav_tx.key_a  );

static int cnt_p;

//SDK-------外部直接力矩控制接口
nav_tx.request_gait = charFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

nav_tx.exp_spd_o[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.exp_spd_o[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.exp_spd_o[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

nav_tx.exp_att_o[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.exp_att_o[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.exp_att_o[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

nav_tx.exp_datt_o[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.exp_datt_o[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.exp_datt_o[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

nav_tx.exp_pos_o[0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.exp_pos_o[1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
nav_tx.exp_pos_o[2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

for(i=0;i<4;i++){
    nav_tx.exp_q_o[i][0] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_tx.exp_q_o[i][1] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_tx.exp_q_o[i][2] = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);

    nav_tx.exp_GRF_o[i].x = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_tx.exp_GRF_o[i].y = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
    nav_tx.exp_GRF_o[i].z = floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
}

//-----------------------OCU param-------------------
mems.imu_pos.x= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.imu_pos.y= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.imu_pos.z= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.imu_att.x= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.imu_att.y= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.imu_att.z= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.gps_pos.x= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.gps_pos.y= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.gps_pos.z= floatFromData_spi(mem_read_buf_c, &mem_read_cnt_c);
mems.Acc_CALIBRATE=charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;
mems.Gyro_CALIBRATE=charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;
mems.Mag_CALIBRATE=charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;
spi_rx.brain_connect_c=brain_connect_c=charFromData_spi(mem_read_buf_c, &mem_read_cnt_c)*mem_connect_c;

//-----------------------数据转化---------------------
ocu.connect=nav_tx.connect;
ocu.mode=nav_tx.ocu_mode;
ocu.rc_spd_w[Xr]=nav_tx.rc_spd_b[0];
ocu.rc_spd_w[Yr]=nav_tx.rc_spd_b[1];
ocu.rc_att_w[PITr]=nav_tx.rc_rate_b[0];
ocu.rc_att_w[ROLr]=nav_tx.rc_rate_b[1];
ocu.rate_yaw_w=nav_tx.rc_rate_b[2];

ocu.key_st=nav_tx.key_st;
ocu.key_back=nav_tx.key_back;
ocu.key_lr=nav_tx.key_lr;
ocu.key_ud=nav_tx.key_ud;
ocu.key_x=nav_tx.key_x;
ocu.key_y=nav_tx.key_y;
ocu.key_b=nav_tx.key_b;
ocu.key_a=nav_tx.key_a;
ocu.key_ll=nav_tx.key_ll;
ocu.key_rr=nav_tx.key_rr;
#if 0
if(cnt_p++>10||1){cnt_p=0;
printf("connect=%d st=%d back=%d x=%d a=%d\n",
    ocu.connect+10*ocu.sbus_conncect ,  ocu.key_st,ocu.key_back, ocu.key_x,ocu.key_a );
printf("b=%d y=%d ll=%d rr=%d lr=%d up=%d\n",
    ocu.key_b ,  ocu.key_y,ocu.key_ll, ocu.key_rr,spi_rx.ocu.key_lr ,ocu.key_ud);
printf("spd0=%.2f spd1=%.2f att0=%.2f att1=%.2f yaw=%.2f\n",
    ocu.rc_spd_w[0],ocu.rc_spd_w[1],ocu.rc_att_w[0],ocu.rc_att_w[1],ocu.rate_yaw_w);
}
#endif


//nav_tx.request_gait	=		0;//ocu.up_mode;
ocu.rc_spd_b[Xr]=sdk.cmd_vx=nav_tx.exp_spd_o[0];
ocu.rc_spd_b[Yr]=sdk.cmd_vy=nav_tx.exp_spd_o[1];
ocu.rc_spd_b[Zr]=sdk.cmd_vz=nav_tx.exp_spd_o[2];

ocu.rc_rate_b[PITr]=nav_tx.exp_att_o[0];
ocu.rc_rate_b[ROLr]=nav_tx.exp_att_o[1];
ocu.rc_rate_b[YAWr]=sdk.cmd_vyaw=nav_tx.exp_datt_o[2];


if(spi_rx.ocu.connect||spi_rx.ocu.sbus_conncect){//ocu from stm32
    if(spi_rx.ocu.mode==RC_REMOTE){
        ocu.connect=spi_rx.ocu.connect;
        ocu.mode=spi_rx.ocu.mode;
        ocu.rc_spd_w[Xr]=spi_rx.ocu.rc_spd_w[0];
        ocu.rc_spd_w[Yr]=spi_rx.ocu.rc_spd_w[1];
        ocu.rc_att_w[PITr]=spi_rx.ocu.rc_att_w[0];
        ocu.rc_att_w[ROLr]= spi_rx.ocu.rc_att_w[1];
        ocu.rate_yaw_w= dead(spi_rx.ocu.rate_yaw_w,0.1);
        ocu.key_st=spi_rx.ocu.key_st;
        ocu.key_back=spi_rx.ocu.key_back;
        ocu.key_lr=spi_rx.ocu.key_lr;
        ocu.key_ud=spi_rx.ocu.key_ud;
        ocu.key_x=spi_rx.ocu.key_x;
        ocu.key_y=spi_rx.ocu.key_y;
        ocu.key_b=spi_rx.ocu.key_b;
        ocu.key_a= spi_rx.ocu.key_a;
        ocu.key_ll= spi_rx.ocu.key_ll;
        ocu.key_rr=spi_rx.ocu.key_rr;
    }else if(spi_rx.ocu.mode==RC_SBUS){
        ocu.connect=spi_rx.ocu.sbus_conncect;
        ocu.mode=spi_rx.ocu.mode;
        ocu.sbus_ch[0]=spi_rx.ocu.sbus_ch[0];
        ocu.sbus_ch[1]=spi_rx.ocu.sbus_ch[1];
        ocu.sbus_ch[2]=spi_rx.ocu.sbus_ch[2];
        ocu.sbus_ch[3]=spi_rx.ocu.sbus_ch[3];
        ocu.sbus_ch[4]=spi_rx.ocu.sbus_ch[4];
        ocu.sbus_ch[5]=spi_rx.ocu.sbus_ch[5];
        ocu.sbus_aux[0]=spi_rx.ocu.sbus_aux[0];
        ocu.sbus_aux[1]=spi_rx.ocu.sbus_aux[1];
        ocu.sbus_aux[2]=spi_rx.ocu.sbus_aux[2];
        ocu.sbus_aux[3]=spi_rx.ocu.sbus_aux[3];
        ocu.sbus_aux[4]=spi_rx.ocu.sbus_aux[4];
        ocu.sbus_aux[5]=spi_rx.ocu.sbus_aux[5];
        sbus_mode_anal();
//        Rc_Get_SBUS.SBUS_CH[0]=LIMIT((float)(Rc_Get_SBUS.THROTTLE-1500)/500.0,-1,1);
//        Rc_Get_SBUS.SBUS_CH[1]=LIMIT((float)(Rc_Get_SBUS.PITCH-1500)/500.0,-1,1);
//        Rc_Get_SBUS.SBUS_CH[2]=LIMIT((float)(Rc_Get_SBUS.ROLL-1500)/500.0,-1,1);
//        Rc_Get_SBUS.SBUS_CH[3]=LIMIT((float)(Rc_Get_SBUS.YAW-1500)/500.0,-1,1);
    }
}
//printf("rcMode:%d %d %d\n",spi_rx.ocu.connect,spi_rx.ocu.sbus_conncect,spi_rx.ocu.mode);
sdk.cmd_pit=nav_tx.exp_att_o[0];
sdk.cmd_rol=nav_tx.exp_att_o[1];
sdk.cmd_yaw=nav_tx.exp_att_o[2];

sdk.cmd_x=nav_tx.exp_pos_o[0];
sdk.cmd_y=nav_tx.exp_pos_o[1];
sdk.cmd_z=nav_tx.exp_pos_o[2];
}

//------------------------------内存管理--Hardware----------------------feedback important!!!!
void memory_read(void){//读取STM 回传传感器和 编码器 力矩
    char temp;
    mem_read_cnt=0;
    spi_rx.att[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.att[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.att[2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.att_rate[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.att_rate[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.att_rate[2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.acc_b[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.acc_b[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.acc_b[2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.acc_n[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.acc_n[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.acc_n[2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    //imu
    spi_rx.imu_mems_connect = charFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.att_usb[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.att_usb[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.att_usb[2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.att_rate_usb[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.att_rate_usb[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.att_rate_usb[2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.acc_b_usb[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.acc_b_usb[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.acc_b_usb[2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);

    if(spi_rx.imu_mems_connect
           //&&vmc_all.side_flip
            ){//&&vmc_all.side_flip){
        for(int i=0;i<3;i++){
            if(fabs( spi_rx.att_usb[0])<180&&fabs( spi_rx.att_usb[1])<180&&fabs( spi_rx.att_usb[2])<360){
                spi_rx.att[i]=spi_rx.att_usb[i];
                spi_rx.att_rate[i]=spi_rx.att_rate_usb[i];
                spi_rx.acc_b[i]=spi_rx.acc_b_usb[i];
            }
        }
    }

    //leg
    for (int i = 0; i < 4; i++)
    {
        spi_rx.q[i][0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
        spi_rx.q[i][1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
        spi_rx.q[i][2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
        spi_rx.tau[i][0] = floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
        spi_rx.tau[i][1] = floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
        spi_rx.tau[i][2] = floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;

    //    if(i==1&&spi_rx.tau[i][0]!=0)
    //        printf("use::%f %f %f\n", spi_rx.tau[i][0], spi_rx.tau[i][1], spi_rx.tau[i][2]);
        spi_rx.bat_v[i] = floatFromData_spi(mem_read_buf, &mem_read_cnt);

        temp= charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
        spi_rx.connect[i] = temp/100%10;
        spi_rx.connect_motor[i][0] = (temp-spi_rx.connect[i] *100)/10;
        spi_rx.ready[i][0] = temp%10;

        temp= charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
        spi_rx.connect[i] = temp/100%10;
        spi_rx.connect_motor[i][1] = (temp-spi_rx.connect[i] *100)/10;
        spi_rx.ready[i][1] = temp%10;

        temp= charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
        spi_rx.connect[i] = temp/100%10;
        spi_rx.connect_motor[i][2] = (temp-spi_rx.connect[i] *100)/10;
        spi_rx.ready[i][2] = temp%10;
       // printf("%d %d %d %d\n",spi_rx.connect[i],spi_rx.connect_motor[i][0],spi_rx.connect_motor[i][1],spi_rx.connect_motor[i][2]);
    }


    //SBUS stm32
    #if SBUS_YUNZHUO
    temp= charFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.sbus_conncect = temp/100;
    spi_rx.ocu.sbus_aux[0] = (temp-int(temp/100) *100)/10;
    spi_rx.ocu.sbus_aux[1] = temp%10;

    temp= charFromData_spi(mem_read_buf, &mem_read_cnt);
    // spi_rx.ocu.sbus_aux[0] = temp/100;
    spi_rx.ocu.sbus_aux[2] = (temp-int(temp/100) *100)/10;
    spi_rx.ocu.sbus_aux[3] = temp%10;

    temp= charFromData_spi(mem_read_buf, &mem_read_cnt);
    // spi_rx.ocu.sbus_aux[0] = temp/100;
    spi_rx.ocu.sbus_aux[4] = (temp-int(temp/100) *100)/10;
    spi_rx.ocu.sbus_aux[5] = temp%10;
    spi_rx.ocu.sbus_ch[0] =dead(Moving_Median(31,  3,  floatFromData_spi(mem_read_buf, &mem_read_cnt)),0.05);
    spi_rx.ocu.sbus_ch[1] =dead(-Moving_Median(32,  3,  floatFromData_spi(mem_read_buf, &mem_read_cnt)),0.05);
    spi_rx.ocu.sbus_ch[2] =dead(Moving_Median(33,  3,  floatFromData_spi(mem_read_buf, &mem_read_cnt)),0.05);
    spi_rx.ocu.sbus_ch[3] =dead(Moving_Median(34,  3,  floatFromData_spi(mem_read_buf, &mem_read_cnt)),0.05);
    spi_rx.ocu.sbus_ch[4] =dead(Moving_Median(35,  3,  floatFromData_spi(mem_read_buf, &mem_read_cnt)),0.05);
    spi_rx.ocu.sbus_ch[5] =dead(Moving_Median(36,  3,  floatFromData_spi(mem_read_buf, &mem_read_cnt)),0.05);
    #else
    //SBUS
    temp= charFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.sbus_conncect = temp/100;
    spi_rx.ocu.sbus_aux[0] = (temp-int(temp/100) *100)/10;
    spi_rx.ocu.sbus_aux[1] = temp%10;

    temp= charFromData_spi(mem_read_buf, &mem_read_cnt);
    // spi_rx.ocu.sbus_aux[0] = temp/100;
    spi_rx.ocu.sbus_aux[2] = (temp-int(temp/100) *100)/10;
    spi_rx.ocu.sbus_aux[3] = temp%10;

    temp= charFromData_spi(mem_read_buf, &mem_read_cnt);
    // spi_rx.ocu.sbus_aux[0] = temp/100;
    spi_rx.ocu.sbus_aux[4] = (temp-int(temp/100) *100)/10;
    spi_rx.ocu.sbus_aux[5] = temp%10;
    spi_rx.ocu.sbus_ch[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.sbus_ch[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.sbus_ch[2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.sbus_ch[3] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.sbus_ch[4] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.ocu.sbus_ch[5] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    #endif
    if(spi_rx.ocu.connect&&!spi_rx.ocu.sbus_conncect)
        spi_rx.ocu.mode=RC_REMOTE;
    else if(spi_rx.ocu.sbus_conncect)
        spi_rx.ocu.mode=RC_SBUS;

    //AOA
    spi_rx.aoa.angle= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.aoa.dis= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.aoa.rssi= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    //Wheel
    spi_rx.w_dq_now[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.w_dq_now[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.w_dq_now[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.w_dq_now[3]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.w_tau_now[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.w_tau_now[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.w_tau_now[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.w_tau_now[3]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
#if 0
    printf("w_connect:%d%d%d%d\n",spi_rx.w_connect[0],spi_rx.w_connect[1],spi_rx.w_connect[2],spi_rx.w_connect[3]);
    printf("wd: %.3f %.3f %.3f %.3f\n",spi_rx.w_dq_now[0],spi_rx.w_dq_now[1],spi_rx.w_dq_now[2],spi_rx.w_dq_now[3]);
    printf("tau:%.3f %.3f %.3f %.3f\n",spi_rx.w_tau_now[0],spi_rx.w_tau_now[1],spi_rx.w_tau_now[2],spi_rx.w_tau_now[3]);
#endif
    temp= charFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.w_connect[0] = (temp-int(temp/100) *100)/10;
    spi_rx.w_connect[1] = temp%10;
    temp= charFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_rx.w_connect[2] = (temp-int(temp/100) *100)/10;
    spi_rx.w_connect[3] = temp%10;

    #if 0
    printf("%f %f %d\n",spi_rx.aoa.dis,spi_rx.aoa.angle,spi_rx.aoa.rssi);
    #endif
    #if 0
      printf("connect=%d st=%d back=%d x=%d a=%d\n",
          spi_rx.ocu.sbus_conncect ,  spi_rx.ocu.sbus_aux[0],spi_rx.ocu.sbus_aux[1], spi_rx.ocu.sbus_aux[2],spi_rx.ocu.sbus_aux[3] );
      printf("rc0=%f rc1=%f rc2=%f rc3=%f\n",
          spi_rx.ocu.sbus_ch[0],spi_rx.ocu.sbus_ch[1],spi_rx.ocu.sbus_ch[2],spi_rx.ocu.sbus_ch[3]);
    #endif
    #if 0
    printf("connect=%d st=%d back=%d x=%d a=%d\n",
        spi_rx.ocu.connect ,  spi_rx.ocu.key_st,spi_rx.ocu.key_back, spi_rx.ocu.key_x,spi_rx.ocu.key_a );
    printf("b=%d y=%d ll=%d rr=%d lr=%d up=%d\n",
        spi_rx.ocu.key_b ,  spi_rx.ocu.key_y,spi_rx.ocu.key_ll, spi_rx.ocu.key_rr,spi_rx.ocu.key_lr ,spi_rx.ocu.key_ud);
    printf("spd0=%f spd1=%f att0=%f att1=%f yaw=%f\n",
        spi_rx.ocu.rc_spd_w[0],spi_rx.ocu.rc_spd_w[1],spi_rx.ocu.rc_att_w[0],spi_rx.ocu.rc_att_w[1],spi_rx.ocu.rate_yaw_w);
    #endif
    //----------------------数据转化--------------------
    robotwb.IMU_now_o.pitch=spi_rx.att[0];
    if(vmc_all.side_flip)
        robotwb.IMU_now_o.roll=spi_rx.att[1];
    else
        robotwb.IMU_now_o.roll=spi_rx.att[1];

    robotwb.IMU_now_o.yaw=spi_rx.att[2];
    robotwb.IMU_dot_o.pitch=spi_rx.att_rate[0];
    robotwb.IMU_dot_o.roll=spi_rx.att_rate[1];
    robotwb.IMU_dot_o.yaw=spi_rx.att_rate[2];
    robotwb.cog_acc_b.x=spi_rx.acc_b[0];
    robotwb.cog_acc_b.y=spi_rx.acc_b[1];
    robotwb.cog_acc_b.z=spi_rx.acc_b[2];

    for (int i = 0; i < 4; i++){
        leg_motor[i].connect=spi_rx.connect[0];//node connect

        leg_motor[i].connect_motor[0]=spi_rx.connect_motor[0][0];
        leg_motor[i].ready[0]=spi_rx.ready[0][0];
        leg_motor[i].connect_motor[1]=spi_rx.connect_motor[0][1];
        leg_motor[i].ready[1]=spi_rx.ready[0][1];
        leg_motor[i].connect_motor[2]=spi_rx.connect_motor[0][2];
        leg_motor[i].ready[2]=spi_rx.ready[0][2];

        leg_motor[i].q_now[0]=spi_rx.q[i][0];
        leg_motor[i].q_now[1]=spi_rx.q[i][1];
        leg_motor[i].q_now[2]=spi_rx.q[i][2];
        leg_motor[i].t_now[0]=Moving_Median(50+i*3+0,  3,  spi_rx.tau[i][0]);
        leg_motor[i].t_now[1]=Moving_Median(50+i*3+1,  3,  spi_rx.tau[i][1]);
        leg_motor[i].t_now[2]=Moving_Median(50+i*3+2,  3,  spi_rx.tau[i][2]);
    }
    //printf("%f %f %f\n",leg_motor[0].t_now[0],leg_motor[0].t_now[1],leg_motor[0].t_now[2]);
    arm_motor[0].ready[0]=leg_motor[0].ready[0];
    arm_motor[0].ready[1]=leg_motor[0].ready[1];
    arm_motor[0].ready[2]=leg_motor[0].ready[2];
    arm_motor[0].ready[3]=1;
    arm_motor[0].ready[4]=1;
    arm_motor[0].ready[5]=1;

    wheel_motor.ready[0]=1;
    wheel_motor.ready[1]=1;
    wheel_motor.ready[2]=1;
    wheel_motor.ready[3]=1;
    //printf("att0=%f att1=%f att2=%f dt=%f\n",spi_rx.att[0],spi_rx.att[1],spi_rx.att[2], Get_Cycle_T(0));
}

void memory_write(void)//写入Memeory to STM32 期望力 角度和系统参数 action
{
    static float t_temp=0;
    static float temp=0;
    mem_write_cnt=MEM_SIZE/2;
    char id_swap_flig[4]={2,3,0,1};
    float q_tar_temp[3]={0};
    //-------------------------数据转化------------------------
    for(int id=0;id<4;id++)
    {
        #if !SPI_TEST
        spi_tx.q_set[id][0]=robotwb.Leg[id].tar_sita[0];//dq for z-height
        spi_tx.q_set[id][1]=robotwb.Leg[id].tar_sita[1];
        spi_tx.q_set[id][2]=robotwb.Leg[id].tar_sita[2];

        spi_tx.q_reset[id][0]=leg_motor[id].q_reset[0];
        spi_tx.q_reset[id][1]=leg_motor[id].q_reset[1];
        spi_tx.q_reset[id][2]=leg_motor[id].q_reset[2]*vmc[id].flag_rl;
    #if MESS_FIX_TEST
           if(id==MESS_FIX_ID||MESS_FIX_ID==99){
            spi_tx.tau_ff[id][0]=leg_motor[id].set_t[0]*mem_connect;//*vmc_all.param.soft_weight;
            spi_tx.tau_ff[id][1]=leg_motor[id].set_t[1]*mem_connect;//*vmc_all.param.soft_weight;
            spi_tx.tau_ff[id][2]=leg_motor[id].set_t[2]*mem_connect;//*vmc_all.param.soft_weight;
           }
           else{
            spi_tx.tau_ff[id][0]=leg_motor[id].set_t[0]*mem_connect*vmc_all.param.soft_weight;
            spi_tx.tau_ff[id][1]=leg_motor[id].set_t[1]*mem_connect*vmc_all.param.soft_weight;
            spi_tx.tau_ff[id][2]=leg_motor[id].set_t[2]*mem_connect*vmc_all.param.soft_weight;
           }
    #else
            spi_tx.tau_ff[id][0]=leg_motor[id].set_t[0]*mem_connect*vmc_all.param.soft_weight;
            spi_tx.tau_ff[id][1]=leg_motor[id].set_t[1]*mem_connect*vmc_all.param.soft_weight;
            spi_tx.tau_ff[id][2]=leg_motor[id].set_t[2]*mem_connect*vmc_all.param.soft_weight;

    #endif
            //if(id==1)
           // printf("%f %f %f\n",spi_tx.tau_ff[id][0],spi_tx.tau_ff[id][1],spi_tx.tau_ff[id][2]);
        #else
            t_temp += 0.001 * 1;
            spi_tx.q_set[id][0]=(2 * sin(t_temp));
            spi_tx.q_set[id][1]=(-2 * sin(t_temp));
            spi_tx.tau_ff[id][0]=(id*1.3+2);
            spi_tx.tau_ff[id][1]=(id*-2.3+1);
        #endif
        spi_tx.t_to_i=leg_motor[id].t_to_i[0];
        spi_tx.max_i=leg_motor[id].max_i[0];

        spi_tx.kp=robotwb.Leg[id].q_pid.kp*vmc_all.param.soft_weight;
        spi_tx.ki=robotwb.Leg[id].q_pid.ki*vmc_all.param.soft_weight;
        spi_tx.kd=robotwb.Leg[id].q_pid.kd*vmc_all.param.soft_weight;

        spi_tx.kp_sw=robotwb.Leg[id].q_pid.kp_sw*vmc_all.param.soft_weight;
        spi_tx.ki_sw=robotwb.Leg[id].q_pid.ki_sw*vmc_all.param.soft_weight;
        spi_tx.kd_sw=robotwb.Leg[id].q_pid.kd_sw*vmc_all.param.soft_weight;

        spi_tx.kp_st=robotwb.Leg[id].q_pid.kp_st*vmc_all.param.soft_weight;
        spi_tx.ki_st=robotwb.Leg[id].q_pid.ki_st*vmc_all.param.soft_weight;
        spi_tx.kd_st=robotwb.Leg[id].q_pid.kd_st*vmc_all.param.soft_weight;

        //0------PVT param
        spi_tx.kp_sw_d[0]=robotwb.Leg[id].q_pid.kp_sw_d[0]*vmc_all.param.soft_weight;
        spi_tx.ki_sw_d[0]=robotwb.Leg[id].q_pid.ki_sw_d[0]*vmc_all.param.soft_weight;
        spi_tx.kd_sw_d[0]=robotwb.Leg[id].q_pid.kd_sw_d[0]*vmc_all.param.soft_weight;

        spi_tx.kp_st_d[0]=robotwb.Leg[id].q_pid.kp_st_d[0]*vmc_all.param.soft_weight;
        spi_tx.ki_st_d[0]=robotwb.Leg[id].q_pid.ki_st_d[0]*vmc_all.param.soft_weight;
        spi_tx.kd_st_d[0]=robotwb.Leg[id].q_pid.kd_st_d[0]*vmc_all.param.soft_weight;

        //1
        spi_tx.kp_sw_d[1]=robotwb.Leg[id].q_pid.kp_sw_d[1]*vmc_all.param.soft_weight;
        spi_tx.ki_sw_d[1]=robotwb.Leg[id].q_pid.ki_sw_d[1]*vmc_all.param.soft_weight;
        spi_tx.kd_sw_d[1]=robotwb.Leg[id].q_pid.kd_sw_d[1]*vmc_all.param.soft_weight;

        spi_tx.kp_st_d[1]=robotwb.Leg[id].q_pid.kp_st_d[1]*vmc_all.param.soft_weight;
        spi_tx.ki_st_d[1]=robotwb.Leg[id].q_pid.ki_st_d[1]*vmc_all.param.soft_weight;
        spi_tx.kd_st_d[1]=robotwb.Leg[id].q_pid.kd_st_d[1]*vmc_all.param.soft_weight;

        //2
        spi_tx.kp_sw_d[2]=robotwb.Leg[id].q_pid.kp_sw_d[2]*vmc_all.param.soft_weight;
        spi_tx.ki_sw_d[2]=robotwb.Leg[id].q_pid.ki_sw_d[2]*vmc_all.param.soft_weight;
        spi_tx.kd_sw_d[2]=robotwb.Leg[id].q_pid.kd_sw_d[2]*vmc_all.param.soft_weight;

        spi_tx.kp_st_d[2]=robotwb.Leg[id].q_pid.kp_st_d[2]*vmc_all.param.soft_weight;
        spi_tx.ki_st_d[2]=robotwb.Leg[id].q_pid.ki_st_d[2]*vmc_all.param.soft_weight;
        spi_tx.kd_st_d[2]=robotwb.Leg[id].q_pid.kd_st_d[2]*vmc_all.param.soft_weight;
#if 0
        printf("PID[%d]:%f %f %f\n",id,robotwb.Leg[id].q_pid.kp,robotwb.Leg[id].q_pid.kp_sw,robotwb.Leg[id].q_pid.kp_st);
        printf("PIDd[%d]:%f %f %f\n",id,robotwb.Leg[id].q_pid.kp_sw_d[0],robotwb.Leg[id].q_pid.kp_sw_d[1],robotwb.Leg[id].q_pid.kp_sw_d[2]);
#endif
        spi_tx.param_sel[id]=robotwb.Leg[id].q_pid.param_sel;//ID isolate chose sw & st param

        spi_tx.en_motor=leg_motor[id].motor_en;
        spi_tx.reser_q=leg_motor[id].reset_q;
        spi_tx.reset_err=leg_motor[id].reset_err;
    }

    //------------------------写入内存--------------------------------------
    for (int i = 0; i < 4; i++)
    {
        setDataFloat_mem( spi_tx.q_set[i][0],&mem_write_cnt);//dq for z-height
        setDataFloat_mem( spi_tx.q_set[i][1],&mem_write_cnt);
        setDataFloat_mem( spi_tx.q_set[i][2],&mem_write_cnt);
        setDataFloat_mem( spi_tx.q_reset[i][0],&mem_write_cnt);
        setDataFloat_mem( spi_tx.q_reset[i][1],&mem_write_cnt);
        setDataFloat_mem( spi_tx.q_reset[i][2],&mem_write_cnt);
        setDataFloat_mem( spi_tx.tau_ff[i][0],&mem_write_cnt);
        setDataFloat_mem( spi_tx.tau_ff[i][1],&mem_write_cnt);
        setDataFloat_mem( spi_tx.tau_ff[i][2],&mem_write_cnt);
        mem_write_buf[mem_write_cnt++] = spi_tx.param_sel[i];
    }

    setDataFloat_mem( spi_tx.t_to_i,&mem_write_cnt);
    setDataFloat_mem( spi_tx.max_i,&mem_write_cnt);

    setDataFloat_mem( spi_tx.kp,&mem_write_cnt);
    setDataFloat_mem( spi_tx.ki,&mem_write_cnt);
    setDataFloat_mem( spi_tx.kd,&mem_write_cnt);

    setDataFloat_mem( spi_tx.kp_sw,&mem_write_cnt);
    setDataFloat_mem( spi_tx.ki_sw,&mem_write_cnt);
    setDataFloat_mem( spi_tx.kd_sw,&mem_write_cnt);

    setDataFloat_mem( spi_tx.kp_st,&mem_write_cnt);
    setDataFloat_mem( spi_tx.ki_st,&mem_write_cnt);
    setDataFloat_mem( spi_tx.kd_st,&mem_write_cnt);

    //-0------------PVT param
    setDataFloat_mem( spi_tx.kp_sw_d[0],&mem_write_cnt);
    setDataFloat_mem( spi_tx.ki_sw_d[0],&mem_write_cnt);
    setDataFloat_mem( spi_tx.kd_sw_d[0],&mem_write_cnt);

    setDataFloat_mem( spi_tx.kp_st_d[0],&mem_write_cnt);
    setDataFloat_mem( spi_tx.ki_st_d[0],&mem_write_cnt);
    setDataFloat_mem( spi_tx.kd_st_d[0],&mem_write_cnt);
    //-1
    setDataFloat_mem( spi_tx.kp_sw_d[1],&mem_write_cnt);
    setDataFloat_mem( spi_tx.ki_sw_d[1],&mem_write_cnt);
    setDataFloat_mem( spi_tx.kd_sw_d[1],&mem_write_cnt);

    setDataFloat_mem( spi_tx.kp_st_d[1],&mem_write_cnt);
    setDataFloat_mem( spi_tx.ki_st_d[1],&mem_write_cnt);
    setDataFloat_mem( spi_tx.kd_st_d[1],&mem_write_cnt);
    //-2
    setDataFloat_mem( spi_tx.kp_sw_d[2],&mem_write_cnt);
    setDataFloat_mem( spi_tx.ki_sw_d[2],&mem_write_cnt);
    setDataFloat_mem( spi_tx.kd_sw_d[2],&mem_write_cnt);

    setDataFloat_mem( spi_tx.kp_st_d[2],&mem_write_cnt);
    setDataFloat_mem( spi_tx.ki_st_d[2],&mem_write_cnt);
    setDataFloat_mem( spi_tx.kd_st_d[2],&mem_write_cnt);


    mem_write_buf[mem_write_cnt++] = spi_tx.en_motor;
    mem_write_buf[mem_write_cnt++] = spi_tx.reser_q;
    mem_write_buf[mem_write_cnt++] = spi_tx.reset_err;
    mem_write_buf[mem_write_cnt++] = spi_tx.led_enable[0]*10+ spi_tx.led_enable[1];
    mem_write_buf[mem_write_cnt++] = spi_tx.led_side_enable[0];
    mem_write_buf[mem_write_cnt++] = spi_tx.led_side_enable[1];//side led
    //--------------------Param From OCU-------------
    setDataFloat_mem( mems.imu_pos.x,&mem_write_cnt);
    setDataFloat_mem( mems.imu_pos.y,&mem_write_cnt);
    setDataFloat_mem( mems.imu_pos.z,&mem_write_cnt);
    setDataFloat_mem( mems.imu_att.x,&mem_write_cnt);
    setDataFloat_mem( mems.imu_att.y,&mem_write_cnt);
    setDataFloat_mem( mems.imu_att.z,&mem_write_cnt);
    setDataFloat_mem( mems.gps_pos.x,&mem_write_cnt);
    setDataFloat_mem( mems.gps_pos.y,&mem_write_cnt);
    setDataFloat_mem( mems.gps_pos.z,&mem_write_cnt);
    if(ocu.sbus_cal_mems==1)
        mems.Acc_CALIBRATE=mems.Gyro_CALIBRATE=1;

    mem_write_buf[mem_write_cnt++] = mems.Acc_CALIBRATE;
    mem_write_buf[mem_write_cnt++] = mems.Gyro_CALIBRATE;
    mem_write_buf[mem_write_cnt++] = mems.Mag_CALIBRATE;
    mem_write_buf[mem_write_cnt++] = spi_tx.beep_state= robotwb.beep_state;

    #if 0//test head and hand control
                spi_tx.arm_head3.att_set[0]=45;//p
                spi_tx.arm_head3.att_set[1]=0;
                spi_tx.arm_head3.att_set[2]=0;
                spi_tx.arm_head3.head_set[0]=0;
                spi_tx.arm_head3.head_set[1]=0;
                spi_tx.arm_head3.head_set[2]=0;
                spi_tx.arm_head3.cap=1.0;
    #endif

    setDataFloat_mem( spi_tx.arm_head3.att_set[0],&mem_write_cnt);
    setDataFloat_mem( spi_tx.arm_head3.att_set[1],&mem_write_cnt);
    setDataFloat_mem( spi_tx.arm_head3.att_set[2],&mem_write_cnt);
    setDataFloat_mem( spi_tx.arm_head3.head_set[0],&mem_write_cnt);
    setDataFloat_mem( spi_tx.arm_head3.head_set[1],&mem_write_cnt);
    setDataFloat_mem( spi_tx.arm_head3.head_set[2],&mem_write_cnt);
    mem_write_buf[mem_write_cnt++] =spi_tx.arm_head3.power;
    setDataFloat_mem( spi_tx.arm_head3.cap,&mem_write_cnt);
#if 0
 spi_tx.w_dq_exp[0]= spi_tx.w_dq_exp[1]= spi_tx.w_dq_exp[2]= spi_tx.w_dq_exp[3]=0.72;
#endif

    //---wheel control
    setDataFloat_mem( spi_tx.w_dq_exp[0],&mem_write_cnt);
    setDataFloat_mem( spi_tx.w_dq_exp[1],&mem_write_cnt);
    setDataFloat_mem( spi_tx.w_dq_exp[2],&mem_write_cnt);
    setDataFloat_mem( spi_tx.w_dq_exp[3],&mem_write_cnt);
    setDataFloat_mem( spi_tx.w_tau_exp[0],&mem_write_cnt);
    setDataFloat_mem( spi_tx.w_tau_exp[1],&mem_write_cnt);
    setDataFloat_mem( spi_tx.w_tau_exp[2],&mem_write_cnt);
    setDataFloat_mem( spi_tx.w_tau_exp[3],&mem_write_cnt);

    mem_write_buf[mem_write_cnt++] =brain_connect_c;
}


void* Thread_Mem_Servo(void*)//内存管理线程
{
    float timer[5]={0};
    float sys_dt = 0,dT=0;
    static int mem_init_cnt=0;
    int i=0;
    int link_cnt=0;
    int shmid_rx = shmget((key_t)MEM_SPI, sizeof(shareMemory_spi), 0666|IPC_CREAT); //失败返回-1，假设成功
    void *shm_rx = shmat(shmid_rx, 0, 0);
    shareMemory *pshm_rx = (shareMemory*)shm_rx;
    printf("Control::Memory Hardware attached at %p\n",shm_rx);

    while(1)
    {
       sys_dt = Get_Cycle_T(0);
       timer[0]+=sys_dt;

       //共享内存读取 to Servo task
        if(pshm_rx->flag == 1)
        {
            if(!mem_connect){
            mem_init_cnt++;
            if(mem_init_cnt>2){
                printf("Control::Memory Hardware Link=%d!!!\n",link_cnt++);
                mem_connect=1;
            }
            }
            mem_loss_cnt=0;
#if EN_THREAD_LOCK
            pthread_mutex_lock(&lock);
#endif
            for(int k=0;k<MEM_SIZE/2-1;k++)
                mem_read_buf[k]=pshm_rx->szMsg[k];

            memory_read();
            memory_write();
#if EN_THREAD_LOCK
            pthread_mutex_unlock(&lock);
#endif
            for(int k=MEM_SIZE/2;k<MEM_SIZE;k++)
                pshm_rx->szMsg[k]=mem_write_buf[k];

            pshm_rx->flag = 0;
        }

        mem_loss_cnt+=sys_dt;
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
            printf("Control::Memery Hardware Loss!!!\n");
        }
        //printf("%f\n",mem_loss_cnt);
        usleep(500);
    }

    shmdt(shm_rx);  //失败返回-1，假设成功
    shmctl(shmid_rx, IPC_RMID, 0);  //失败返回-1，假设成功。仅在reader这里删除共享内存，保证读完最后一个消息
    return 0;
}


void* Thread_Mem_Navigation(void*)//内存管理线程
{
    float timer[5]={0};
    float sys_dt = 0,dT=0;
    static int  mem_init_cnt=0;
    int i=0;
    int link_cnt=0;
    int shmid_rx_c = shmget((key_t)MEM_CONTROL, sizeof(shareMemory_control), 0666|IPC_CREAT); //失败返回-1，假设成功
    void *shm_rx_c = shmat(shmid_rx_c, 0, 0);
    shareMemory *pshm_rx_c = (shareMemory*)shm_rx_c;
    pshm_rx_c->flag = 0;
    printf("Control::Memory Control attached at %p\n",shm_rx_c);

    while(1)
    {
       sys_dt = Get_Cycle_T(1);
       timer[0]+=sys_dt;

         //共享内存读取 to Nav task
        if(pshm_rx_c->flag == 0)
        {
            if(!mem_connect_c){
            mem_init_cnt++;
                if(mem_init_cnt>2){
                printf("Control::Memory Navigaition Link=%d!!!\n",link_cnt++);
                mem_connect_c=1;
                }
            }
            mem_loss_cnt_c=0;
            memory_write_c(0);
            for(int k=0;k<MEM_SIZE/2-1;k++)
                pshm_rx_c->szMsg[k]=mem_write_buf_c[k];
            for(int k=MEM_SIZE/2;k<MEM_SIZE;k++)
                mem_read_buf_c[k]=pshm_rx_c->szMsg[k];
            memory_read_c(MEM_SIZE/2);
            pshm_rx_c->flag = 1;
        }else
            mem_loss_cnt_c+=sys_dt;

        if(mem_loss_cnt_c>1&&mem_connect_c==1){
            mem_connect_c=0;
            mem_loss_cnt_c=0;
            mem_init_cnt=0;
            printf("Control::Memory Navigaition Loss!!!\n");
        }
        //usleep(200);
        usleep(1000*5);
    }

    shmdt(shm_rx_c);  //失败返回-1，假设成功
    shmctl(shmid_rx_c, IPC_RMID, 0);  //失败返回-1，假设成功。仅在reader这里删除共享内存，保证读完最后一个消息
    return 0;
}
