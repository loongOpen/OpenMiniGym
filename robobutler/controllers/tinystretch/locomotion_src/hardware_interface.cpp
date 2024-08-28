#include "include.h"
#include "locomotion_header.h"
#include "gait_math.h"
#include "adrc.h"
#include "base_struct.h"
#include "spi_node.h"
#if RUN_WEBOTS
#include "wbInterface.h"
#endif
//测量陀螺仪数据    弧度制

float FLT_ATT_RT = 0;//不为0翻转站立无法正常
float min_q0[3]={0};
float max_q0[3]={0};
float min_q1[3]={0};
float max_q1[3]={0};
float min_q2[3]={0};
float max_q2[3]={0};

void subscribe_imu_to_webot(robotTypeDef* rob,float dt)
{
#if RUN_WEBOTS
    const double* data = wb_inertial_unit_get_roll_pitch_yaw(IMU);
    rob->IMU_now_o.roll = 90 + deg(data[0]);
    rob->IMU_now_o.yaw = deg(data[2]);
    rob->IMU_now_o.pitch = deg(data[1]);

    rob->IMU_now.roll = -(90 + deg(data[0]));
    rob->IMU_now.yaw = To_180_degreesw(deg(data[2]) - 90);
    rob->IMU_now.pitch = deg(data[1]);

    rob->IMU_dot.roll = (rob->IMU_now.roll - rob->IMU_last.roll) / dt;
    if(fabs(rob->IMU_now.yaw - rob->IMU_last.yaw)<180)
        rob->IMU_dot.yaw = (rob->IMU_now.yaw - rob->IMU_last.yaw) / dt;
    rob->IMU_dot.pitch = (rob->IMU_now.pitch - rob->IMU_last.pitch) / dt;

    rob->IMU_last.roll  = rob->IMU_now.roll;
    rob->IMU_last.yaw   = rob->IMU_now.yaw;
    rob->IMU_last.pitch = rob->IMU_now.pitch;
    rob->now_att = rob->IMU_now;
    rob->now_rate = rob->IMU_dot;
    //printf("pit =%f rol=%f yaw=%f ||dpit =%f drol=%f dyaw=%f\n", rob->IMU_now.pitch, rob->IMU_now.roll,rob->IMU_now.yaw,rob->IMU_dot.pitch, rob->IMU_dot.roll, rob->IMU_dot.yaw);
#else
    robotwb.IMU_now.pitch= robotwb.IMU_now_o.pitch;
    robotwb.IMU_now.roll= -robotwb.IMU_now_o.roll;
    robotwb.IMU_now.yaw= -robotwb.IMU_now_o.yaw;
    robotwb.IMU_dot.pitch= robotwb.IMU_dot_o.pitch;
    robotwb.IMU_dot.roll= -robotwb.IMU_dot_o.roll;
    robotwb.IMU_dot.yaw= robotwb.IMU_dot_o.yaw;
#endif
    //printf("pit =%f rol=%f yaw=%f ||dpit =%f drol=%f dyaw=%f\n", rob->IMU_now.pitch, rob->IMU_now.roll,rob->IMU_now.yaw,rob->IMU_dot.pitch, rob->IMU_dot.roll, rob->IMU_dot.yaw);

    vmc_all.att[PITr]= robotwb.IMU_now.pitch;
    vmc_all.att[ROLr]= robotwb.IMU_now.roll;
    vmc_all.att[YAWr]= robotwb.IMU_now.yaw;

    vmc_all.att_rate[PITr]=robotwb.IMU_dot.pitch;
    vmc_all.att_rate[ROLr]=robotwb.IMU_dot.roll;
    vmc_all.att_rate[YAWr]=robotwb.IMU_dot.yaw;

    static float att_rt_use[3] = {0};
    static char flip_flag_reg = 0;
    if (vmc_all.side_flip != flip_flag_reg) {
        att_rt_use[PITr] = vmc_all.att[PITr];
        att_rt_use[ROLr] = vmc_all.att[ROLr];
        att_rt_use[YAWr] = vmc_all.att[YAWr];
    }
    DigitalLPF(vmc_all.att[PITr], &att_rt_use[PITr], FLT_ATT_RT, dt);
    DigitalLPF(vmc_all.att[ROLr], &att_rt_use[ROLr], FLT_ATT_RT, dt);
    DigitalLPF(vmc_all.att[YAWr], &att_rt_use[YAWr], FLT_ATT_RT, dt);

    robotwb.IMU_now_e_off.yaw= To_180_degrees(vmc_all.att[YAWr]-robotwb.e_yaw_off);
    att_rt_use[YAWr] = To_180_degrees(vmc_all.att[YAWr]-robotwb.e_yaw_off);
    robotwb.Rn_b[0][0] = cosd(-att_rt_use[PITr])*cosd(att_rt_use[YAWr]);
    robotwb.Rn_b[1][0] = -cosd(-att_rt_use[ROLr])*sind(att_rt_use[YAWr]) + sind(-att_rt_use[PITr])*sind(-att_rt_use[ROLr])*cosd(att_rt_use[YAWr]);
    robotwb.Rn_b[2][0] = sind(-att_rt_use[ROLr])*sind(att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*cosd(att_rt_use[YAWr]);

    robotwb.Rn_b[0][1] = cosd(-att_rt_use[PITr])*sind(att_rt_use[YAWr]);
    robotwb.Rn_b[1][1] = cosd(-att_rt_use[ROLr])*cosd(att_rt_use[YAWr]) + sind(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(att_rt_use[YAWr]);
    robotwb.Rn_b[2][1] = -sind(-att_rt_use[ROLr])*cosd(att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(att_rt_use[YAWr]);

    robotwb.Rn_b[0][2] = -sind(-att_rt_use[PITr]);
    robotwb.Rn_b[1][2] = sind(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);
    robotwb.Rn_b[2][2] = cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);

    mat_trans(robotwb.Rn_b, robotwb.Rb_n);

    att_rt_use[YAWr] = 0;
    robotwb.Rn_b_noyaw[0][0] = cosd(-att_rt_use[PITr])*cosd(att_rt_use[YAWr]);
    robotwb.Rn_b_noyaw[1][0] = -cosd(-att_rt_use[ROLr])*sind(att_rt_use[YAWr]) + sind(-att_rt_use[PITr])*sind(-att_rt_use[ROLr])*cosd(att_rt_use[YAWr]);
    robotwb.Rn_b_noyaw[2][0] = sind(-att_rt_use[ROLr])*sind(att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*cosd(att_rt_use[YAWr]);

    robotwb.Rn_b_noyaw[0][1] = cosd(-att_rt_use[PITr])*sind(att_rt_use[YAWr]);
    robotwb.Rn_b_noyaw[1][1] = cosd(-att_rt_use[ROLr])*cosd(att_rt_use[YAWr]) + sind(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(att_rt_use[YAWr]);
    robotwb.Rn_b_noyaw[2][1] = -sind(-att_rt_use[ROLr])*cosd(att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(att_rt_use[YAWr]);

    robotwb.Rn_b_noyaw[0][2] = -sind(-att_rt_use[PITr]);
    robotwb.Rn_b_noyaw[1][2] = sind(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);
    robotwb.Rn_b_noyaw[2][2] = cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);

    mat_trans(robotwb.Rn_b_noyaw, robotwb.Rb_n_noyaw);

    att_rt_use[PITr] = 0;
    att_rt_use[ROLr] = 0;
    att_rt_use[YAWr] = robotwb.arm_att_h.z;
    robotwb.Rb_h_noatt_grip[0][0] = cosd(-att_rt_use[PITr])*cosd(att_rt_use[YAWr]);
    robotwb.Rb_h_noatt_grip[1][0] = -cosd(-att_rt_use[ROLr])*sind(att_rt_use[YAWr]) + sind(-att_rt_use[PITr])*sind(-att_rt_use[ROLr])*cosd(att_rt_use[YAWr]);
    robotwb.Rb_h_noatt_grip[2][0] = sind(-att_rt_use[ROLr])*sind(att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*cosd(att_rt_use[YAWr]);

    robotwb.Rb_h_noatt_grip[0][1] = cosd(-att_rt_use[PITr])*sind(att_rt_use[YAWr]);
    robotwb.Rb_h_noatt_grip[1][1] = cosd(-att_rt_use[ROLr])*cosd(att_rt_use[YAWr]) + sind(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(att_rt_use[YAWr]);
    robotwb.Rb_h_noatt_grip[2][1] = -sind(-att_rt_use[ROLr])*cosd(att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(att_rt_use[YAWr]);

    robotwb.Rb_h_noatt_grip[0][2] = -sind(-att_rt_use[PITr]);
    robotwb.Rb_h_noatt_grip[1][2] = sind(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);
    robotwb.Rb_h_noatt_grip[2][2] = cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);

    mat_trans(robotwb.Rb_h_noatt_grip, robotwb.Rh_b_noatt_grip);

    //convert acc
    Vect3 body_acc, world_acc;
#if RUN_WEBOTS&&1
    const double* acc_data = wb_accelerometer_get_values(ACC);
    vmc_all.acc[0] = acc_data[0];
    vmc_all.acc[1] = acc_data[1];
    vmc_all.acc[2] = acc_data[2];
    //printf("acc=%.3f %.3f %.3f\n", vmc_all.acc[0], vmc_all.acc[1], vmc_all.acc[2]);
    body_acc.x = vmc_all.acc_b.x = -vmc_all.acc[0];
    body_acc.y = vmc_all.acc_b.y = vmc_all.acc[1];
    body_acc.z = vmc_all.acc_b.z = vmc_all.acc[2];

    converV_b_to_nw(body_acc, &world_acc);

    vmc_all.acc_n.x = world_acc.x;
    vmc_all.acc_n.y = world_acc.y;
    vmc_all.acc_n.z = world_acc.z;
#else
    //convert acczZ
    body_acc.x=vmc_all.acc_b.x=spi_rx.acc_b[1]*9.81;
    body_acc.y=vmc_all.acc_b.y=spi_rx.acc_b[0]*9.81;
    body_acc.z=vmc_all.acc_b.z=spi_rx.acc_b[2]*9.81;

    converV_b_to_nw(body_acc,&world_acc);

    vmc_all.acc_n.x=world_acc.x;
    vmc_all.acc_n.y=world_acc.y;
    vmc_all.acc_n.z=world_acc.z-9.81;

    att_rt_use[PITr]=0;
    att_rt_use[ROLr]=0;
    att_rt_use[YAWr]=0;//-vmc_all.att[YAWr];

    vmc_all.acc_nn.x =  spi_rx.acc_n[0] * cosd(att_rt_use[YAWr]) -  (-spi_rx.acc_n[1]) * sind(att_rt_use[YAWr]);
    vmc_all.acc_nn.y =  spi_rx.acc_n[0] * sind(att_rt_use[YAWr]) +  (-spi_rx.acc_n[1]) * cosd(att_rt_use[YAWr]);
    vmc_all.acc_nn.z =  spi_rx.acc_n[2];
#endif
    // printf("body  acc=%f %f %f\n",vmc_all.acc_b.x,vmc_all.acc_b.y,vmc_all.acc_b.z);
    // printf("world acc=%f %f %f\n",vmc_all.acc_n.x,vmc_all.acc_n.y,vmc_all.acc_n.z);

    robotwb.now_att = robotwb.IMU_now;
    robotwb.now_rate = robotwb.IMU_dot;
}


void subscribe_webot_to_vmc(float dt)
{
  int i=0,j=0;
  for(i=0;i<4;i++)
  {
        leg_motor[i].t_to_i[0]=pos_force_p.t_to_i;
        leg_motor[i].t_to_i[2]=leg_motor[i].t_to_i[1]=leg_motor[i].t_to_i[0];

        robotwb.Leg[i].limit_tao[0]=pos_force_p.max_t_d[0];
        robotwb.Leg[i].limit_tao[1]=pos_force_p.max_t_d[1];
        robotwb.Leg[i].limit_tao[2]=pos_force_p.max_t_d[2];

        robotwb.Leg[i].f_pos_pid[Xr]=pos_force_p.f_pos_pid_st[Xr];
        robotwb.Leg[i].f_pos_pid[Yr]=pos_force_p.f_pos_pid_st[Yr];
        robotwb.Leg[i].f_pos_pid[Zr]=pos_force_p.f_pos_pid_st[Zr];

        // printf("robotwb.Leg[i].f_pos_pid[Zr].kp=%f\n",robotwb.Leg[i].f_pos_pid[Zr].kp);
        robotwb.Leg[i].f_pid[Xr]=pos_force_p.f_pid_st[Xr];
        robotwb.Leg[i].f_pid[Yr]=pos_force_p.f_pid_st[Yr];
        robotwb.Leg[i].f_pid[Zr]=pos_force_p.f_pid_st[Zr];

        //-------------------------joint PID
        if(robotwb.Leg[i].is_ground)
            robotwb.Leg[i].q_pid=pos_force_p.q_pid_st_stance;
        else
            robotwb.Leg[i].q_pid=pos_force_p.q_pid_sw;

        //param divde<<=====================New version===============================
   //Stand & init----------------------------------------------------------------------------------------

        robotwb.Leg[i].q_pid.kp_st=pos_force_p.q_pid_st_stance.kp;
        robotwb.Leg[i].q_pid.ki_st=pos_force_p.q_pid_st_stance.ki;
        robotwb.Leg[i].q_pid.kd_st=pos_force_p.q_pid_st_stance.kd;

        robotwb.Leg[i].q_pid.kp_sw=pos_force_p.q_pid_sw.kp;
        robotwb.Leg[i].q_pid.ki_sw=pos_force_p.q_pid_sw.ki;
        robotwb.Leg[i].q_pid.kd_sw=pos_force_p.q_pid_sw.kd;

        for(int j=0;j<3;j++){
            robotwb.Leg[i].q_pid.kp_st_d[j]=pos_force_p.q_pid_st_stance.kp_d[j];
            robotwb.Leg[i].q_pid.ki_st_d[j]=pos_force_p.q_pid_st_stance.ki_d[j];
            robotwb.Leg[i].q_pid.kd_st_d[j]=pos_force_p.q_pid_st_stance.kd_d[j];

            robotwb.Leg[i].q_pid.kp_sw_d[j]=pos_force_p.q_pid_sw.kp_d[j];
            robotwb.Leg[i].q_pid.ki_sw_d[j]=pos_force_p.q_pid_sw.ki_d[j];
            robotwb.Leg[i].q_pid.kd_sw_d[j]=pos_force_p.q_pid_sw.kd_d[j];
        }

        if(stand_force_enable_flag[4]&&robotwb.Leg[i].is_ground){
            robotwb.Leg[i].q_pid.param_sel=1;
        }
        else
        {
            robotwb.Leg[i].q_pid.param_sel=0;
        }

        leg_motor[i].max_t[0]=leg_motor[i].max_t[1]=leg_motor[i].max_t[2]=pos_force_p.max_t;
        leg_motor[i].max_i[0]=leg_motor[i].max_i[1]=leg_motor[i].max_i[2]=pos_force_p.max_i;
  }


  //-----------------output
  for(int i=0;i<4;i++){
      leg_motor[i].motor_en=arm_motor[0].motor_en;
      leg_motor[i].reset_q=arm_motor[0].reset_q;
  }
  //地形角期望命令
  if(vmc_all.param.leg_dof==3){
      DigitalLPF( vmc_all.ground_att_est[PITr], &vmc_all.ground_att_cmd[PITr] , 1, dt);
      DigitalLPF(-dead(vmc_all.ground_att_est[ROLr],5)*0, &vmc_all.ground_att_cmd[ROLr] , 1, dt);
  }else
     vmc_all.ground_att_cmd[PITr]=vmc_all.ground_att_est[PITr];
  //printf("%f %f\n",vmc_all.ground_att_cmd[PITr],vmc_all.ground_att_cmd[ROLr]);
  robotwb.exp_att.pitch = vmc_all.tar_att[PITr] + LIMIT(dead(vmc_all.ground_att_cmd[PITr],0) * (vmc_all.gait_mode > 0)*EN_ATT_GROUND_CONTROL, -35, 35);
  robotwb.exp_att.roll  = vmc_all.tar_att[ROLr] + LIMIT(dead(vmc_all.ground_att_cmd[ROLr],4)  * (vmc_all.gait_mode > 0)*EN_ATT_GROUND_CONTROL, -25, 25);

}
/* 读取所有电机转角 */
void readAllMotorPos(robotTypeDef* rob, float dt)
{
#if RUN_WEBOTS&&1
    rob->arm_base_height = wb_position_sensor_get_value(posensor_armt[0]);
    rob->arm_q[0][0] = wb_position_sensor_get_value(posensor_armt[1]);
    rob->arm_q[0][1] = wb_position_sensor_get_value(posensor_armt[2]);
    rob->arm_q[0][2] = wb_position_sensor_get_value(posensor_armt[3]);
    rob->arm_q[0][3] = wb_position_sensor_get_value(posensor_armt[4]);
    rob->arm_q[0][4] = wb_position_sensor_get_value(posensor_armt[5]);

    rob->wheel_q[0] = -wb_position_sensor_get_value(posensor_wt[0]);
    rob->wheel_q[1] = wb_position_sensor_get_value(posensor_wt[1]);
    rob->wheel_q[2] = -wb_position_sensor_get_value(posensor_wt[2]);
    rob->wheel_q[3] = wb_position_sensor_get_value(posensor_wt[3]);
    for (int i = 0; i < 6; i++)
        DigitalLPFw(LIMIT(To_180_degreesw(rob->arm_q[0][i] - rob->arm_q_reg[0][i]) / dt, -200, 200), &rob->arm_dq[0][i], 0, dt);

    for (int i = 0; i < 4; i++){
        DigitalLPFw(LIMIT(To_180_degreesw(rob->wheel_q[i] - rob->wheel_q_reg[i]) / dt, -2000, 2000), &rob->wheel_dq[i], 0, dt);
        rob->wheel_v[i]=rob->wheel_dq[i]*rob->wheel_r;
    }
#else
    //div for z height
#if 1
    rob->arm_base_height=(leg_motor[0].q_now[0]-leg_motor[0].q_bias[0])*rob->dq_2_dv_base;
    rob->arm_base_height=LIMIT(rob->arm_base_height,-1,2);//z height
    DigitalLPFw(LIMIT(To_180_degreesw(rob->arm_base_height- rob->q_reg_based_height) / dt, -0.8, 0.8),
            &rob->arm_base_dheight, 100, dt);
   // printf("div_z:z=%.3f dz=%.3f z_o=%.3f z_b=%.3f\n",rob->arm_base_height,rob->arm_base_dheight,leg_motor[0].q_now[0],leg_motor[0].q_bias[0]);
#else//rx q_now is rad of motor
    rob->arm_base_dheight=(leg_motor[0].q_now[0]-leg_motor[0].q_bias[0])*rob->dq_2_dv_base;
    rob->arm_base_height+=rob->arm_base_dheight*dt;
    rob->arm_base_height=LIMIT(rob->arm_base_height,0,0.6);//z height
#endif
    rob->arm_q[0][0] = leg_motor[0].q_now[1]/57.3;//conver to rad
    rob->arm_q[0][1] = leg_motor[0].q_now[2]/57.3;
    rob->arm_q[0][2] = spi_tx.arm_head3.att_set[0]/57.3;
    rob->arm_q[0][3] = spi_tx.arm_head3.att_set[1]/57.3;
    rob->arm_q[0][4] = spi_tx.arm_head3.att_set[2]/57.3;

    rob->arm_base_tau=leg_motor[0].t_now[0]*rob->dtau_t_basetau;
    rob->arm_t[0][0]= leg_motor[0].t_now[1];
    rob->arm_t[0][1]= leg_motor[0].t_now[2];
    rob->arm_t[0][2]=0;
    rob->arm_t[0][3]=0;
    rob->arm_t[0][4]=0;
    //printf("taufb=%f %f %f\n",leg_motor[0].t_now[1],leg_motor[0].t_now[2],leg_motor[0].t_now[3]);
    for (int i = 0; i < 5; i++)
        DigitalLPFw(LIMIT(To_180_degreesw(rob->arm_q[0][i] - rob->arm_q_reg[0][i]) / dt, -200, 200), &rob->arm_dq[0][i], 0, dt);//unused

    rob->wheel_dq[0] = spi_rx.w_dq_now[0];//rad from stm32
    rob->wheel_dq[1] = spi_rx.w_dq_now[1];
    rob->wheel_dq[2] = spi_rx.w_dq_now[2];
    rob->wheel_dq[3] = spi_rx.w_dq_now[3];

    for (int i = 0; i < 4; i++)
        rob->wheel_v[i]=rob->wheel_dq[i]*rob->wheel_r;

    rob->arm_base_height_reg=rob->arm_base_height;
    rob->q_reg_based_height=rob->arm_base_height;
#endif

#if 0
    printf("dw=%.2f %.2f %.2f %.2f\n",
        rob->wheel_dq[0] * 57.3, rob->wheel_dq[1] * 57.3,rob->wheel_dq[2] * 57.3,rob->wheel_dq[3] * 57.3);
    printf("vw=%.2f %.2f %.2f %.2f\n",
        rob->wheel_v[0] , rob->wheel_v[1] ,rob->wheel_v[2] ,rob->wheel_v[3] );
    //printf("q=%.2f %.2f %.2f | %.2f | %.2f\n",
     //   rob->arm_q[0][0] * 57.3, rob->arm_q[0][1] * 57.3, rob->arm_q[0][2] * 57.3, rob->arm_q[0][3] * 57.3, rob->arm_q[0][4] * 57.3);
    //printf("dq=%.2f %.2f %.2f | %.2f | %.2f\n",
    //	rob->arm_dq[0][0]*57.3, rob->arm_dq[0][1] * 57.3, rob->arm_dq[0][2] * 57.3, rob->arm_dq[0][3] * 57.3, rob->arm_dq[0][4] * 57.3);
#endif
    for(int i=0;i<6;i++)
        rob->arm_q_reg[0][i] = rob->arm_q[0][i];

    for(int i=0;i<4;i++)
        rob->wheel_q_reg[i] = rob->wheel_q[i];
}


void set_motor_head(void) {//input degree
    static float tar_sita[2] = { 0,0 };
    float flt=0.015;
    tar_sita[0] = flt*limitw(robotwb.head_att_exp[0], -89.9, 89.9)+(1-flt)* tar_sita[0];//航向
    tar_sita[1] = flt*limitw(robotwb.head_att_exp[1], -64, 35)    +(1-flt)* tar_sita[1];//俯仰
#if RUN_WEBOTS
    wb_motor_set_position(motor_headt[0], rad(tar_sita[0]));
    wb_motor_set_position(motor_headt[1], rad(tar_sita[1]));
#else
    spi_tx.arm_head3.head_set[1]=-tar_sita[0];//yaw
    spi_tx.arm_head3.head_set[0]=-tar_sita[1];//pitch
#endif
}

void set_motor_cap(void) {//input degree
    float tar_cap[2] = { 0.1,0.1 };
    if(robotwb.arm_control_mode==99){
        robotwb.cap_set=(robotwb.cap_set_sdk[0]);
    }
#if RUN_WEBOTS
    float flt=0.5;
    robotwb.cap_set_flt=flt*robotwb.cap_set+(1-flt)*robotwb.cap_set_flt;
    wb_motor_set_position(motor_capt[0], INFINITY);
    wb_motor_set_position(motor_capt[1], INFINITY);
    tar_cap[0]=0.034*(1-robotwb.cap_set_flt);
    tar_cap[1]=0.034*(1-robotwb.cap_set_flt);
    float cap_pos0 = wb_position_sensor_get_value(posensor_capt[0]);
    float exp_u0 = LIMIT((tar_cap[0] - cap_pos0) * 3.5,-0.05,0.05);
    float cap_pos1 = wb_position_sensor_get_value(posensor_capt[1]);
    float exp_u1 = LIMIT((tar_cap[1] - cap_pos1) * 3.5,-0.05,0.05);

    wb_motor_set_position(motor_capt[0], INFINITY);
    wb_motor_set_velocity(motor_capt[0], exp_u0);
    wb_motor_set_position(motor_capt[1], INFINITY);
    wb_motor_set_velocity(motor_capt[1], exp_u1);
#else
    spi_tx.arm_head3.cap=(1-robotwb.cap_set);
#endif
}

void set_motor_arm(void)//机械臂控制 input degree
{

    float tar_sita[5] = { 45,89,45,-89,45 };
    float out_flt[6]={0.73,0.73,0.7375,0.7375,0.7375,0.75};
    float out_flt_z=0.85;
    if(robotwb.arm_control_mode==99&&robotwb.state_gait>=2){
        robotwb.arm_base_height_exp=robotwb.arm_q_exp_sdk[0][0];
        robotwb.arm_q_exp[0][0]=robotwb.arm_q_exp_sdk[0][1];
        robotwb.arm_q_exp[0][1]=robotwb.arm_q_exp_sdk[0][2];
        robotwb.arm_q_exp[0][2]=robotwb.arm_q_exp_sdk[0][3];
        robotwb.arm_q_exp[0][3]=robotwb.arm_q_exp_sdk[0][4];
        robotwb.arm_q_exp[0][4]=robotwb.arm_q_exp_sdk[0][5];
        float rate_all=0.05;
        out_flt[0]=rate_all;
        out_flt[1]=rate_all;
        out_flt[2]=rate_all;
        out_flt[3]=rate_all;
        out_flt[4]=rate_all;
        out_flt[5]=rate_all;
    }

    //robotwb.arm_base_height_exp=0.15;
    robotwb.arm_base_height_exp_flt=out_flt_z*robotwb.arm_base_height_exp+(1-out_flt_z)*robotwb.arm_base_height_exp_flt;
    robotwb.arm_base_height_exp_flt=LIMIT(robotwb.arm_base_height_exp_flt,robotwb.min_q_z,robotwb.max_q_z);
#if RUN_WEBOTS
    float arm_base_height = wb_position_sensor_get_value(posensor_armt[0]);
    float exp_u = LIMIT((robotwb.arm_base_height_exp_flt - arm_base_height) * 10,-0.99,0.99);

    for(int i=0;i<6;i++){
        robotwb.arm_q_exp_flt[0][i]=out_flt[i]*robotwb.arm_q_exp[0][i]+(1-out_flt[i])*robotwb.arm_q_exp_flt[0][i];
    }
//    printf("arm:%.2f %.2f %.2f %.2f %.2f\n",
//            robotwb.arm_q_exp_flt[0][0],
//            robotwb.arm_q_exp_flt[0][1],
//            robotwb.arm_q_exp_flt[0][2],
//            robotwb.arm_q_exp_flt[0][3],
//            robotwb.arm_q_exp_flt[0][4]);
    wb_motor_set_position(motor_armt[0], INFINITY);
    wb_motor_set_velocity(motor_armt[0], exp_u);

    tar_sita[0] = limitw(robotwb.arm_q_exp_flt[0][0], -89.9, 89.9);//大臂
    tar_sita[1] = limitw(robotwb.arm_q_exp_flt[0][1], 0,179.9);//小臂
    tar_sita[2] = limitw(robotwb.arm_q_exp_flt[0][2], -100,100);//航向
    tar_sita[3] = limitw(robotwb.arm_q_exp_flt[0][3], -89.9,89.9);//俯仰
    tar_sita[4] = limitw(robotwb.arm_q_exp_flt[0][4], -90,90);//横滚
    wb_motor_set_position(motor_armt[1], rad(tar_sita[0]));
    wb_motor_set_position(motor_armt[2], rad(tar_sita[1]));
    wb_motor_set_position(motor_armt[3], rad(tar_sita[2]));
    wb_motor_set_position(motor_armt[4], rad(tar_sita[3]));
    wb_motor_set_position(motor_armt[5], rad(tar_sita[4]));
#else
    for(int i=0;i<6;i++){
        robotwb.arm_q_exp_flt[0][i]=out_flt[i]*robotwb.arm_q_exp[0][i]+(1-out_flt[i])*robotwb.arm_q_exp_flt[0][i];
    }

    tar_sita[0] = limitw(robotwb.arm_q_exp_flt[0][0], -89.9, 89.9);//大臂
    tar_sita[1] = limitw(robotwb.arm_q_exp_flt[0][1], 0,179.9);//小臂
    tar_sita[2] = limitw(robotwb.arm_q_exp_flt[0][2], -100,100);//航向
    tar_sita[3] = limitw(robotwb.arm_q_exp_flt[0][3], -89.9,89.9);//俯仰
    tar_sita[4] = limitw(robotwb.arm_q_exp_flt[0][4], -89.9,89.9);//横滚

    float arm_base_height = robotwb.arm_base_height;
    float exp_u = LIMIT((robotwb.arm_base_height_exp_flt - arm_base_height) * 10,-0.3,0.3);
    if(robotwb.reset_z_mode==1)//auto down
        exp_u=-0.2;
    //printf("cmd[%d]=%f %f %f\n", leg_motor[0].motor_en,tar_sita[0], tar_sita[1], tar_sita[2]);
    robotwb.Leg[0].tar_sita[0]=LIMIT(exp_u*robotwb.dv_2_dq_base,-300,300);//converto dq_exp
    //printf("z[%d]=exp=%.3f now=%.3f | exp_dq=%.3f out=%f| fb=%f\n", leg_motor[0].motor_en,robotwb.arm_base_height_exp_flt, arm_base_height, exp_u,robotwb.Leg[0].tar_sita[0],robotwb.arm_base_tau);
    robotwb.Leg[0].tar_sita[1]=tar_sita[0];//degree to stm32
    robotwb.Leg[0].tar_sita[2]=tar_sita[1];
    spi_tx.arm_head3.att_set[0]=tar_sita[2];
    spi_tx.arm_head3.att_set[1]=tar_sita[3];
    spi_tx.arm_head3.att_set[2]=tar_sita[4];
#endif
}

void set_motor_w(void)//轮毂转速控制 input degree
{
    float wheel_dq_exp[4] = {0,0,0,0};
//    printf("%f %f %f %f\n",robotwb.wheel_dq_exp[0],robotwb.wheel_dq_exp[1],robotwb.wheel_dq_exp[2],robotwb.wheel_dq_exp[3]);
    wheel_dq_exp[0] = limitw(robotwb.wheel_dq_exp[0], -2300, 2300);//左前
    wheel_dq_exp[1] = limitw(robotwb.wheel_dq_exp[1], -2300, 2300);//右前
    wheel_dq_exp[2] = limitw(robotwb.wheel_dq_exp[2], -2300, 2300);//左后
    wheel_dq_exp[3] = limitw(robotwb.wheel_dq_exp[3], -2300, 2300);//右后
#if RUN_WEBOTS
    wb_motor_set_position(motor_wt[0], INFINITY);
    wb_motor_set_position(motor_wt[1], INFINITY);
    wb_motor_set_position(motor_wt[2], INFINITY);
    wb_motor_set_position(motor_wt[3], INFINITY);
    wb_motor_set_velocity(motor_wt[0], rad(-wheel_dq_exp[0]));
    wb_motor_set_velocity(motor_wt[1], rad(wheel_dq_exp[1]));
    wb_motor_set_velocity(motor_wt[2], rad(-wheel_dq_exp[2]));
    wb_motor_set_velocity(motor_wt[3], rad(wheel_dq_exp[3]));
#else
    spi_tx.w_dq_exp[0]=wheel_dq_exp[0]/57.3;//rad/s to stm32
    spi_tx.w_dq_exp[1]=wheel_dq_exp[1]/57.3;
    spi_tx.w_dq_exp[2]=wheel_dq_exp[2]/57.3;
    spi_tx.w_dq_exp[3]=wheel_dq_exp[3]/57.3;
#endif
}

#if 1
void pos_control_pd(float dt)
{
    int id = 3;
    for (id = 0; id < 2; id++) {//测试角度坐标系
        if (vmc_all.side_flip) {
            robotwb.Leg[id].tar_sita[0] = -0;
            robotwb.Leg[id].tar_sita[1] = 180;

            robotwb.Leg[id].q_pid.kp = 0.05;
            robotwb.Leg[id].q_pid.kd = 0.001;
        }
        robotwb.Leg[id].tar_sita[0] = limitw(robotwb.Leg[id].tar_sita[0], -robotwb.Leg[id].limit_sita[0], 180 + robotwb.Leg[id].limit_sita[0]);
        robotwb.Leg[id].tar_sita[1] = limitw(robotwb.Leg[id].tar_sita[1], -robotwb.Leg[id].limit_sita[1], 180 + robotwb.Leg[id].limit_sita[1]);
        robotwb.Leg[id].taod[0] = (To_180_degreesw(robotwb.Leg[id].tar_sita[0] - robotwb.Leg[id].sita[0])*robotwb.Leg[id].q_pid.kp - robotwb.Leg[id].sita_d[0] * robotwb.Leg[id].q_pid.kd) * 1;
        robotwb.Leg[id].taod[1] = (To_180_degreesw(robotwb.Leg[id].tar_sita[1] - robotwb.Leg[id].sita[1])*robotwb.Leg[id].q_pid.kp - robotwb.Leg[id].sita_d[1] * robotwb.Leg[id].q_pid.kd) * 1;
        //printf("leg=%d s0=%f %f s1=%f %f ds=%f %f\n",id,robotwb.Leg[id].tar_sita[0],robotwb.Leg[id].sita[0],robotwb.Leg[id].tar_sita[1],robotwb.Leg[id].sita[1],robotwb.Leg[id].sita_d[0],robotwb.Leg[id].sita_d[1]);
        set_motor_t(id);
    }
}

#if 1
void set_motor_t(int id)//g
{
    robotwb.Leg[id].taod[0] = limitw(robotwb.Leg[id].taod[0], -robotwb.Leg[id].limit_tao[0], robotwb.Leg[id].limit_tao[0]);
    robotwb.Leg[id].taod[1] = limitw(robotwb.Leg[id].taod[1], -robotwb.Leg[id].limit_tao[1], robotwb.Leg[id].limit_tao[1]);
    robotwb.Leg[id].taod[2] = limitw(robotwb.Leg[id].taod[2], -robotwb.Leg[id].limit_tao[2], robotwb.Leg[id].limit_tao[2]);
    robotwb.Leg[id].taod[3] = limitw(robotwb.Leg[id].taod[3], -robotwb.Leg[id].limit_tao[3], robotwb.Leg[id].limit_tao[3]);
    robotwb.Leg[id].taod[4] = limitw(robotwb.Leg[id].taod[4], -robotwb.Leg[id].limit_tao[4], robotwb.Leg[id].limit_tao[4]);

    robotwb.Leg[id].taom_output[0] = robotwb.Leg[id].taod[0];
    robotwb.Leg[id].taom_output[1] = robotwb.Leg[id].taod[1];
    robotwb.Leg[id].taom_output[2] = robotwb.Leg[id].taod[2];
    robotwb.Leg[id].taom_output[3] = robotwb.Leg[id].taod[3];
    robotwb.Leg[id].taom_output[4] = robotwb.Leg[id].taod[4];
}

#endif
void set_motor_q(int id) {}
void set_motor_q_up(int id) {}
void set_motor_q_sel(int id) {}
#if RUN_WEBOTS
float rc_scale_all=TIME_STEP/10;
#define KEY_W 87
#define KEY_S 83
#define KEY_A 65
#define KEY_D 68
#define KEY_  32

#define KEY_UP 315
#define KEY_DOWN 317
#define KEY_LEFT 314
#define KEY_RIGHT 316
#define KEY_ZU 91
#define KEY_ZD 93

#define KEY_7 55
#define KEY_8 56
#define KEY_9 57
#define KEY_0 48
#define KEY_Jian 45
#define KEY_Jia 61
#define KEY_Back 3

#define KEY_J 74
#define KEY_K 75

#define KEY_O 79
#define KEY_P 80

#define KEY_H 72
#define KEY_N 78

#define KEY_WAY 96
#define KEY_1 49
#define KEY_2 50
#define KEY_3 51
#define KEY_4 52
#define KEY_5 53
int key = 0;
int updateKey(void) {
    static int cnt=0;
    key = wb_keyboard_get_key();
    if (key > 0) {
        //printf("Key board==%d[%d]\n", key,cnt++);
        return 1;
    }
    return 0;
}

void updateJoy(void)//user_xd_d button
{
    float axis_max = 32768.0;
    float FLT_RC = 1;
    float T = 0.005;
    float rate_yaw_w[2];
    int axis_dead = 200;
    int axis = wb_joystick_get_number_of_axes();
    int povs = wb_joystick_get_number_of_povs();
    static float timer_rc = 0;
    static int flag_rc = 0;
    if (wb_joystick_is_connected())
    {
        // printf("joy_is_connected Axis=%d %d\n",wb_joystick_get_number_of_axes(),wb_joystick_get_number_of_povs());
        for (int i = 0; i < axis; i++)
        {
            int axis_v = 0;// dead(wb_joystick_get_axis_value(i), axis_dead);
            float rc_temp = (float)axis_v / axis_max;
            //printf("[%d]axis = %f\n",i,rc_temp);
            if (i == 0)
                robotwb.ocu.rc_spd_w[Xrw] = -rc_temp;

            if (i == 1)
                robotwb.ocu.rc_spd_w[Yrw] = rc_temp;

            if (i == 2)
                robotwb.ocu.rc_att_w[PITrw] = -rc_temp;

            if (i == 3)float rc_scale_all=0.8;
                robotwb.ocu.rc_att_w[ROLrw] = rc_temp;

            if (i == 4)
                rate_yaw_w[0] = rc_temp;

            if (i == 5)
                rate_yaw_w[1] = rc_temp;
        }

        robotwb.ocu.rate_yaw_w = rate_yaw_w[0] - rate_yaw_w[1];
        for (int i = 0; i < povs; i++)
        {
            int povs_v = (wb_joystick_get_pov_value(i));
            // printf("[%d]pov = %f\n",i,povs_v);

        }
        int button = wb_joystick_get_pressed_button();
        //printf("button = %d\n",button);
        robotwb.ocu.key_y = 0;
        robotwb.ocu.key_x = 0;
        robotwb.ocu.key_b = 0;
        robotwb.ocu.key_a = 0;
        robotwb.ocu.key_st = 0;
        robotwb.ocu.key_back = 0;
        robotwb.ocu.key_ll = 0;
        robotwb.ocu.key_rr = 0;

        if (button == 11)
            robotwb.ocu.key_y = 1;
        if (button == 10)
            robotwb.ocu.key_x = 1;
        if (button == 9)
            robotwb.ocu.key_b = 1;
        if (button == 8)
            robotwb.ocu.key_a = 1;
        if (button == 4)
            robotwb.ocu.key_ll = 1;
        if (button == 5)
            robotwb.ocu.key_rr = 1;
        if (button == 0)
            robotwb.ocu.key_st = 1;
        if (button == 1)
            robotwb.ocu.key_back = 1;
    }
    else//key board
    {
         if(updateKey()){

             robotwb.base_rate_exp_rc=0;
             robotwb.base_vel_b_exp_rc.x=0;

             if(key==KEY_H){
                  robotwb.head_control_mode=HEAD_FREE;
                  robotwb.arm_control_mode=ARM_M_H;
                  robotwb.head_att_exp[0]=robotwb.head_att_exp[1]=0;
             }
             if(key==KEY_N){
                  robotwb.head_control_mode=HEAD_LOOK_B;
                  robotwb.arm_control_mode=ARM_M_E;
             }

             if(robotwb.arm_control_mode==ARM_M_H){
#if EN_ARM_H
                     robotwb.arm_vpos_h_exp.x=robotwb.arm_vpos_h_exp.y=0;
                     if(key==KEY_UP)
                          robotwb.arm_vpos_h_exp.x=0.15*rc_scale_all;
                     if(key==KEY_DOWN)
                          robotwb.arm_vpos_h_exp.x=-0.15*rc_scale_all;

                     if(key==KEY_LEFT)
                          robotwb.arm_vpos_h_exp.y=0.15*rc_scale_all;
                     if(key==KEY_RIGHT)
                          robotwb.arm_vpos_h_exp.y=-0.15*rc_scale_all;
                     robotwb.arm_vpos_h_exp.z=0;
                     conver_h_to_b_griper(robotwb.arm_vpos_h_exp,&robotwb.arm_vpos_b_exp);

                     robotwb.arm_epos_b_exp.x+=robotwb.arm_vpos_b_exp.x*0.005;
                     robotwb.arm_epos_b_exp.y+=robotwb.arm_vpos_b_exp.y*0.005;
#else
                     if(key==KEY_UP)
                          robotwb.arm_epos_b_exp.x+=0.15*0.005*rc_scale_all;
                     if(key==KEY_DOWN)
                          robotwb.arm_epos_b_exp.x-=0.15*0.005*rc_scale_all;

                     //robotwb.arm_epos_b_exp.x=LIMIT(robotwb.arm_epos_b_exp.x,0,0.25);

                     if(key==KEY_LEFT)
                          robotwb.arm_epos_b_exp.y+=0.15*0.005*rc_scale_all;
                     if(key==KEY_RIGHT)
                          robotwb.arm_epos_b_exp.y-=0.15*0.005*rc_scale_all;
#endif

                     if(key==KEY_ZU)
                          robotwb.arm_epos_b_exp.z-=0.15*0.005*rc_scale_all;
                     if(key==KEY_ZD)
                          robotwb.arm_epos_b_exp.z+=0.15*0.005*rc_scale_all;

                     if(key==KEY_){
                         robotwb.arm_epos_b_exp.x=0.3235;
                         robotwb.arm_epos_b_exp.y=-0.1;

                         robotwb.arm_att_b_exp.x=0;
                         robotwb.arm_att_b_exp.y=0;
                         robotwb.arm_att_b_exp.z=0;
                     }
                     robotwb.arm_epos_b_exp.z=LIMIT(robotwb.arm_epos_b_exp.z,-0.05,0.3);

                     if(key==KEY_9)
                          robotwb.arm_att_b_exp.y+=20*0.005*rc_scale_all;
                     if(key==KEY_0)
                          robotwb.arm_att_b_exp.y-=20*0.005*rc_scale_all;

                     if(key==KEY_Jian)
                          robotwb.arm_att_b_exp.z+=35*0.005*rc_scale_all;
                     if(key==KEY_Jia)
                          robotwb.arm_att_b_exp.z-=35*0.005*rc_scale_all;

                     if(key==KEY_Back){
                         robotwb.arm_att_b_exp.x=0;
                         robotwb.arm_att_b_exp.y=0;
                         robotwb.arm_att_b_exp.z=0;
                     }

                     if(key==KEY_8)
                         robotwb.arm_att_b_exp.x=-89.9;
                     if(key==KEY_7)
                         robotwb.arm_att_b_exp.y=30.9;
             }else if(robotwb.arm_control_mode==ARM_M_E){

                 if(key==KEY_UP)
                      robotwb.arm_epos_e_exp.x+=0.15*0.005*rc_scale_all;
                 if(key==KEY_DOWN)
                      robotwb.arm_epos_e_exp.x-=0.15*0.005*rc_scale_all;

                 //robotwb.arm_epos_b_exp.x=LIMIT(robotwb.arm_epos_b_exp.x,0,0.25);

                 if(key==KEY_LEFT)
                      robotwb.arm_epos_e_exp.y+=0.15*0.005*rc_scale_all;
                 if(key==KEY_RIGHT)
                      robotwb.arm_epos_e_exp.y-=0.15*0.005*rc_scale_all;

                 //robotwb.arm_epos_b_exp.z=LIMIT(robotwb.arm_epos_b_exp.z,0,0.25);

                 if(key==KEY_ZU)
                      robotwb.arm_epos_e_exp.z-=0.15*0.005*rc_scale_all;
                 if(key==KEY_ZD)
                      robotwb.arm_epos_e_exp.z+=0.15*0.005*rc_scale_all;

                 if(key==KEY_){
                     reset_robot_statement();

                     robotwb.e_yaw_off=robotwb.IMU_now.yaw;
                     robotwb.base_pos_n_off= robotwb.base_pos_n;

                     robotwb.arm_epos_e_exp.x=0.3235;
                     robotwb.arm_epos_e_exp.y=-0.1;
                     robotwb.arm_epos_e_exp.z=robotwb.arm_epos_n.z+robotwb.base_height;

                     robotwb.arm_att_n_exp.x=0;
                     robotwb.arm_att_n_exp.y=0;
                     robotwb.arm_att_n_exp.z=0;
                 }

                 if(key==KEY_Jian)
                      robotwb.arm_att_n_exp.z+=35*0.005*rc_scale_all;
                 if(key==KEY_Jia)
                      robotwb.arm_att_n_exp.z-=35*0.005*rc_scale_all;

                 if(key==KEY_Back){
                     robotwb.arm_att_n_exp.x=0;
                     robotwb.arm_att_n_exp.y=0;
                     robotwb.arm_att_n_exp.z=0;
                 }

                 if(key==KEY_8)
                     robotwb.arm_att_n_exp.x=-89.90;
                 if(key==KEY_7)
                     robotwb.arm_att_n_exp.y=30.9;
                 robotwb.arm_epos_e_exp.z=LIMIT(robotwb.arm_epos_e_exp.z,-0.05,0.3);
            }

             if(key==KEY_O)
                  robotwb.cap_set=1;
             if(key==KEY_P)
                  robotwb.cap_set=0.0;


             if(key==KEY_W){
                 robotwb.base_rate_exp_rc=0;
                 robotwb.base_vel_b_exp_rc.x=0.1*rc_scale_all;
             }
             else if(key==KEY_S){
                 robotwb.base_rate_exp_rc=0;
                 robotwb.base_vel_b_exp_rc.x=-0.1*rc_scale_all;
             }
             else if(key==KEY_A){
                 robotwb.base_vel_b_exp_rc.x=0;
                 robotwb.base_rate_exp_rc=0.3*rc_scale_all;
             }
             else if(key==KEY_D){
                 robotwb.base_vel_b_exp_rc.x=0;
                 robotwb.base_rate_exp_rc=-0.3*rc_scale_all;
             }


             if(key==KEY_WAY){
                 robotwb.wheel_control_mode=0;robotwb.good_waypoint=1;
                 robotwb.tar_waypoint=robotwb.way_point[0];
             }
             if(key==KEY_1){
                 robotwb.wheel_control_mode=1;robotwb.good_waypoint=1;
                 robotwb.tar_waypoint=robotwb.way_point[0];
             }
             if(key==KEY_2){
                 robotwb.wheel_control_mode=1;robotwb.good_waypoint=1;
                 robotwb.tar_waypoint=robotwb.way_point[1];
             }
             if(key==KEY_3){
                 robotwb.wheel_control_mode=1;robotwb.good_waypoint=1;
                 robotwb.tar_waypoint=robotwb.way_point[2];
             }
             if(key==KEY_4){
                 robotwb.wheel_control_mode=1;robotwb.good_waypoint=1;
                 robotwb.tar_waypoint=robotwb.way_point[3];
             }
             if(key==KEY_5){
                 robotwb.wheel_control_mode=1;robotwb.good_waypoint=1;
                 robotwb.tar_waypoint=robotwb.way_point[4];
             }
             static int reg_wheel=0;
             if(robotwb.wheel_control_mode==1&&reg_wheel==0){
                 printf("SDK::Waypoint Mode!\n");
             }
             else if(robotwb.wheel_control_mode==0&&reg_wheel==1){
                 printf("SDK::RC Mode!\n");
             }
             reg_wheel=robotwb.wheel_control_mode;

             if(key==KEY_J&&robotwb.start_record==0&&robotwb.record_mode==0){
                 printf("SDK::Ready to Record&Imitate---Please waiting for 8~10s!!!!!!!!!!!!\n");
                 robotwb.start_record=1;
             }
             else if(key==KEY_K&&robotwb.start_record==1){
                 printf("SDK::Stop Record&Imitate-------!!!!!!!!!!!!!\n");
                 robotwb.start_record=robotwb.record_mode=0;
             }
         }else{
             robotwb.base_vel_b_exp_rc.x=robotwb.base_rate_exp_rc=0;

         }

    }
}

#else
void updateJoy(void)//
{
    float FLT_RC=1.2;
    float T=0.005;
    float MAX_SPD_X=1.0;
    float MAX_SPD_RAD=0.8;
    static int request_mode_reg=0;
    static int key_st_reg=0,key_x_reg=0,key_a_reg=0,key_b_reg=0,aux_smart_reg=0,key_selet_reg=0;
    static int rc_interuped=0;
    static float rc_interuped_cnt=0;
    //printf("%d %d %d %d %d\n",ocu.mode,nav_tx.ocu_mode,spi_rx.ocu.mode,RC_REMOTE,ocu.key_st);
    if(ocu.mode==RC_REMOTE_UP&&0){//auto mode unused now
            if(key_st_reg==0&&ocu.key_st==1)//&&vmc_all.gait_mode!=IDLE)
            {
                if(sdk.sdk_mode==0){
                    sdk.sdk_mode=1;
                    printf("Control:: Enable SDK command!\n");
                }else{
                    sdk.sdk_mode=0;
                    printf("Control:: Disable SDK command!\n");
                }
            }else if(key_a_reg==0&&sdk.sdk_mode==1&&(ocu.key_a==1||ocu.key_b==1||ocu.key_y==1)){
                    sdk.sdk_mode=0;
                    printf("Control:: Disable SDK command!\n");
            }

    }else{//SBUS
       // printf("%d %d %d %d\n",spi_rx.ocu.sbus_aux[0],&spi_rx.ocu.sbus_aux[1],spi_rx.ocu.sbus_aux[2],spi_rx.ocu.sbus_aux[3]);
        if(aux_smart_reg!=2&&spi_rx.ocu.sbus_aux[3]==2)
        {
            if(sdk.sdk_mode==0){
                sdk.sdk_mode=1;
                printf("Control:: Enable SDK command!\n");
            }
        }else  if(aux_smart_reg==2&&spi_rx.ocu.sbus_aux[3]!=2)
        {
            if(sdk.sdk_mode==1){
                sdk.sdk_mode=0;
                printf("Control:: Disable SDK command!\n");
            }
        }
    }

    if(request_mode_reg!=nav_tx.request_gait){//control by ros
        if(nav_tx.request_gait==100){
            sdk.sdk_mode=1;
            printf("ROS API:: Enable SDK command!\n");
        }
        else if(nav_tx.request_gait==101){
            sdk.sdk_mode=0;
            printf("ROS API:: Disable SDK command!\n");
        }else if(nav_tx.request_gait==102){
            sdk.sdk_mode=0;
            ocu.key_ud=-1;
            printf("ROS API:: Force Safe command!\n");
        }
    }
    request_mode_reg=nav_tx.request_gait;

    if(sdk.sdk_mode&&!rc_interuped){
        FLT_RC=10;
        DigitalLPF(sdk.cmd_vx,&robotwb.base_vel_b_exp_rc.x,FLT_RC,T);
        DigitalLPF(sdk.cmd_vy,&robotwb.base_vel_b_exp_rc.y,FLT_RC,T);
        DigitalLPF(sdk.cmd_vyaw/2,&robotwb.base_rate_exp_rc,FLT_RC,T);
        DigitalLPF(-ocu.rc_att_w[PITr]*vmc_all.param.MAX_PIT/2,&vmc_all.tar_att[PITr],FLT_RC,T);
        DigitalLPF(ocu.rc_att_w[ROLr]*vmc_all.param.MAX_ROL/3, &vmc_all.tar_att[ROLr],0.068,T);
    }else{
        FLT_RC=0.35;
        if( spi_rx.ocu.sbus_conncect){
            DigitalLPF(spi_rx.ocu.sbus_ch[1]*MAX_SPD_X,&robotwb.base_vel_b_exp_rc.x,FLT_RC,T);
            DigitalLPF(ocu.rc_spd_w[Yr]*MAX_SPD_X/2,&robotwb.base_vel_b_exp_rc.y,FLT_RC,T);
            DigitalLPF(-spi_rx.ocu.sbus_ch[2]*MAX_SPD_RAD,&robotwb.base_rate_exp_rc,FLT_RC,T);
            DigitalLPF(-ocu.rc_att_w[PITr]*vmc_all.param.MAX_PIT/2,&vmc_all.tar_att[PITr],FLT_RC,T);
        }else{
            DigitalLPF(ocu.rc_spd_w[Xr]*MAX_SPD_X,&robotwb.base_vel_b_exp_rc.x,FLT_RC,T);
            DigitalLPF(ocu.rc_spd_w[Yr]*MAX_SPD_X/2,&robotwb.base_vel_b_exp_rc.y,FLT_RC,T);
            DigitalLPF(ocu.rate_yaw_w*MAX_SPD_RAD,&robotwb.base_rate_exp_rc,FLT_RC,T);
            DigitalLPF(-ocu.rc_att_w[PITr]*vmc_all.param.MAX_PIT/2,&vmc_all.tar_att[PITr],FLT_RC,T);
            DigitalLPF(ocu.rc_att_w[ROLr]*vmc_all.param.MAX_ROL/3, &vmc_all.tar_att[ROLr],0.068,T);
        }
        //printf("OCU: %f %f|%f\n",robotwb.base_vel_b_exp_rc.x,robotwb.base_rate_exp_rc,ocu.rc_spd_w[Xr]);
    }
#if DIR_ARM_POWER
    static float timer_rc_auto=0;
    static int flag_rc=0;
    robotwb.head_control_mode=HEAD_LOOK_B;
    //printf("timer_rc_auto=%f\n",timer_rc_auto);
    if(robotwb.state_gait>=2){
        timer_rc_auto+=0.005;
#if 0
        switch(flag_rc){
            case 0:
                robotwb.base_vel_b_exp_rc.x=0.3;
                ocu.rc_spd_w[0]=0.6;
                if(timer_rc_auto>5)
                {
                    timer_rc_auto=0;
                    flag_rc++;
                }
            break;
            case 1:
                robotwb.base_vel_b_exp_rc.x=0;
                 robotwb.arm_epos_b_exp.z=0.25;
                if(timer_rc_auto>3)
                {
                    timer_rc_auto=0;
                    flag_rc++;
                }
            break;
            case 2:
                robotwb.base_vel_b_exp_rc.x=-0.3;
                ocu.rc_spd_w[0]=-0.6;
                if(timer_rc_auto>5)
                {
                    timer_rc_auto=0;
                    flag_rc++;
                }
            break;
            case 3:
                robotwb.base_vel_b_exp_rc.x=0;
                 robotwb.arm_epos_b_exp.z=0.65;
                if(timer_rc_auto>5)
                {
                    timer_rc_auto=0;
                    flag_rc=0;
                }
            break;
        }
#else
        switch(flag_rc){
            case 0:
                robotwb.base_vel_b_exp_rc.x=0.3;
                ocu.rc_spd_w[0]=0.4;
                ocu.rc_att_w[0]=0.1;
                ocu.rc_att_w[1]=0.2;
                if(timer_rc_auto>5)
                {
                    timer_rc_auto=0;
                    flag_rc++;
                }
            break;
            case 1:
                robotwb.base_vel_b_exp_rc.x=0;
                 robotwb.arm_epos_b_exp.z=0.15;
                if(timer_rc_auto>3)
                {
                    timer_rc_auto=0;
                    flag_rc++;
                }
            break;
            case 2:
                robotwb.base_vel_b_exp_rc.x=-0.3;
                ocu.rc_att_w[0]=-0.1;
                ocu.rc_spd_w[0]=-0.4;
                ocu.rc_att_w[1]=-0.2;
                if(timer_rc_auto>5)
                {
                    timer_rc_auto=0;
                    flag_rc++;
                }
            break;
            case 3:
                robotwb.base_vel_b_exp_rc.x=0;
                 robotwb.arm_epos_b_exp.z=0.75;
                if(timer_rc_auto>5)
                {
                    timer_rc_auto=0;
                    flag_rc=0;
                }
            break;
        }
#endif
    }

#endif

#if 1//ARM
    if(robotwb.state_gait>=2){
        if(ocu.key_x&&key_x_reg==0){
             robotwb.head_control_mode=HEAD_FREE;
             //robotwb.head_control_mode=HEAD_LOOK_B;
             robotwb.arm_control_mode=ARM_M_H;
             robotwb.head_att_exp[0]=robotwb.head_att_exp[1]=0;
             reset_tar_pos(0,0);
             reset_robot_statement();
             robotwb.reset_arm=1;
        }
        if(ocu.key_b&&key_b_reg==0){
             robotwb.reset_arm=1;
             robotwb.head_control_mode=HEAD_LOOK_B;
             robotwb.arm_control_mode=ARM_M_E;
        }
        if(ocu.key_a&&key_a_reg==0){
            // robotwb.head_control_mode=HEAD_LOOK_B;
             robotwb.head_control_mode=HEAD_RECODE;
        }

        key_x_reg=ocu.key_x;
        key_b_reg=ocu.key_b;
        if(robotwb.arm_control_mode==ARM_M_H){

#if EN_ARM_H
            robotwb.arm_vpos_h_exp.x=robotwb.arm_vpos_h_exp.y=0;
            robotwb.arm_vpos_h_exp.x=ocu.rc_spd_w[0];
            robotwb.arm_vpos_h_exp.y=-ocu.rc_spd_w[1];
            robotwb.arm_vpos_h_exp.z=0;

            conver_h_to_b_griper(robotwb.arm_vpos_h_exp,&robotwb.arm_vpos_b_exp);
            //printf("%f %f\n",robotwb.arm_vpos_b_exp.x,robotwb.arm_vpos_b_exp.y);
            robotwb.arm_epos_b_exp.x+=dead(robotwb.arm_vpos_b_exp.x,0.001)*0.0005;
            robotwb.arm_epos_b_exp.y+=dead(robotwb.arm_vpos_b_exp.y,0.001)*0.0005;
            robotwb.arm_epos_b_exp.x=LIMIT(robotwb.arm_epos_b_exp.x,0.35,0.7);
            robotwb.arm_epos_b_exp.y=LIMIT(robotwb.arm_epos_b_exp.y,-0.15+-0.15,-0.15+0.35);
#else
            robotwb.arm_epos_b_exp.x+=ocu.rc_spd_w[0]*0.0005;
            robotwb.arm_epos_b_exp.x=LIMIT(robotwb.arm_epos_b_exp.x,0.35,0.7);
            robotwb.arm_epos_b_exp.y+=-ocu.rc_spd_w[1]*0.0005;
            robotwb.arm_epos_b_exp.y=LIMIT(robotwb.arm_epos_b_exp.y,-0.15+-0.15,-0.15+0.35);
#endif
                //printf("exp=%f %f\n",robotwb.arm_epos_b_exp.x,robotwb.arm_epos_b_exp.y);

                if(ocu.key_ll)
                     robotwb.arm_epos_b_exp.z+=0.065*0.005;
                if(ocu.key_rr)
                     robotwb.arm_epos_b_exp.z-=0.065*0.005;

                if(ocu.key_st){//rest
                    robotwb.reset_arm=1;
                    robotwb.arm_epos_b_exp.x=0.3235;
                    robotwb.arm_epos_b_exp.y=-0.1;

                    robotwb.arm_att_b_exp.x=0;
                    robotwb.arm_att_b_exp.y=0;
                    robotwb.arm_att_b_exp.z=0;
                }
                robotwb.arm_epos_b_exp.z=LIMIT(robotwb.arm_epos_b_exp.z,robotwb.min_epos_z,robotwb.max_epos_z);
                //printf(" robotwb.arm_epos_b_exp.z=%f\n", robotwb.arm_epos_b_exp.z);
                //pitch
                robotwb.arm_att_b_exp.y+=ocu.rc_att_w[0]*25*0.005;
                robotwb.arm_att_b_exp.y=LIMIT(robotwb.arm_att_b_exp.y,-80,80);
                //yaw
                robotwb.arm_att_b_exp.z+=-ocu.rc_att_w[1]*55*0.005;
                robotwb.arm_att_b_exp.z=LIMIT(robotwb.arm_att_b_exp.z,-45,45);

                robotwb.arm_att_b_exp.x+=ocu.rate_yaw_w*100*0.005;
                robotwb.arm_att_b_exp.x=LIMIT(robotwb.arm_att_b_exp.x,-90,90);

                if(ocu.key_back){
                    robotwb.reset_arm=1;
                    robotwb.arm_att_b_exp.x=0;
                    robotwb.arm_att_b_exp.y=0;
                    robotwb.arm_att_b_exp.z=0;
                }
        }else if(robotwb.arm_control_mode==ARM_M_E){
            robotwb.arm_epos_e_exp.x+=ocu.rc_spd_w[0]*0.0005;
            robotwb.arm_epos_e_exp.x=LIMIT(robotwb.arm_epos_e_exp.x,0.35,0.7);
            robotwb.arm_epos_e_exp.y+=-ocu.rc_spd_w[1]*0.0005;
            robotwb.arm_epos_e_exp.y=LIMIT(robotwb.arm_epos_e_exp.y,-0.15+-0.15,-0.15+0.35);

            if(ocu.key_ll)
                 robotwb.arm_epos_e_exp.z+=0.065*0.005;
            if(ocu.key_rr)
                 robotwb.arm_epos_e_exp.z-=0.065*0.005;

            if(ocu.key_st){//rest
                robotwb.reset_arm=1;
                reset_robot_statement();

                robotwb.e_yaw_off=robotwb.IMU_now.yaw;
                robotwb.base_pos_n_off= robotwb.base_pos_n;

                robotwb.arm_epos_e_exp.x=0.3235;
                robotwb.arm_epos_e_exp.y=-0.1;
                robotwb.arm_epos_e_exp.z=robotwb.arm_epos_n.z+robotwb.base_height;

                robotwb.arm_att_n_exp.x=0;
                robotwb.arm_att_n_exp.y=0;
                robotwb.arm_att_n_exp.z=0;
            }
            robotwb.arm_epos_e_exp.z=LIMIT(robotwb.arm_epos_e_exp.z,robotwb.min_epos_z+robotwb.base_height,robotwb.max_epos_z+robotwb.base_height);

            //pitch
            robotwb.arm_att_n_exp.y+=ocu.rc_att_w[0]*25*0.005;
            robotwb.arm_att_n_exp.y=LIMIT(robotwb.arm_att_n_exp.y,-80,80);
            //yaw
            robotwb.arm_att_n_exp.z+=-ocu.rc_att_w[1]*55*0.005;
            robotwb.arm_att_n_exp.z=LIMIT(robotwb.arm_att_n_exp.z,-45,45);

            robotwb.arm_att_n_exp.x+=ocu.rate_yaw_w*200*0.005;
            robotwb.arm_att_n_exp.x=LIMIT(robotwb.arm_att_n_exp.x,-90,90);

            if(ocu.key_back){
                robotwb.reset_arm=1;
                robotwb.arm_att_n_exp.x=0;
                robotwb.arm_att_n_exp.y=0;
                robotwb.arm_att_n_exp.z=0;
            }
       }


        if(ocu.key_lr==1&&robotwb.start_record==0&&robotwb.record_mode==0){
            robotwb.head_control_mode=HEAD_LOOK_B;//HEAD_RECODE;
            printf("SDK::Ready to Record&Imitate---Please waiting for 8~10s!!!!!!!!!!!!\n");
            robotwb.start_record=1;
        }
        else if(((ocu.key_lr==-1)&&robotwb.start_record==1)||ocu.key_ud==-1){
            printf("SDK::Stop Record&Imitate-------!!!!!!!!!!!!!\n");
            robotwb.start_record=robotwb.record_mode=0;
            robotwb.head_control_mode=HEAD_FREE;
            robotwb.head_att_exp[0]=robotwb.head_att_exp[1]=0;
        }

        static int key_up_reg=0;
        static int flag_cap=0;
        if(ocu.key_ud==1&key_up_reg==0){
            if(flag_cap)
             robotwb.cap_set=1;
            else
             robotwb.cap_set=0.1;

            flag_cap=!flag_cap;
        }
        key_up_reg=ocu.key_ud;
    }
#endif



    //printf("%f %f %f\n",sdk.cmd_vx,sdk.cmd_vy,sdk.cmd_vyaw);
    if((fabs(ocu.rc_spd_w[Xr])>0.1||fabs(ocu.rc_spd_w[Yr])>0.1||fabs(ocu.rate_yaw_w)>0.1)&&!rc_interuped&&sdk.sdk_mode)
        {rc_interuped=1;rc_interuped_cnt=0;printf("Control:: Disable SDK for RC interuped!\n");}
    if((fabs(ocu.rc_spd_w[Xr])<0.1&&fabs(ocu.rc_spd_w[Yr])<0.1&&fabs(ocu.rate_yaw_w)<0.1)&&rc_interuped==1)
        rc_interuped_cnt+=T;
    else
        rc_interuped_cnt=0;

    if(rc_interuped_cnt>2&&rc_interuped==1)
    {   rc_interuped=0;
        printf("Control:: Re-enable SDK command without RC!\n");
    }

    if(robotwb.base_vel_b_exp_rc.x>0.2)
    {spi_tx.led_side_enable[0]=13;spi_tx.led_side_enable[1]=13;}
    else if(robotwb.base_vel_b_exp_rc.x<-0.2)
    {spi_tx.led_side_enable[0]=23;spi_tx.led_side_enable[1]=23;}
    else if(robotwb.base_rate_exp_rc>0.025)
    {spi_tx.led_side_enable[0]=12;spi_tx.led_side_enable[1]=4;}
    else if(robotwb.base_rate_exp_rc<-0.025)
    {spi_tx.led_side_enable[0]=4;spi_tx.led_side_enable[1]=12;}
    else if(robotwb.base_vel_b_exp_rc.x>0.05)
    {spi_tx.led_side_enable[0]=13;spi_tx.led_side_enable[1]=13;}
    else if(robotwb.base_vel_b_exp_rc.x<-0.05)
    {spi_tx.led_side_enable[0]=23;spi_tx.led_side_enable[1]=23;}
    else
    {
       if(spi_rx.brain_connect_c){
        spi_tx.led_side_enable[0]=33;spi_tx.led_side_enable[1]=33;
       }else{
        spi_tx.led_side_enable[0]=31;spi_tx.led_side_enable[1]=31;
       }
    }


    key_a_reg=ocu.key_a;
    key_st_reg=ocu.key_st;
    key_selet_reg=ocu.key_back;
    aux_smart_reg=spi_rx.ocu.sbus_aux[3];
}
#endif

FILE *fp1, *fp2, *fp3,*fp4, *fp5;
#define LEN_RECORD 80//记录位置
float data_record1[LEN_RECORD] = {0}, data_record2[LEN_RECORD] = { 0 }, data_record3[LEN_RECORD] = { 0 }, data_record4[LEN_RECORD] = { 0 }, data_record5[LEN_RECORD] = { 0 };//webots
void recorder(float dt) {
    static int state = 0;
    int i = 0;
    //webots绘图  doghome--------------------
    static int cnt=0;
    if (cnt++ > 1||1) {
        cnt = 0;
        //share_memory_drawing();//to drawing controller
    }
#if EN_RECORDER
    switch (state)
    {
    case 0:
        fp1 = fopen(".\\Data\\file1.txt", "w");
        fp2 = fopen(".\\Data\\file2.txt", "w");
        fp3 = fopen(".\\Data\\file3.txt", "w");
        fp4 = fopen(".\\Data\\file4.txt", "w");
        fp5 = fopen(".\\Data\\file5.txt", "w");
        if (fp1 != 0) {
            printf("---------------------------------Record Ready---------------------------------\n");
            state++;
        }
        break;
    case 1:
        for (i = 0; i < LEN_RECORD; i++)
            fprintf(fp1, "%.6f ",//姿态数据
                data_record1[i]);
        fprintf(fp1, "\n");

        for (i = 0; i < LEN_RECORD; i++)
            fprintf(fp2, "%.6f ",//单腿力
                data_record2[i]);
        fprintf(fp2, "\n");

        for (i = 0; i < LEN_RECORD; i++)
            fprintf(fp3, "%.6f ",//单腿运动
                data_record3[i]);
        fprintf(fp3, "\n");

        for (i = 0; i < LEN_RECORD; i++)
            fprintf(fp4, "%.6f ",//单腿运动
                data_record4[i]);
        fprintf(fp4, "\n");

        for (i = 0; i < LEN_RECORD; i++)
            fprintf(fp5, "%.6f ",//单腿运动
                data_record5[i]);
        fprintf(fp5, "\n");
        break;
    case 2:
        fclose(fp1); fclose(fp2); fclose(fp3); fclose(fp4); fclose(fp5);
        printf("---------------------------------Record Finish--------------------------\n");
        break;
    }
#endif
}

void record_thread(char sel, float dt)
{
    int cnt = 0;
    switch (sel) {
    case 0:
        cnt = 0;
        data_record1[cnt++] = robotwb.exp_att.pitch;
        data_record1[cnt++] = robotwb.now_att.pitch;

        data_record1[cnt++] = robotwb.exp_att.roll;
        data_record1[cnt++] = robotwb.now_att.roll;

        data_record1[cnt++] = robotwb.exp_att.yaw;
        data_record1[cnt++] = robotwb.now_att.yaw;

        data_record1[cnt++] = robotwb.now_rate.pitch;
        data_record1[cnt++] = robotwb.now_rate.roll;
        data_record1[cnt++] = robotwb.now_rate.yaw;

        data_record1[cnt++] = robotwb.exp_pos_n.x;
        data_record1[cnt++] = vmc_all.pos_n.x;// robotwb.cog_pos_n.x;
        data_record1[cnt++] = vmc_all.spd_n.x; //robotwb.cog_spd_n.x;

        data_record1[cnt++] = robotwb.exp_pos_n.z;
        data_record1[cnt++] = vmc_all.pos_n.z;// robotwb./home/jiao/usb_cam/src/roboimitate-r-master.zipcog_pos_n.z;
        data_record1[cnt++] = vmc_all.spd_n.z;// robotwb.cog_spd_n.z;

        data_record1[cnt++] = vmc_all.ground_att_est[PITr];// robotwb.ground_att_cmd[PITrw];
        data_record1[cnt++] = vmc_all.ground_att_est[ROLr];//robotwb.ground_att_cmd[ROLrw];

        data_record1[cnt++] = robotwb.exp_spd_n.x;// -robotwb.exp_spd_b.x;
        data_record1[cnt++] = robotwb.exp_spd_n.z;//robotwb.exp_spd_b.z;
//-----------
        cnt = 0;
        data_record2[cnt++] = robotwb.Leg[0].tar_force_dis_n.y;
        data_record2[cnt++] = robotwb.Leg[0].force_est_n_output.y;
        data_record2[cnt++] = robotwb.Leg[0].tar_force_dis_n.z;
        data_record2[cnt++] = robotwb.Leg[0].force_est_n_output.z;
        data_record2[cnt++] = robotwb.Leg[1].tar_force_dis_n.y;
        data_record2[cnt++] = robotwb.Leg[1].force_est_n_output.y;
        data_record2[cnt++] = robotwb.Leg[1].tar_force_dis_n.z;
        data_record2[cnt++] = robotwb.Leg[1].force_est_n_output.z;

        data_record2[cnt++] = robotwb.Leg[0].is_ground;
        data_record2[cnt++] = robotwb.Leg[0].is_touch_est;
        data_record2[cnt++] = robotwb.Leg[1].is_ground;
        data_record2[cnt++] = robotwb.Leg[1].is_touch_est;

        data_record2[cnt++] = robotwb.Leg[0].trig_state;
        data_record2[cnt++] = robotwb.Leg[1].trig_state;

        data_record2[cnt++] = robotwb.Leg[0].tar_sita[0];
        data_record2[cnt++] = robotwb.Leg[0].tar_sita[1];
        data_record2[cnt++] = robotwb.Leg[0].sita[0];
        data_record2[cnt++] = robotwb.Leg[0].sita[1];
        data_record2[cnt++] = vmc[0].param.tar_epos_h.x;// robotwb.Leg[0].tar_epos_h.x;
        data_record2[cnt++] = vmc[0].param.tar_epos_h.z;// robotwb.Leg[0].tar_epos_h.z;
        data_record2[cnt++] = robotwb.Leg[0].epos_h.x;
        data_record2[cnt++] = robotwb.Leg[0].epos_h.z;
        //printf("%f\n", robotwb.Leg[0].tar_epos_h.z);
        data_record2[cnt++] = robotwb.Leg[1].tar_sita[0];
        data_record2[cnt++] = robotwb.Leg[1].tar_sita[1];
        data_record2[cnt++] = robotwb.Leg[1].sita[0];
        data_record2[cnt++] = robotwb.Leg[1].sita[1];
        data_record2[cnt++] = vmc[1].param.tar_epos_h.x;// robotwb.Leg[0].tar_epos_h.x;
        data_record2[cnt++] = vmc[1].param.tar_epos_h.z;// robotwb.Leg[0].tar_epos_h.z;
        data_record2[cnt++] = robotwb.Leg[1].epos_h.x;
        data_record2[cnt++] = robotwb.Leg[1].epos_h.z;


        cnt = 0;//临时记录
        float spd_gps = 0;// wb_gps_get_speed(GPS);jilu

        break;
    }
}
#endif
