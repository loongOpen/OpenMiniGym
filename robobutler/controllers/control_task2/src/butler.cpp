#include "locomotion_header.h"
#include "base_struct.h"
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/parse.h>
#include <iostream>
using namespace std;
YAML::Node config_robot=YAML::LoadFile("/home/odroid/robobutler/Param/param_robot.yaml");
YAML::Node config_gait=YAML::LoadFile("/home/odroid/robobutler/Param/param_gait.yaml");

void butler_param(void)
{
float pos_gain_scale_sw=1;
float pos_gain_scale_st=1;
int i=0;

  printf("Loading Butler Param\n");


  vmc_all.param.soft_weight=1;

  MAX_SPD_X= config_gait["vmc_param"]["max_spd_x"].as<float>();
  MAX_SPD_Y= config_gait["vmc_param"]["max_spd_y"].as<float>();
  MAX_SPD_RAD= config_gait["vmc_param"]["max_spd_rotate"].as<float>();
   //----------------------------底层伺服参数--------------------------------

  for(i=0;i<4;i++)
  {
        leg_motor[i].t_to_i[2]=leg_motor[i].t_to_i[1]=leg_motor[i].t_to_i[0]=pos_force_p.t_to_i=config_robot["kin_param"]["i_2_Nm_q"][i].as<float>();//I_2_Nm;//设置扭矩系数<<-----------------------------
        //printf("leg_motor[i].t_to_i[0]=%f\n",leg_motor[i].t_to_i[0]);
        //--------------------------------关节PD------------------------------------------
        robotwb.Leg[i].q_pid_sw.kp=pos_force_p.q_pid_sw.kp=config_gait["imp_param"]["sw_kp"].as<float>();//1.4*pos_gain_scale_sw;//pos gain  关节闭环
        robotwb.Leg[i].q_pid_sw.ki=pos_force_p.q_pid_sw.ki=config_gait["imp_param"]["sw_ki"].as<float>();//0.03*pos_gain_scale_sw;
        robotwb.Leg[i].q_pid_sw.kd=pos_force_p.q_pid_sw.kd=config_gait["imp_param"]["sw_kd"].as<float>();//0.035*pos_gain_scale_sw;
        robotwb.Leg[i].q_pid_sw.vff=pos_force_p.q_pid_sw.vff= config_gait["imp_param"]["sw_vff"].as<float>();//10;

        robotwb.Leg[i].q_pid_st.kp=pos_force_p.q_pid_st_stance.kp=config_gait["imp_param"]["st_kp"].as<float>();//1.4*pos_gain_scale_st;//       力控关节增益 Stand
        robotwb.Leg[i].q_pid_st.ki=pos_force_p.q_pid_st_stance.ki=config_gait["imp_param"]["st_ki"].as<float>();//0.03*pos_gain_scale_st;
        robotwb.Leg[i].q_pid_st.kd=pos_force_p.q_pid_st_stance.kd=config_gait["imp_param"]["st_kd"].as<float>();//0.035*pos_gain_scale_st;
        robotwb.Leg[i].q_pid_st.vff=pos_force_p.q_pid_st_stance.vff= config_gait["imp_param"]["st_vff"].as<float>();//10;

        robotwb.Leg[i].q_pid_st.kp=pos_force_p.q_pid_init.kp=config_gait["imp_param"]["int_kp"].as<float>();//1.4*pos_gain_scale_st;//       力控关节增益 Stand
        robotwb.Leg[i].q_pid_st.ki=pos_force_p.q_pid_init.ki=config_gait["imp_param"]["int_ki"].as<float>();//0.03*pos_gain_scale_st;
        robotwb.Leg[i].q_pid_st.kd=pos_force_p.q_pid_init.kd=config_gait["imp_param"]["int_kd"].as<float>();//0.035*pos_gain_scale_st;
        robotwb.Leg[i].q_pid_st.vff=pos_force_p.q_pid_init.vff= config_gait["imp_param"]["int_vff"].as<float>();//10;

        //bldc div0----------------------------------------------------------------------------
        char bldc_id=0;
        pos_force_p.q_pid_sw.kp_d[bldc_id]=config_gait["imp_param"]["sw_kp0"].as<float>();//1.4*pos_gain_scale_sw;//pos gain  关节闭环
        pos_force_p.q_pid_sw.ki_d[bldc_id]=config_gait["imp_param"]["sw_ki0"].as<float>();//0.03*pos_gain_scale_sw;
        pos_force_p.q_pid_sw.kd_d[bldc_id]=config_gait["imp_param"]["sw_kd0"].as<float>();//0.035*pos_gain_scale_sw;
        pos_force_p.q_pid_sw.vff_d[bldc_id]= config_gait["imp_param"]["sw_vff0"].as<float>();//10;

        pos_force_p.q_pid_st_stance.kp_d[bldc_id]=config_gait["imp_param"]["st_kp0"].as<float>();//1.4*pos_gain_scale_st;//       力控关节增益 Stand
        pos_force_p.q_pid_st_stance.ki_d[bldc_id]=config_gait["imp_param"]["st_ki0"].as<float>();//0.03*pos_gain_scale_st;
        pos_force_p.q_pid_st_stance.kd_d[bldc_id]=config_gait["imp_param"]["st_kd0"].as<float>();//0.035*pos_gain_scale_st;
        pos_force_p.q_pid_st_stance.vff_d[bldc_id]= config_gait["imp_param"]["st_vff0"].as<float>();//10;

        bldc_id=1;
        pos_force_p.q_pid_sw.kp_d[bldc_id]=config_gait["imp_param"]["sw_kp1"].as<float>();//1.4*pos_gain_scale_sw;//pos gain  关节闭环
        pos_force_p.q_pid_sw.ki_d[bldc_id]=config_gait["imp_param"]["sw_ki1"].as<float>();//0.03*pos_gain_scale_sw;
        pos_force_p.q_pid_sw.kd_d[bldc_id]=config_gait["imp_param"]["sw_kd1"].as<float>();//0.035*pos_gain_scale_sw;
        pos_force_p.q_pid_sw.vff_d[bldc_id]= config_gait["imp_param"]["sw_vff1"].as<float>();//10;

        pos_force_p.q_pid_st_stance.kp_d[bldc_id]=config_gait["imp_param"]["st_kp1"].as<float>();//1.4*pos_gain_scale_st;//       力控关节增益 Stand
        pos_force_p.q_pid_st_stance.ki_d[bldc_id]=config_gait["imp_param"]["st_ki1"].as<float>();//0.03*pos_gain_scale_st;
        pos_force_p.q_pid_st_stance.kd_d[bldc_id]=config_gait["imp_param"]["st_kd1"].as<float>();//0.035*pos_gain_scale_st;
        pos_force_p.q_pid_st_stance.vff_d[bldc_id]= config_gait["imp_param"]["st_vff1"].as<float>();//10;

        bldc_id=2;
        pos_force_p.q_pid_sw.kp_d[bldc_id]=config_gait["imp_param"]["sw_kp2"].as<float>();//1.4*pos_gain_scale_sw;//pos gain  关节闭环
        pos_force_p.q_pid_sw.ki_d[bldc_id]=config_gait["imp_param"]["sw_ki2"].as<float>();//0.03*pos_gain_scale_sw;
        pos_force_p.q_pid_sw.kd_d[bldc_id]=config_gait["imp_param"]["sw_kd2"].as<float>();//0.035*pos_gain_scale_sw;
        pos_force_p.q_pid_sw.vff_d[bldc_id]= config_gait["imp_param"]["sw_vff2"].as<float>();//10;

        pos_force_p.q_pid_st_stance.kp_d[bldc_id]=config_gait["imp_param"]["st_kp2"].as<float>();//1.4*pos_gain_scale_st;//       力控关节增益 Stand
        pos_force_p.q_pid_st_stance.ki_d[bldc_id]=config_gait["imp_param"]["st_ki2"].as<float>();//0.03*pos_gain_scale_st;
        pos_force_p.q_pid_st_stance.kd_d[bldc_id]=config_gait["imp_param"]["st_kd2"].as<float>();//0.035*pos_gain_scale_st;
        pos_force_p.q_pid_st_stance.vff_d[bldc_id]= config_gait["imp_param"]["st_vff2"].as<float>();//10;
        //
        pos_force_p.q_pid_st_trot  = pos_force_p.q_pid_st_stance;
        //--------------------------------力导纳------------------------------------------
        robotwb.Leg[i].f_pos_pid_st[Xr].kp=pos_force_p.f_pos_pid_st[Xr].kp=config_gait["imp_param"]["imp_x_kp"].as<float>();//0.11;//力导纳
        robotwb.Leg[i].f_pos_pid_st[Xr].kd=pos_force_p.f_pos_pid_st[Xr].kd=config_gait["imp_param"]["imp_x_kd"].as<float>();//

        robotwb.Leg[i].f_pos_pid_st[Yr].kp=pos_force_p.f_pos_pid_st[Yr].kp=config_gait["imp_param"]["imp_y_kp"].as<float>();//0.11;//力导纳
        robotwb.Leg[i].f_pos_pid_st[Yr].kd=pos_force_p.f_pos_pid_st[Yr].kd=config_gait["imp_param"]["imp_y_kd"].as<float>();//

        robotwb.Leg[i].f_pos_pid_st[Zr].kp=pos_force_p.f_pos_pid_st[Zr].kp=config_gait["imp_param"]["imp_z_kp"].as<float>();//0.12;
        robotwb.Leg[i].f_pos_pid_st[Zr].kd=pos_force_p.f_pos_pid_st[Zr].kd=config_gait["imp_param"]["imp_z_kd"].as<float>();
        //---------------------------------力反馈--------------------------------------------
        robotwb.Leg[i].f_pid_st[Xr].kp=pos_force_p.f_pid_st[Xr].kp=config_gait["imp_param"]["fb_x_kp"].as<float>();//force fb  力反馈
        robotwb.Leg[i].f_pid_st[Yr].kp=pos_force_p.f_pid_st[Yr].kp=config_gait["imp_param"]["fb_y_kp"].as<float>();//force fb  力反馈
        robotwb.Leg[i].f_pid_st[Zr].kp=pos_force_p.f_pid_st[Zr].kp=config_gait["imp_param"]["fb_z_kp"].as<float>();


        min_q0[i]=config_robot["kin_param"]["min_q0"][i].as<float>();
        min_q1[i]=config_robot["kin_param"]["min_q1"][i].as<float>();
        min_q2[i]=config_robot["kin_param"]["min_q2"][i].as<float>();
        max_q0[i]=config_robot["kin_param"]["max_q0"][i].as<float>();
        max_q1[i]=config_robot["kin_param"]["max_q1"][i].as<float>();
        max_q2[i]=config_robot["kin_param"]["max_q2"][i].as<float>();
  }

//    robotwb.max_torque.x=config_gait["vmc_param"]["max_vTx"].as<float>();//15;//ROL  全局姿态扭矩
//    robotwb.max_torque.y=config_gait["vmc_param"]["max_vTy"].as<float>();//15;//PIT
//    robotwb.max_torque.z=config_gait["vmc_param"]["max_vTz"].as<float>();//15;//YAW

//    robotwb.max_force.x=config_gait["vmc_param"]["max_vFx"].as<float>();//30;//N 全局最大力
//    robotwb.max_force.y=config_gait["vmc_param"]["max_vFy"].as<float>();//30;//N 全局最大力
//    robotwb.max_force.z=config_gait["vmc_param"]["max_vFz"].as<float>();//60;//N

//    robotwb.max_leg_force.x=config_gait["vmc_param"]["max_leg_Fx"].as<float>();//30;//N 全局最大力
//    robotwb.max_leg_force.y=config_gait["vmc_param"]["max_leg_Fy"].as<float>();//30;//N 全局最大力
//    robotwb.max_leg_force.z=config_gait["vmc_param"]["max_leg_Fz"].as<float>();//60;//N

//    robotwb.min_leg_force.x=config_gait["vmc_param"]["min_leg_Fx"].as<float>();//30;//N 全局最大力
//    robotwb.min_leg_force.y=config_gait["vmc_param"]["min_leg_Fx"].as<float>();//30;//N 全局最大力
//    robotwb.min_leg_force.z=config_gait["vmc_param"]["min_leg_Fx"].as<float>();//60;//N

//    robotwb.max_err_force.x=config_gait["vmc_param"]["max_leg_err_Fx"].as<float>();//10;
//    robotwb.max_err_force.y=config_gait["vmc_param"]["max_leg_err_Fy"].as<float>();//10;
//    robotwb.max_err_force.z=config_gait["vmc_param"]["max_leg_err_Fz"].as<float>();//10;//WS//太小导致站立都有问题！！！！ 《10不行
	
//    //着地判断阈值new
//    pos_force_p.touch_z_param_st.st_td=config_robot["touch_st_stand"]["st_td"].as<float>();//0.5;//N  站立
//    pos_force_p.touch_z_param_st.st_lf=config_robot["touch_st_stand"]["st_lf"].as<float>();//0.25;//N

//    pos_force_p.touch_z_param_trot_st.st_td=config_robot["touch_st_trot"]["st_td"].as<float>();//0.5;//N  TROT
//    pos_force_p.touch_z_param_trot_st.st_lf=config_robot["touch_st_trot"]["st_lf"].as<float>();//0.25;//N
	
//    pos_force_p.touch_z_param_sw.st_td=config_robot["touch_swing"]["st_td"].as<float>();//10.0;//N  SW
//    pos_force_p.touch_z_param_sw.st_lf=config_robot["touch_swing"]["st_lf"].as<float>();//0.4;//N
//    pos_force_p.touch_z_param_sw.trot_sw=config_robot["touch_swing"]["trot_sw"].as<float>();//0.4;//N

//    pos_force_p.touch_z_param_td.st_td=config_robot["touch_td"]["st_td"].as<float>();//1.8;//N  TD
//    pos_force_p.touch_z_param_td.st_lf=config_robot["touch_td"]["st_lf"].as<float>();//0.4;//N
	
//    pos_force_p.touch_z_param_pronk.st_td=6.6;//N  SW
//    pos_force_p.touch_z_param_pronk.st_lf=0.4;//N

//    pos_force_p.touch_z_param_climb.st_td=6.6;//N  SW
//    pos_force_p.touch_z_param_climb.st_lf=0.4;//N

//    pos_force_p.touch_z_param_bound.st_td=8.6;//N  SW
//    pos_force_p.touch_z_param_bound.st_lf=0.4;//N

//    pos_force_p.touch_z_param_bound.check_spd = 0.8;
//    pos_force_p.touch_z_param_st.check_spd = config_robot["touch_st_stand"]["check_spd"].as<float>();//0.6;
//    pos_force_p.touch_z_param_trot_st.check_spd =config_robot["touch_st_trot"]["check_spd"].as<float>();// 0.6;
//    pos_force_p.touch_z_param_pronk.check_spd = 1.2;
//    pos_force_p.touch_z_param_climb.check_spd = 0.6;
//    pos_force_p.touch_z_param_sw.check_spd =config_robot["touch_swing"]["check_spd"].as<float>();// 1.8;
//    pos_force_p.touch_z_param_td.check_spd =config_robot["touch_td"]["check_spd"].as<float>();// 1.2;

//    pos_force_p.touch_z_param_bound.check_td=2;
//    pos_force_p.touch_z_param_bound.check_lf=1;
//    pos_force_p.touch_z_param_climb.check_td=2;
//    pos_force_p.touch_z_param_climb.check_lf=1;
//    pos_force_p.touch_z_param_pronk.check_td=2;
//    pos_force_p.touch_z_param_pronk.check_lf=1;
//    pos_force_p.touch_z_param_trot_st.check_td=config_robot["touch_st_trot"]["check_td"].as<float>();//1;
//    pos_force_p.touch_z_param_trot_st.check_lf=config_robot["touch_st_trot"]["check_lf"].as<float>();//2;

//    pos_force_p.touch_z_param_st.check_td=config_robot["touch_st_stand"]["check_td"].as<float>();//2;
//    pos_force_p.touch_z_param_st.check_lf=config_robot["touch_st_stand"]["check_lf"].as<float>();//1;
//    pos_force_p.touch_z_param_sw.check_td=config_robot["touch_swing"]["check_td"].as<float>();//2;
//    pos_force_p.touch_z_param_sw.check_lf=config_robot["touch_swing"]["check_lf"].as<float>();//1;
//    pos_force_p.touch_z_param_td.check_td=config_robot["touch_td"]["check_td"].as<float>();//1;
//    pos_force_p.touch_z_param_td.check_lf=config_robot["touch_td"]["check_lf"].as<float>();//1;
//    //---------------------------------------------------------------------------------------------
//    vmc_all.param.param_vmc.move_att_off[PITr]=config_gait["vmc_param"]["move_off_pit"].as<float>();//
//    vmc_all.param.param_vmc.move_att_off[ROLr]=config_gait["vmc_param"]["move_off_rol"].as<float>();//

//    vmc_all.param.climb_off.x=config_gait["vmc_param"]["climb_off_x"].as<float>();//
//    vmc_all.param.climb_off.y=config_gait["vmc_param"]["climb_off_y"].as<float>();//
//    vmc_all.param.climb_off.z=config_gait["vmc_param"]["climb_off_z"].as<float>();//
	//最大扭矩矩和最大电流
    pos_force_p.max_i=config_robot["kin_param"]["max_q_I"].as<float>();
    pos_force_p.max_t=config_robot["kin_param"]["max_q_tau"].as<float>();
	
    pos_force_p.max_t_d[0]=config_robot["kin_param"]["max_q_tau_div"][0].as<float>();
    pos_force_p.max_t_d[1]=config_robot["kin_param"]["max_q_tau_div"][1].as<float>();
    pos_force_p.max_t_d[2]=config_robot["kin_param"]["max_q_tau_div"][2].as<float>();

	pos_force_p.en_force_control_cal=USE_FPOS_CONTROL;
	pos_force_p.en_force_control_out=USE_FPOS_CONTROL;

    robotwb.arm_epos_h_grasp_off.x=config_gait["grasp_param"]["pose_off_x"].as<float>();//0.017814;//0.0117;
    robotwb.arm_epos_h_grasp_off.y=config_gait["grasp_param"]["pose_off_y"].as<float>();//-0.026334;//-0.04;
    robotwb.arm_epos_h_grasp_off.z=config_gait["grasp_param"]["pose_off_z"].as<float>();//0;
    printf("arm_epos_h_grasp_off=%f %f %f\n",robotwb.arm_epos_h_grasp_off.x,robotwb.arm_epos_h_grasp_off.y,robotwb.arm_epos_h_grasp_off.z);
    robotwb.arm_epos_h_grasp_gpd_off.x=config_gait["grasp_param"]["pose_off_x_gpd"].as<float>();//0.017814;//0.0117;
    robotwb.arm_epos_h_grasp_gpd_off.y=config_gait["grasp_param"]["pose_off_y_gpd"].as<float>();//-0.026334;//-0.04;
    robotwb.arm_epos_h_grasp_gpd_off.z=config_gait["grasp_param"]["pose_off_z_gpd"].as<float>();//0;
}
