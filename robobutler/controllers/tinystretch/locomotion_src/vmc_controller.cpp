#include "include.h"
#include "gait_math.h"
#include "locomotion_header.h"
#if RUN_WEBOTS
#include <webots/Robot.hpp>
#include <webots/supervisor.h>
#include "wbInterface.h"
END_POS tar_waypoint;
#endif

void traj_planner(float dt)
{
#if RUN_WEBOTS
    static int init=0;
    if(!init&&1){
        WbNodeRef flag[20];
        WbFieldRef flag_t[20];
        flag[0] = wb_supervisor_node_get_from_def("WAY0");
        flag_t[0] = wb_supervisor_node_get_field(flag[0], "translation");
        const double *flag_translation;
        flag_translation = wb_supervisor_node_get_position(flag[0]);
        robotwb.way_point[0].x=flag_translation[0];
        robotwb.way_point[0].y=flag_translation[2];
        robotwb.way_point[0].z=90;//yaw

        flag[0] = wb_supervisor_node_get_from_def("WAY1");
        flag_t[0] = wb_supervisor_node_get_field(flag[0], "translation");
        flag_translation = wb_supervisor_node_get_position(flag[0]);
        robotwb.way_point[1].x=flag_translation[0];
        robotwb.way_point[1].y=flag_translation[2];
        robotwb.way_point[1].z=90;//yaw

        flag[0] = wb_supervisor_node_get_from_def("WAY2");
        flag_t[0] = wb_supervisor_node_get_field(flag[0], "translation");
        flag_translation = wb_supervisor_node_get_position(flag[0]);
        robotwb.way_point[2].x=flag_translation[0];
        robotwb.way_point[2].y=flag_translation[2];
        robotwb.way_point[2].z=-90;//yaw

        flag[0] = wb_supervisor_node_get_from_def("WAY3");
        flag_t[0] = wb_supervisor_node_get_field(flag[0], "translation");
        flag_translation = wb_supervisor_node_get_position(flag[0]);
        robotwb.way_point[3].x=flag_translation[0];
        robotwb.way_point[3].y=flag_translation[2];
        robotwb.way_point[3].z=0;//yaw

        flag[0] = wb_supervisor_node_get_from_def("WAY4");
        flag_t[0] = wb_supervisor_node_get_field(flag[0], "translation");
        flag_translation = wb_supervisor_node_get_position(flag[0]);
        robotwb.way_point[4].x=flag_translation[0];
        robotwb.way_point[4].y=flag_translation[2];
        robotwb.way_point[4].z=-45;//yaw
    }
    #if 0
    if(robotwb.robot_mode==STAND_RC){
        robotwb.base_vel_b_exp_rc.x=0.2*1;//m/s
        robotwb.base_rate_exp_rc=-0.655*1;//rad/s
    }
    #endif
    if(robotwb.arm_control_mode==99){
        robotwb.base_vel_b_exp_rc.x=robotwb.base_vel_sdk[0];
        robotwb.base_rate_exp_rc=robotwb.base_vel_sdk[2];
    }
    //way point control

    if(robotwb.wheel_control_mode==1&&robotwb.good_waypoint==1){
        tar_waypoint=robotwb.tar_waypoint;
        float tar_yaw=To_180_degreesw(-atan2f(tar_waypoint.y-_sim_webots.base_pos.y,tar_waypoint.x-_sim_webots.base_pos.x)*57.3-90);
        robotwb.base_rate_exp_rc=LIMIT(To_180_degreesw(tar_yaw-vmc_all.att[2])/57.3,-35,35)*0.5;

        float dis=sqrt(pow(tar_waypoint.x-_sim_webots.base_pos.x,2)+pow(tar_waypoint.y-_sim_webots.base_pos.y,2));
        float weight_fb=1-LIMIT(fabs(To_180_degreesw(tar_yaw-vmc_all.att[2]))/5,0,1);
        if(dis>0.035)
            robotwb.base_vel_b_exp_rc.x=LIMIT(dis,-0.3,0.3)*0.5*weight_fb;
        else
            robotwb.base_rate_exp_rc=LIMIT(To_180_degreesw(tar_waypoint.z-vmc_all.att[2])/57.3,-35,35)*0.5;
        //printf("base_pos: x=%.2f %.2f yaw=%.2f %.2f dis=%.2f weight_fb=%.2f\n",_sim_webots.base_pos.x,_sim_webots.base_pos.y,tar_yaw,vmc_all.att[2],dis,weight_fb);
        robotwb.base_vel_b_exp_rc.x=LIMIT(robotwb.base_vel_b_exp_rc.x,-0.35,0.35);
        robotwb.base_rate_exp_rc=LIMIT(robotwb.base_rate_exp_rc,-1,1);
        if(dis<0.035&&fabs(To_180_degreesw(robotwb.tar_waypoint.z-vmc_all.att[2]))<2){
            robotwb.good_waypoint=0;
            robotwb.reach_waypoint=1;
            printf("SDK::Reach Waypoint!\n");
        }
        else
            robotwb.reach_waypoint=0;
    }
#endif
    //base
    float w_yaw=LIMIT(dead(fabs(robotwb.base_rate_exp_rc)/0.9,0.2)*0.5,0,1);

    DigitalLPF(robotwb.base_vel_b_exp_rc.x*(1-w_yaw), &robotwb.base_vel_b_exp_rc_flt.x, 0.9, dt);
    DigitalLPF(robotwb.base_rate_exp_rc, &robotwb.base_rate_exp_rc_flt, 0.9, dt);

    //arm-planner-demo

#if 0//hip mode test
    if(robotwb.robot_mode==STAND_RC){
        robotwb.yaw_lock=1;
        robotwb.arm_epos_h_exp.x=0.3235;
        robotwb.arm_epos_h_exp.y=-0.15;
        robotwb.arm_epos_h_exp.z=0;

        robotwb.arm_att_h_exp.x=0;
        robotwb.arm_att_h_exp.y=15;
        robotwb.arm_att_h_exp.z=0;

        robotwb.head_att_exp[0]=0;
        robotwb.head_att_exp[1]=-15;
    }
#endif
#if 0//base mode test
    if(robotwb.robot_mode==STAND_RC){
        robotwb.yaw_lock=1;
        robotwb.arm_epos_b_exp.x=0.5;
        robotwb.arm_epos_b_exp.y=0;
        robotwb.arm_epos_b_exp.z=0;

        robotwb.arm_att_b_exp.x=0;
        robotwb.arm_att_b_exp.y=15;
        robotwb.arm_att_b_exp.z=0;

        converV_b_to_h(robotwb.arm_epos_b_exp.x,robotwb.arm_epos_b_exp.y,robotwb.arm_epos_b_exp.z,
                       &robotwb.arm_epos_h_exp.x,&robotwb.arm_epos_h_exp.y,&robotwb.arm_epos_h_exp.z);

        ik_hip_protect();

        robotwb.arm_att_h_exp= robotwb.arm_att_b_exp;

        robotwb.head_att_exp[0]=0;
        robotwb.head_att_exp[1]=-15;
    }
#endif

#if 0//global-b mode test
    if(robotwb.robot_mode==STAND_RC){
        robotwb.base_rate_exp_rc=0.1;//rad/s

        robotwb.arm_epos_n_exp.x=0.5;
        robotwb.arm_epos_n_exp.y=0;
        robotwb.arm_epos_n_exp.z=0;

        robotwb.arm_att_n_exp.x=15;
        robotwb.arm_att_n_exp.y=45;
        robotwb.arm_att_n_exp.z=0;

        robotwb.arm_att_b_exp= robotwb.arm_att_n_exp;
        robotwb.arm_att_b_exp.z=LIMIT(To_180_degreesw(robotwb.arm_att_n_exp.z-robotwb.IMU_now_e_off.yaw),-90,90);

        //printf("yb:%f %f %f\n",robotwb.arm_att_b_exp.x,robotwb.arm_att_b_exp.y,robotwb.arm_att_b_exp.z);
        converV_n_to_b(robotwb.arm_epos_n_exp.x,robotwb.arm_epos_n_exp.y,robotwb.arm_epos_n_exp.z,
                       &robotwb.arm_epos_b_exp.x,&robotwb.arm_epos_b_exp.y,&robotwb.arm_epos_b_exp.z);
        //printf("b:%f %f %f\n",robotwb.arm_epos_b_exp.x,robotwb.arm_epos_b_exp.y,robotwb.arm_epos_b_exp.z);
        converV_b_to_h(robotwb.arm_epos_b_exp.x,robotwb.arm_epos_b_exp.y,robotwb.arm_epos_b_exp.z,
                       &robotwb.arm_epos_h_exp.x,&robotwb.arm_epos_h_exp.y,&robotwb.arm_epos_h_exp.z);

        ik_hip_protect();

        robotwb.arm_att_h_exp= robotwb.arm_att_b_exp;

        robotwb.head_att_exp[0]=0;
        robotwb.head_att_exp[1]=-15;
    }
#endif


#if 0//global-n mode test
    if(robotwb.robot_mode==STAND_RC){
        static float timer_test=0;
        timer_test+=dt;
        if(timer_test>8.6){
            robotwb.base_vel_b_exp_rc.x=0;
            robotwb.base_rate_exp_rc=-0.2;//rad/s
        }else if(timer_test>5.6){
            robotwb.base_vel_b_exp_rc.x=-0.02;
            robotwb.base_rate_exp_rc=0;//rad/s
        }else{
            robotwb.base_vel_b_exp_rc.x=0.01;
            robotwb.base_rate_exp_rc=0.35;//rad/s
        }

        robotwb.arm_epos_e_exp.x=0.5;
        robotwb.arm_epos_e_exp.y=-0.1;
        robotwb.arm_epos_e_exp.z=0.1;

        robotwb.arm_att_n_exp.x=15;
        robotwb.arm_att_n_exp.y=45;
        robotwb.arm_att_n_exp.z=0;

        robotwb.arm_att_b_exp= robotwb.arm_att_n_exp;
        robotwb.arm_att_b_exp.z=LIMIT(To_180_degreesw(robotwb.arm_att_n_exp.z-robotwb.IMU_now_e_off.yaw),-90,90);

        converV_e_to_n(robotwb.arm_epos_e_exp.x,robotwb.arm_epos_e_exp.y,robotwb.arm_epos_e_exp.z,
                       &robotwb.arm_epos_n_exp.x,&robotwb.arm_epos_n_exp.y,&robotwb.arm_epos_n_exp.z);

        converV_n_to_b(robotwb.arm_epos_n_exp.x,robotwb.arm_epos_n_exp.y,robotwb.arm_epos_n_exp.z,
                       &robotwb.arm_epos_b_exp.x,&robotwb.arm_epos_b_exp.y,&robotwb.arm_epos_b_exp.z);
        //printf("b:%f %f %f\n",robotwb.arm_epos_b_exp.x,robotwb.arm_epos_b_exp.y,robotwb.arm_epos_b_exp.z);
        converV_b_to_h(robotwb.arm_epos_b_exp.x,robotwb.arm_epos_b_exp.y,robotwb.arm_epos_b_exp.z,
                       &robotwb.arm_epos_h_exp.x,&robotwb.arm_epos_h_exp.y,&robotwb.arm_epos_h_exp.z);

        ik_hip_protect();

        robotwb.arm_att_h_exp= robotwb.arm_att_b_exp;

        robotwb.head_att_exp[0]=0;
        robotwb.head_att_exp[1]=-15;
    }
#endif


#if 0//global-n mode test head focus-h
    if(robotwb.robot_mode==STAND_RC){
        static float timer_test=0;
        timer_test+=dt;
        if(timer_test>3.6){
            robotwb.base_vel_b_exp_rc.x=0;
            robotwb.base_rate_exp_rc=-0.4;//rad/s
        }else if(timer_test>1.6){
            robotwb.base_vel_b_exp_rc.x=-0.02;
            robotwb.base_rate_exp_rc=0;//rad/s
        }else{
            robotwb.base_vel_b_exp_rc.x=0.01;
            robotwb.base_rate_exp_rc=0.35;//rad/s
        }

        robotwb.arm_epos_e_exp.x=0.5;
        robotwb.arm_epos_e_exp.y=-0.1;
        robotwb.arm_epos_e_exp.z=0.1;

        robotwb.arm_att_n_exp.x=15;
        robotwb.arm_att_n_exp.y=-10;
        robotwb.arm_att_n_exp.z=0;

        robotwb.arm_att_b_exp= robotwb.arm_att_n_exp;
        robotwb.arm_att_b_exp.z=LIMIT(To_180_degreesw(robotwb.arm_att_n_exp.z-robotwb.IMU_now_e_off.yaw),-90,90);

        converV_e_to_n(robotwb.arm_epos_e_exp.x,robotwb.arm_epos_e_exp.y,robotwb.arm_epos_e_exp.z,
                       &robotwb.arm_epos_n_exp.x,&robotwb.arm_epos_n_exp.y,&robotwb.arm_epos_n_exp.z);

        converV_n_to_b(robotwb.arm_epos_n_exp.x,robotwb.arm_epos_n_exp.y,robotwb.arm_epos_n_exp.z,
                       &robotwb.arm_epos_b_exp.x,&robotwb.arm_epos_b_exp.y,&robotwb.arm_epos_b_exp.z);

        converV_b_to_h(robotwb.arm_epos_b_exp.x,robotwb.arm_epos_b_exp.y,robotwb.arm_epos_b_exp.z,
                       &robotwb.arm_epos_h_exp.x,&robotwb.arm_epos_h_exp.y,&robotwb.arm_epos_h_exp.z);

        ik_hip_protect();

        robotwb.arm_att_h_exp= robotwb.arm_att_b_exp;

        //--------------------head foucs-hip---------
        robotwb.head_look_pos_h=robotwb.arm_epos_h;//look at arm_epos
        END_POS err_vec;
        err_vec.x=robotwb.head_look_pos_h.x-robotwb.head_pos_h.x;
        err_vec.y=robotwb.head_look_pos_h.y-robotwb.head_pos_h.y;
        err_vec.z=robotwb.head_look_pos_h.z-robotwb.head_pos_h.z;
        END_POS look_att;
        look_att.y=atan2f(err_vec.z,err_vec.x)*57.3;
        look_att.z=-atan2f(err_vec.y,err_vec.x)*57.3;
        //printf("att:%f %f\n", look_att.y, look_att.z);
        robotwb.head_att_exp[0]=LIMIT(look_att.z,-90,90);//yaw
        robotwb.head_att_exp[1]=LIMIT(look_att.y,-60,25);//pit
    }
#endif


#if 0//global-n mode test head focus-n
    if(robotwb.robot_mode==STAND_RC){
        static float timer_test=0;
        timer_test+=dt;

        robotwb.base_vel_b_exp_rc.x=0.01;
        robotwb.base_rate_exp_rc=0;//rad/s

        robotwb.arm_epos_e_exp.x=0.5;
        robotwb.arm_epos_e_exp.y=-0.15;
        robotwb.arm_epos_e_exp.z=0.1;

        robotwb.arm_att_n_exp.x=15;
        robotwb.arm_att_n_exp.y=-10;
        robotwb.arm_att_n_exp.z=0;

        robotwb.arm_att_b_exp= robotwb.arm_att_n_exp;
        robotwb.arm_att_b_exp.z=LIMIT(To_180_degreesw(robotwb.arm_att_n_exp.z-robotwb.IMU_now_e_off.yaw),-90,90);

        converV_e_to_n(robotwb.arm_epos_e_exp.x,robotwb.arm_epos_e_exp.y,robotwb.arm_epos_e_exp.z,
                       &robotwb.arm_epos_n_exp.x,&robotwb.arm_epos_n_exp.y,&robotwb.arm_epos_n_exp.z);

        converV_n_to_b(robotwb.arm_epos_n_exp.x,robotwb.arm_epos_n_exp.y,robotwb.arm_epos_n_exp.z,
                       &robotwb.arm_epos_b_exp.x,&robotwb.arm_epos_b_exp.y,&robotwb.arm_epos_b_exp.z);

        converV_b_to_h(robotwb.arm_epos_b_exp.x,robotwb.arm_epos_b_exp.y,robotwb.arm_epos_b_exp.z,
                       &robotwb.arm_epos_h_exp.x,&robotwb.arm_epos_h_exp.y,&robotwb.arm_epos_h_exp.z);

        ik_hip_protect();

        robotwb.arm_att_h_exp= robotwb.arm_att_b_exp;

        //--------------------head foucs-hip---------
        robotwb.head_look_pos_n.x=0.5;
        robotwb.head_look_pos_n.y=0;
        robotwb.head_look_pos_n.z=0.15;

        converV_n_to_b(robotwb.head_look_pos_n.x,robotwb.head_look_pos_n.y,robotwb.head_look_pos_n.z,
                       &robotwb.head_look_pos_h.x,&robotwb.head_look_pos_h.y,&robotwb.head_look_pos_h.z);

        END_POS err_vec;
        err_vec.x=robotwb.head_look_pos_h.x-robotwb.head_pos_h.x;
        err_vec.y=robotwb.head_look_pos_h.y-robotwb.head_pos_h.y;
        err_vec.z=robotwb.head_look_pos_h.z-robotwb.head_pos_h.z;
        END_POS look_att;
        look_att.y=atan2f(err_vec.z,err_vec.x)*57.3;
        look_att.z=-atan2f(err_vec.y,err_vec.x)*57.3;
        //printf("att:%f %f\n", look_att.y, look_att.z);
        robotwb.head_att_exp[0]=LIMIT(look_att.z,-90,90);//yaw
        robotwb.head_att_exp[1]=LIMIT(look_att.y,-60,25);//pit
    }
#endif


#if 0//global-n mode test head focus-e
    if(robotwb.robot_mode==STAND_RC){
        static float timer_test=0;
        timer_test+=dt;

        robotwb.base_vel_b_exp_rc.x=0.01;
        robotwb.base_rate_exp_rc=0;//rad/s

        robotwb.arm_epos_e_exp.x=0.5;
        robotwb.arm_epos_e_exp.y=-0.15;
        robotwb.arm_epos_e_exp.z=0.1;

        robotwb.arm_att_n_exp.x=15;
        robotwb.arm_att_n_exp.y=-10;
        robotwb.arm_att_n_exp.z=0;

        robotwb.arm_att_b_exp= robotwb.arm_att_n_exp;
        robotwb.arm_att_b_exp.z=LIMIT(To_180_degreesw(robotwb.arm_att_n_exp.z-robotwb.IMU_now_e_off.yaw),-90,90);

        converV_e_to_n(robotwb.arm_epos_e_exp.x,robotwb.arm_epos_e_exp.y,robotwb.arm_epos_e_exp.z,
                       &robotwb.arm_epos_n_exp.x,&robotwb.arm_epos_n_exp.y,&robotwb.arm_epos_n_exp.z);

        converV_n_to_b(robotwb.arm_epos_n_exp.x,robotwb.arm_epos_n_exp.y,robotwb.arm_epos_n_exp.z,
                       &robotwb.arm_epos_b_exp.x,&robotwb.arm_epos_b_exp.y,&robotwb.arm_epos_b_exp.z);

        converV_b_to_h(robotwb.arm_epos_b_exp.x,robotwb.arm_epos_b_exp.y,robotwb.arm_epos_b_exp.z,
                       &robotwb.arm_epos_h_exp.x,&robotwb.arm_epos_h_exp.y,&robotwb.arm_epos_h_exp.z);

        ik_hip_protect();

        robotwb.arm_att_h_exp= robotwb.arm_att_b_exp;

        //--------------------head foucs-hip---------
        robotwb.head_look_pos_e.x=0.5;
        robotwb.head_look_pos_e.y=0;
        robotwb.head_look_pos_e.z=0.15;

        converV_e_to_n(robotwb.head_look_pos_e.x,robotwb.head_look_pos_e.y,robotwb.head_look_pos_e.z,
                       &robotwb.head_look_pos_n.x,&robotwb.head_look_pos_n.y,&robotwb.head_look_pos_n.z);

        converV_n_to_b(robotwb.head_look_pos_n.x,robotwb.head_look_pos_n.y,robotwb.head_look_pos_n.z,
                       &robotwb.head_look_pos_h.x,&robotwb.head_look_pos_h.y,&robotwb.head_look_pos_h.z);

        END_POS err_vec;
        err_vec.x=robotwb.head_look_pos_h.x-robotwb.head_pos_h.x;
        err_vec.y=robotwb.head_look_pos_h.y-robotwb.head_pos_h.y;
        err_vec.z=robotwb.head_look_pos_h.z-robotwb.head_pos_h.z;
        END_POS look_att;
        look_att.y=atan2f(err_vec.z,err_vec.x)*57.3;
        look_att.z=-atan2f(err_vec.y,err_vec.x)*57.3;
        //printf("att:%f %f\n", look_att.y, look_att.z);
        robotwb.head_att_exp[0]=LIMIT(look_att.z,-90,90);//yaw
        robotwb.head_att_exp[1]=LIMIT(look_att.y,-60,25);//pit
    }
#endif


#if 0//global-n mode test head focus-h ik cap test
    if(robotwb.robot_mode==STAND_RC){
        static float timer_test=0;
        robotwb.yaw_lock=1;
        timer_test+=dt;
        if(timer_test>4.0){
            robotwb.arm_epos_e_exp.x=0.55;
            robotwb.arm_epos_e_exp.y=-0.15;
            robotwb.arm_epos_e_exp.z=0.15;
            robotwb.arm_att_n_exp.x=0;
            robotwb.arm_att_n_exp.y=0;
            robotwb.arm_att_n_exp.z=0;
            robotwb.cap_set=1;
        }else if(timer_test>3.0){
            robotwb.arm_epos_e_exp.x=0.55;
            robotwb.arm_epos_e_exp.y=-0.15;
            robotwb.arm_epos_e_exp.z=0.15;
            robotwb.arm_att_n_exp.x=0;
            robotwb.arm_att_n_exp.y=0;
            robotwb.arm_att_n_exp.z=0;
            robotwb.cap_set=0.3;
        }else if(timer_test>2.0){
            robotwb.arm_epos_e_exp.x=0.35;
            robotwb.arm_epos_e_exp.y=-0.15;
            robotwb.arm_epos_e_exp.z=0.25;
            robotwb.arm_att_n_exp.x=0;
            robotwb.arm_att_n_exp.y=0;
            robotwb.arm_att_n_exp.z=0;
            robotwb.cap_set=0.3;
        }else if(timer_test>1.5){
            robotwb.arm_epos_e_exp.x=0.5;
            robotwb.arm_epos_e_exp.y=-0.1;
            robotwb.arm_epos_e_exp.z=0.15;
            robotwb.arm_att_n_exp.x=0;
            robotwb.arm_att_n_exp.y=45;
            robotwb.arm_att_n_exp.z=0;
            robotwb.cap_set=0.3;
        }else if(timer_test>1.0){
            robotwb.arm_epos_e_exp.x=0.5;
            robotwb.arm_epos_e_exp.y=-0.1;
            robotwb.arm_epos_e_exp.z=0.15;
            robotwb.cap_set=1;
            robotwb.arm_att_n_exp.x=0;
            robotwb.arm_att_n_exp.y=45;
            robotwb.arm_att_n_exp.z=0;

        }else{
            robotwb.arm_epos_e_exp.x=0.5;
            robotwb.arm_epos_e_exp.y=-0.1;
            robotwb.arm_epos_e_exp.z=0.2;
            robotwb.cap_set=1;
            robotwb.arm_att_n_exp.x=0;
            robotwb.arm_att_n_exp.y=25;
            robotwb.arm_att_n_exp.z=0;
        }

        robotwb.base_vel_b_exp_rc.x=0.0;
        robotwb.base_rate_exp_rc=0.0;//rad/s

        robotwb.arm_att_b_exp= robotwb.arm_att_n_exp;
        robotwb.arm_att_b_exp.z=LIMIT(To_180_degreesw(robotwb.arm_att_n_exp.z-robotwb.IMU_now_e_off.yaw),-90,90);

        converV_e_to_n(robotwb.arm_epos_e_exp.x,robotwb.arm_epos_e_exp.y,robotwb.arm_epos_e_exp.z,
                       &robotwb.arm_epos_n_exp.x,&robotwb.arm_epos_n_exp.y,&robotwb.arm_epos_n_exp.z);

        converV_n_to_b(robotwb.arm_epos_n_exp.x,robotwb.arm_epos_n_exp.y,robotwb.arm_epos_n_exp.z,
                       &robotwb.arm_epos_b_exp.x,&robotwb.arm_epos_b_exp.y,&robotwb.arm_epos_b_exp.z);

        converV_b_to_h(robotwb.arm_epos_b_exp.x,robotwb.arm_epos_b_exp.y,robotwb.arm_epos_b_exp.z,
                       &robotwb.arm_epos_h_exp.x,&robotwb.arm_epos_h_exp.y,&robotwb.arm_epos_h_exp.z);

        ik_hip_protect();

        robotwb.arm_att_h_exp= robotwb.arm_att_b_exp;

        //--------------------head foucs-hip---------
        robotwb.head_look_pos_h=robotwb.arm_epos_h;//look at arm_epos
        END_POS err_vec;
        err_vec.x=robotwb.head_look_pos_h.x-robotwb.head_pos_h.x;
        err_vec.y=robotwb.head_look_pos_h.y-robotwb.head_pos_h.y;
        err_vec.z=robotwb.head_look_pos_h.z-robotwb.head_pos_h.z;
        END_POS look_att;
        look_att.y=atan2f(err_vec.z,err_vec.x)*57.3;
        look_att.z=-atan2f(err_vec.y,err_vec.x)*57.3;
        //printf("att:%f %f\n", look_att.y, look_att.z);
        robotwb.head_att_exp[0]=LIMIT(look_att.z,-90,90);//yaw
        robotwb.head_att_exp[1]=LIMIT(look_att.y,-60,25);//pit
    }
#endif


#if 0//global-n multi-mode test head focus-h ik cap test
    if(robotwb.robot_mode==STAND_RC){
        static float timer_test=0;
        robotwb.yaw_lock=1;
        timer_test+=dt;
        if(timer_test>8.5){
            robotwb.arm_epos_n_exp.x=0.5;
            robotwb.arm_epos_n_exp.y=-0.0;
            robotwb.arm_epos_n_exp.z=0.25-robotwb.base_height;
            robotwb.arm_att_n_exp.x=0;
            robotwb.arm_att_n_exp.y=0;
            robotwb.arm_att_n_exp.z=0;
            robotwb.cap_set=1;
            robotwb.arm_control_mode=ARM_M_N;
            robotwb.base_vel_b_exp_rc.x=0.0;
            robotwb.base_rate_exp_rc=0.0;//rad/s
            robotwb.exp_att.yaw=0;
        }else if(timer_test>8.0){
            robotwb.arm_epos_n_exp.x=0.5;
            robotwb.arm_epos_n_exp.y=-0.0;
            robotwb.arm_epos_n_exp.z=0.25-robotwb.base_height;
            robotwb.arm_att_n_exp.x=0;
            robotwb.arm_att_n_exp.y=0;
            robotwb.arm_att_n_exp.z=0;
            robotwb.cap_set=0.3;
            robotwb.arm_control_mode=ARM_M_N;
            robotwb.base_vel_b_exp_rc.x=0.0;
            robotwb.base_rate_exp_rc=0.0;//rad/s
            robotwb.exp_att.yaw=0;
        }else if(timer_test>5.0){
            robotwb.arm_epos_n_exp.x=0.5;
            robotwb.arm_epos_n_exp.y=-0.0;
            robotwb.arm_epos_n_exp.z=0.25-robotwb.base_height;
            robotwb.arm_att_n_exp.x=0;
            robotwb.arm_att_n_exp.y=0;
            robotwb.arm_att_n_exp.z=0;
            robotwb.cap_set=0.3;
            robotwb.arm_control_mode=ARM_M_N;
            robotwb.base_vel_b_exp_rc.x=0.2;
            robotwb.base_rate_exp_rc=0.0;//rad/s
        }else if(timer_test>4.0){
            robotwb.arm_epos_n_exp.x=0.5;
            robotwb.arm_epos_n_exp.y=-0.0;
            robotwb.arm_epos_n_exp.z=0.25-robotwb.base_height;
            robotwb.arm_att_n_exp.x=0;
            robotwb.arm_att_n_exp.y=0;
            robotwb.arm_att_n_exp.z=0;
            robotwb.cap_set=0.3;
            robotwb.arm_control_mode=ARM_M_N;
        }else if(timer_test>3.0){
            robotwb.arm_epos_e_exp.x=0.35;
            robotwb.arm_epos_e_exp.y=-0.15;
            robotwb.arm_epos_e_exp.z=0.25;
            robotwb.arm_att_n_exp.x=0;
            robotwb.arm_att_n_exp.y=0;
            robotwb.arm_att_n_exp.z=0;
            robotwb.cap_set=0.3;
            robotwb.arm_control_mode=ARM_M_E;//N
            robotwb.exp_att.yaw=90;
        }else if(timer_test>2.0){
            robotwb.arm_epos_e_exp.x=0.35;
            robotwb.arm_epos_e_exp.y=-0.15;
            robotwb.arm_epos_e_exp.z=0.25;
            robotwb.arm_att_n_exp.x=0;
            robotwb.arm_att_n_exp.y=0;
            robotwb.arm_att_n_exp.z=0;
            robotwb.cap_set=0.3;
            robotwb.arm_control_mode=ARM_M_E;
        }else if(timer_test>1.5){
            robotwb.arm_epos_e_exp.x=0.5;
            robotwb.arm_epos_e_exp.y=-0.1;
            robotwb.arm_epos_e_exp.z=0.15;
            robotwb.arm_att_n_exp.x=0;
            robotwb.arm_att_n_exp.y=45;
            robotwb.arm_att_n_exp.z=0;
            robotwb.cap_set=0.3;
            robotwb.arm_control_mode=ARM_M_E;
        }else if(timer_test>1.0){
            robotwb.arm_epos_e_exp.x=0.5;
            robotwb.arm_epos_e_exp.y=-0.1;
            robotwb.arm_epos_e_exp.z=0.15;
            robotwb.cap_set=1;
            robotwb.arm_att_n_exp.x=0;
            robotwb.arm_att_n_exp.y=45;
            robotwb.arm_att_n_exp.z=0;
            robotwb.arm_control_mode=ARM_M_E;
        }else{
            robotwb.arm_epos_e_exp.x=0.5;
            robotwb.arm_epos_e_exp.y=-0.1;
            robotwb.arm_epos_e_exp.z=0.2;
            robotwb.cap_set=1;
            robotwb.arm_att_n_exp.x=0;
            robotwb.arm_att_n_exp.y=25;
            robotwb.arm_att_n_exp.z=0;
            robotwb.arm_control_mode=ARM_M_E;
            robotwb.base_vel_b_exp_rc.x=0.0;
            robotwb.base_rate_exp_rc=0.0;//rad/s
        }



        robotwb.arm_att_b_exp= robotwb.arm_att_n_exp;
        robotwb.arm_att_b_exp.z=LIMIT(To_180_degreesw(robotwb.arm_att_n_exp.z-robotwb.IMU_now_e_off.yaw),-90,90);
        if(robotwb.arm_control_mode==ARM_M_E)
        converV_e_to_n(robotwb.arm_epos_e_exp.x,robotwb.arm_epos_e_exp.y,robotwb.arm_epos_e_exp.z,
                       &robotwb.arm_epos_n_exp.x,&robotwb.arm_epos_n_exp.y,&robotwb.arm_epos_n_exp.z);
        if(robotwb.arm_control_mode==ARM_M_N||robotwb.arm_control_mode==ARM_M_E)
        converV_n_to_b(robotwb.arm_epos_n_exp.x,robotwb.arm_epos_n_exp.y,robotwb.arm_epos_n_exp.z,
                       &robotwb.arm_epos_b_exp.x,&robotwb.arm_epos_b_exp.y,&robotwb.arm_epos_b_exp.z);

        converV_b_to_h(robotwb.arm_epos_b_exp.x,robotwb.arm_epos_b_exp.y,robotwb.arm_epos_b_exp.z,
                       &robotwb.arm_epos_h_exp.x,&robotwb.arm_epos_h_exp.y,&robotwb.arm_epos_h_exp.z);



        ik_hip_protect();

        robotwb.arm_att_h_exp= robotwb.arm_att_b_exp;

        //--------------------head foucs-hip---------
        robotwb.head_look_pos_h=robotwb.arm_epos_h;//look at arm_epos
        END_POS err_vec;
        err_vec.x=robotwb.head_look_pos_h.x-robotwb.head_pos_h.x;
        err_vec.y=robotwb.head_look_pos_h.y-robotwb.head_pos_h.y;
        err_vec.z=robotwb.head_look_pos_h.z-robotwb.head_pos_h.z;
        END_POS look_att;
        look_att.y=atan2f(err_vec.z,err_vec.x)*57.3;
        look_att.z=-atan2f(err_vec.y,err_vec.x)*57.3;
        //printf("att:%f %f\n", look_att.y, look_att.z);
        robotwb.head_att_exp[0]=LIMIT(look_att.z,-89,89);//yaw
        robotwb.head_att_exp[1]=LIMIT(look_att.y,-45,25);//pit
    }
#endif

#if 1//
        static int arm_lsm=0;
        static int arm_control_mode_reg=0;
        switch(arm_lsm){
        case 0:
            if(robotwb.arm_control_mode==ARM_M_E&&robotwb.arm_control_mode!=arm_control_mode_reg){
                robotwb.e_yaw_off=robotwb.IMU_now.yaw;
                robotwb.base_pos_n_off= robotwb.base_pos_n;
                reset_robot_statement();
                //arm_lsm=1;
                printf("Switch to ARM-E mode!\n");
                reset_tar_pos(0,0);

                robotwb.arm_epos_e_exp.x=robotwb.arm_epos_n.x;//+0.3235;
                robotwb.arm_epos_e_exp.y=robotwb.arm_epos_n.y;//-0.1;
                robotwb.arm_epos_e_exp.z=robotwb.arm_epos_n.z+robotwb.base_height;
                robotwb.arm_att_n_exp.x=0;
                robotwb.arm_att_n_exp.y=0;
                robotwb.arm_att_n_exp.z=0;
            }
            else if(robotwb.arm_control_mode==ARM_M_N&&robotwb.arm_control_mode!=arm_control_mode_reg){
                robotwb.e_yaw_off=robotwb.IMU_now.yaw;
                printf("Switch to ARM-N mode!\n");
                //arm_lsm=2;
                reset_tar_pos(0,0);

                robotwb.arm_epos_n_exp.x=0.3235;
                robotwb.arm_epos_n_exp.y=-0.1;
                robotwb.arm_epos_n_exp.z=0.25-robotwb.base_height;
                robotwb.arm_att_n_exp.x=0;
                robotwb.arm_att_n_exp.y=0;
                robotwb.arm_att_n_exp.z=0;

                robotwb.arm_att_b_exp.x=0;
                robotwb.arm_att_b_exp.y=0;
                robotwb.arm_att_b_exp.z=0;

                robotwb.arm_att_h_exp_flt.x=robotwb.arm_att_h_exp.x=0;
                robotwb.arm_att_h_exp_flt.y=robotwb.arm_att_h_exp.y=0;
                robotwb.arm_att_h_exp_flt.z=robotwb.arm_att_h_exp.z=0;
            }
            else if(robotwb.arm_control_mode==ARM_M_H&&robotwb.arm_control_mode!=arm_control_mode_reg){
                robotwb.e_yaw_off=robotwb.IMU_now.yaw;
                printf("Switch to ARM-H mode!\n");
                //arm_lsm=3;
                reset_tar_pos(0,0);

                robotwb.arm_epos_b_exp.x=robotwb.arm_epos_b.x;//0.3235;
                robotwb.arm_epos_b_exp.y=robotwb.arm_epos_b.y;//-0.1;
                robotwb.arm_epos_b_exp.z=robotwb.arm_epos_b.z;//robotwb.arm_epos_h.z;

                robotwb.arm_att_b_exp.x=0;
                robotwb.arm_att_b_exp.y=0;
                robotwb.arm_att_b_exp.z=0;

                robotwb.arm_att_h_exp_flt.x=robotwb.arm_att_h_exp.x=0;
                robotwb.arm_att_h_exp_flt.y=robotwb.arm_att_h_exp.y=0;
                robotwb.arm_att_h_exp_flt.z=robotwb.arm_att_h_exp.z=0;
            }
        break;
        }
        arm_control_mode_reg=robotwb.arm_control_mode;

        if(robotwb.arm_control_mode==ARM_M_E){
            converV_e_to_n(robotwb.arm_epos_e_exp.x-robotwb.base_pos_n_off.x,
                           robotwb.arm_epos_e_exp.y-robotwb.base_pos_n_off.y,
                           robotwb.arm_epos_e_exp.z,
                   &robotwb.arm_epos_n_exp.x,&robotwb.arm_epos_n_exp.y,&robotwb.arm_epos_n_exp.z);
            //printf("%f %f %f\n",robotwb.arm_epos_n_exp.x,robotwb.arm_epos_n_exp.y,robotwb.arm_epos_n_exp.z);
            robotwb.arm_att_b_exp= robotwb.arm_att_n_exp;
            robotwb.arm_att_b_exp.z=LIMIT(To_180_degreesw(robotwb.arm_att_n_exp.z-robotwb.IMU_now_e_off.yaw),-90,90);
        }
        if(robotwb.arm_control_mode==ARM_M_N||robotwb.arm_control_mode==ARM_M_E){
            converV_n_to_b(robotwb.arm_epos_n_exp.x,robotwb.arm_epos_n_exp.y,robotwb.arm_epos_n_exp.z,
                   &robotwb.arm_epos_b_exp.x,&robotwb.arm_epos_b_exp.y,&robotwb.arm_epos_b_exp.z);
            ik_hip_protect(&robotwb.arm_epos_b_exp);
            robotwb.arm_att_b_exp= robotwb.arm_att_n_exp;
            robotwb.arm_att_b_exp.z=LIMIT(To_180_degreesw(robotwb.arm_att_n_exp.z-robotwb.IMU_now_e_off.yaw),-90,90);
        }

        converV_b_to_h(robotwb.arm_epos_b_exp.x,robotwb.arm_epos_b_exp.y,robotwb.arm_epos_b_exp.z,
               &robotwb.arm_epos_h_exp.x,&robotwb.arm_epos_h_exp.y,&robotwb.arm_epos_h_exp.z);
        ik_hip_protect(&robotwb.arm_epos_h_exp);

        robotwb.arm_att_h_exp= robotwb.arm_att_b_exp;

#if DIR_HEAD_RECORD
        robotwb.head_control_mode=HEAD_RECODE;//test
#endif
        //--------------------head foucs-hip---------
        if(robotwb.head_control_mode==HEAD_LOOK_B){
            robotwb.head_look_pos_h=robotwb.arm_epos_h;//look at arm_epos
            END_POS err_vec;
            err_vec.x=robotwb.head_look_pos_h.x-robotwb.head_pos_h.x;
            err_vec.y=robotwb.head_look_pos_h.y-robotwb.head_pos_h.y;
            err_vec.z=robotwb.head_look_pos_h.z-robotwb.head_pos_h.z;
            //printf("look %f %f %f\n",robotwb.head_look_pos_h.x,robotwb.head_look_pos_h.y,robotwb.head_look_pos_h.z);
            //printf("head %f %f %f\n",robotwb.head_pos_h.x,robotwb.head_pos_h.y,robotwb.head_pos_h.z);
            END_POS look_att;
            look_att.y=atan2f(err_vec.z,err_vec.x)*57.3;
            look_att.z=-atan2f(err_vec.y,err_vec.x)*57.3;
            //printf("att:%f %f\n", look_att.y, look_att.z);
            robotwb.head_att_exp[0]=look_att.z;//yaw
            robotwb.head_att_exp[1]=look_att.y;//pit
        }else if(robotwb.head_control_mode==HEAD_LOOK_E){
            robotwb.head_look_pos_e.x=0.5;
            robotwb.head_look_pos_e.y=0;
            robotwb.head_look_pos_e.z=0.15;

            converV_e_to_n(robotwb.head_look_pos_e.x,robotwb.head_look_pos_e.y,robotwb.head_look_pos_e.z,
                           &robotwb.head_look_pos_n.x,&robotwb.head_look_pos_n.y,&robotwb.head_look_pos_n.z);

            converV_n_to_b(robotwb.head_look_pos_n.x,robotwb.head_look_pos_n.y,robotwb.head_look_pos_n.z,
                           &robotwb.head_look_pos_h.x,&robotwb.head_look_pos_h.y,&robotwb.head_look_pos_h.z);

            END_POS err_vec;
            err_vec.x=robotwb.head_look_pos_h.x-robotwb.head_pos_h.x;
            err_vec.y=robotwb.head_look_pos_h.y-robotwb.head_pos_h.y;
            err_vec.z=robotwb.head_look_pos_h.z-robotwb.head_pos_h.z;
            END_POS look_att;
            look_att.y=atan2f(err_vec.z,err_vec.x)*57.3;
            look_att.z=-atan2f(err_vec.y,err_vec.x)*57.3;
            //printf("att:%f %f\n", look_att.y, look_att.z);
            robotwb.head_att_exp[0]=look_att.z;//yaw
            robotwb.head_att_exp[1]=look_att.y;//pit
        }else if(robotwb.head_control_mode==HEAD_RECODE){
#if 1
            robotwb.head_look_pos_h.x=0.55;//look at arm_epos
            robotwb.head_look_pos_h.y=-0.14;
            robotwb.head_look_pos_h.z=robotwb.arm_epos_h.z-0.05;//look at arm_epos
#else
            robotwb.head_look_pos_h.x=0.55;//look at arm_epos
            robotwb.head_look_pos_h.y=-0.135;
            robotwb.head_look_pos_h.z=robotwb.arm_epos_h.z;//look at arm_epos
#endif
            END_POS err_vec;
            err_vec.x=robotwb.head_look_pos_h.x-robotwb.head_pos_h.x;
            err_vec.y=robotwb.head_look_pos_h.y-robotwb.head_pos_h.y;
            err_vec.z=robotwb.head_look_pos_h.z-robotwb.head_pos_h.z;
            //printf("look %f %f %f\n",robotwb.head_look_pos_h.x,robotwb.head_look_pos_h.y,robotwb.head_look_pos_h.z);
            //printf("head %f %f %f\n",robotwb.head_pos_h.x,robotwb.head_pos_h.y,robotwb.head_pos_h.z);
            END_POS look_att;
            look_att.y=atan2f(err_vec.z,err_vec.x)*57.3;
            look_att.z=-atan2f(err_vec.y,err_vec.x)*57.3;
            //printf("att:%f %f\n", look_att.y, look_att.z);
#if 0
            robotwb.head_att_exp[0]=look_att.z;//yaw
            robotwb.head_att_exp[1]=look_att.y;//pit
#else
            robotwb.head_att_exp[0]=60;// pitch now
            robotwb.head_att_exp[1]=-45;// yaw now
#endif
        }
        robotwb.head_att_exp[0]=LIMIT(robotwb.head_att_exp[0],-89,89);//yaw
        robotwb.head_att_exp[1]=LIMIT(robotwb.head_att_exp[1],-45,25);//pit
       // printf("head:%f %f\n",robotwb.head_att_exp[0],robotwb.head_att_exp[1]);

#endif
}

void body_servo_control(float dt)//机体伺服控制器
{
    traj_planner(dt);

    float err=(robotwb.base_vel_b_exp.x-robotwb.base_vel_b_w_fushion.x);

    if(robotwb.base_vel_b_exp_rc_flt.x!=0)
        robotwb.base_vel_b_exp.x=robotwb.base_vel_b_exp_rc_flt.x;//m/s
    else
        robotwb.base_vel_b_exp.x=0;//m/s traj control

    float err_yaw=LIMIT(To_180_degreesw(dead(robotwb.exp_att.yaw-robotwb.IMU_now.yaw,1)),-60,60);

    if(robotwb.base_rate_exp_rc_flt!=0||!robotwb.yaw_lock){
        robotwb.exp_att.yaw=robotwb.IMU_now.yaw;
        robotwb.base_rate_exp=robotwb.base_rate_exp_rc_flt;//m/s
    }
    else
        robotwb.base_rate_exp=err_yaw*0.015;//rad/s

    force_dis_n();
}


void force_dis_n(void)//-------------------力分配
{
    float wheel_dq_exp[4]={0};
#if 0
    robotwb.base_vel_b_exp.x=0.35*1;//m/s
    robotwb.base_rate_exp=-0.55*1;//rad/s
#endif
    if(robotwb.wheel_touch_cnt){
         wheel_dq_exp[0]=robotwb.base_vel_b_exp.x*robotwb.wheel_touch[0]/robotwb.wheel_r;
         wheel_dq_exp[1]=robotwb.base_vel_b_exp.x*robotwb.wheel_touch[1]/robotwb.wheel_r;
         wheel_dq_exp[2]=robotwb.base_vel_b_exp.x*robotwb.wheel_touch[2]/robotwb.wheel_r;
         wheel_dq_exp[3]=robotwb.base_vel_b_exp.x*robotwb.wheel_touch[3]/robotwb.wheel_r;

         float d_w=robotwb.base_rate_exp*(robotwb.wheel_w);

         int cnt_l=robotwb.wheel_touch[0]+robotwb.wheel_touch[2];
         int cnt_r=robotwb.wheel_touch[1]+robotwb.wheel_touch[3];
         if(cnt_l>0&&cnt_r==0)
         {
             wheel_dq_exp[0]-=d_w*robotwb.wheel_touch[0]/robotwb.wheel_r;
             wheel_dq_exp[2]-=d_w*robotwb.wheel_touch[2]/robotwb.wheel_r;
         }else if(cnt_r>0&&cnt_l==0)
         {
             wheel_dq_exp[1]-=d_w*robotwb.wheel_touch[1]/robotwb.wheel_r;
             wheel_dq_exp[3]-=d_w*robotwb.wheel_touch[3]/robotwb.wheel_r;
         }else{
             wheel_dq_exp[0]-=d_w/robotwb.wheel_r;
             wheel_dq_exp[1]+=d_w/robotwb.wheel_r;
             wheel_dq_exp[2]-=d_w/robotwb.wheel_r;
             wheel_dq_exp[3]+=d_w/robotwb.wheel_r;
         }
    }else{
        for(int i=0;i<4;i++)
            wheel_dq_exp[i]=0;
    }

    for(int i=0;i<4;i++)
        robotwb.wheel_dq_exp[i]=wheel_dq_exp[i]*57.3;
#if 0
    printf("base exp=%f now=%f\n",robotwb.base_vel_b_exp.x,robotwb.base_vel_b_w.x);
    printf("base_yaw exp=%f now=%f\n",robotwb.base_rate_exp,robotwb.base_rate_fushion);
    printf("wheel[%d] %f %f %f %f\n",robotwb.wheel_touch_cnt,robotwb.wheel_dq_exp[0],robotwb.wheel_dq_exp[1],robotwb.wheel_dq_exp[2],robotwb.wheel_dq_exp[3]);
#endif
}


void reset_servo_interge(void)
{
	robotwb.exp_force.x = robotwb.exp_force.y = robotwb.exp_force.z = 0;
	robotwb.exp_torque.x = robotwb.exp_torque.y = robotwb.exp_torque.z = 0;
	robotwb.exp_torque_i.x = robotwb.exp_torque_i.y = robotwb.exp_torque_i.z = 0;
	robotwb.exp_force_i.x = robotwb.exp_force_i.y = robotwb.exp_force_i.z = 0;
}
