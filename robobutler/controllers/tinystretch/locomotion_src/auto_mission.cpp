#include "include.h"
#include "gait_math.h"
#include "locomotion_header.h"
#if RUN_WEBOTS
#include <webots/Robot.hpp>
#include <webots/supervisor.h>
#include "wbInterface.h"
END_POS tar_waypoint;
#endif

void grasp_mission(float dt){
    float grasp_up_z=-0.07;
    float grasp_deep=0.02;
    float grasp_reset_z=0.05;
    static float timer_grasp=0;
    static int ocu_a_reg=0,ocu_ud_reg=0,ocu_key_reg=0,ocu_x_reg;
    static int init=0;
    static int cnt_p=0;
    if(!init){
        init=1;
        robotwb.arm_epos_h_grasp_off.x=0.041;
        robotwb.arm_epos_h_grasp_off.y=-0.0328;
        robotwb.arm_epos_h_grasp_off.z=0;
        robotwb.grasp_mission=0;
        robotwb.grasp_action=0;
    }
    if(ocu.key_ud==-1&&!ocu_ud_reg){
       robotwb.grasp_mission=0;
       robotwb.grasp_action=0;
    }

    switch(robotwb.grasp_mission){
        case 0:
            if(ocu.key_st&&robotwb.grasp_action==0){
                robotwb.grasp_action=1;
                printf("enable robotwb.grasp_action! z=%f\n",robotwb.arm_base_height);
                robotwb.grasp_mission++;
            }
        break;
        case 1:
             if(robotwb.grasp_action==2&&robotwb.arm_epos_h_grasp.x!=0&&robotwb.arm_epos_h_grasp.y!=0)
                 robotwb.grasp_mission++;
        break;
        case 2://locate
               robotwb.arm_att_h_exp.y=robotwb.arm_att_h_exp.y*0.2+0.8*robotwb.arm_att_b_grasp_app.y;
               robotwb.arm_att_h_exp.z=robotwb.arm_att_b_grasp_app.z;
               //robotwb.arm_att_h_exp.x=robotwb.arm_att_b_grasp.x;

               robotwb.cap_set=1;//open
#if 1
               robotwb.arm_epos_h_grasp_off.x+=dead(ocu.rc_spd_w[0],0.001)*0.0005;
               robotwb.arm_epos_h_grasp_off.y-=dead(ocu.rc_spd_w[1],0.001)*0.0005;
               robotwb.arm_epos_h_grasp_off.x=LIMIT(robotwb.arm_epos_h_grasp_off.x,-0.1,0.1);
               robotwb.arm_epos_h_grasp_off.y=LIMIT(robotwb.arm_epos_h_grasp_off.y,-0.1,0.1);
#endif
               robotwb.arm_epos_h_exp=robotwb.arm_epos_h_grasp_app;
               robotwb.arm_epos_h_exp.x+=robotwb.arm_epos_h_grasp_off.x;
               robotwb.arm_epos_h_exp.y+=robotwb.arm_epos_h_grasp_off.y;
               robotwb.arm_epos_h_exp.z+=robotwb.arm_epos_h_grasp_off.z;
                if(cnt_p++>50){cnt_p=0;
                 printf("off grasp=%f %f %f\n",robotwb.arm_epos_h_grasp_off.x,robotwb.arm_epos_h_grasp_off.y,robotwb.arm_epos_h_grasp_off.z);
                 printf("targe_pos=z=%.3f %.3f\n",robotwb.arm_epos_h_exp.z,robotwb.base_height);
                }
               robotwb.arm_epos_h_exp.z+=grasp_up_z;//<--------------------
               if(robotwb.arm_epos_h_exp.z<-0.1)
                   robotwb.arm_epos_h_exp.z=-0.1;

               if(ocu.key_a==1&&!ocu_a_reg){
                   timer_grasp=0;
                   robotwb.grasp_mission++;
               }
               else if(ocu.key_y==1){
                   timer_grasp=0;
                   robotwb.grasp_mission=7;
               }
        break;
        case 3://grasp1
            timer_grasp+=dt;
            robotwb.arm_epos_h_exp=robotwb.arm_epos_h_grasp;
            robotwb.arm_att_h_exp.y=robotwb.arm_att_b_grasp.y;
            robotwb.arm_att_h_exp.z=robotwb.arm_att_b_grasp.z;
            //robotwb.arm_att_h_exp.x=robotwb.arm_att_b_grasp.x;

            robotwb.arm_epos_h_exp.x+=robotwb.arm_epos_h_grasp_off.x;
            robotwb.arm_epos_h_exp.y+=robotwb.arm_epos_h_grasp_off.y;
            robotwb.arm_epos_h_exp.z+=robotwb.arm_epos_h_grasp_off.z;

            robotwb.arm_epos_h_exp.z+=grasp_up_z-grasp_deep;//<--------------------
            if(cnt_p++>50){cnt_p=0;

             printf("targe_pos=z=%.3f %.3f\n",robotwb.arm_epos_h_exp.z,robotwb.arm_base_height);
            }
            if((ocu.key_a==1&&!ocu_a_reg)||timer_grasp>2){
                timer_grasp=0;
                robotwb.grasp_mission++;
            }
        break;
        case 4://grasp2-capture
            timer_grasp+=dt;
            robotwb.arm_epos_h_exp=robotwb.arm_epos_h_grasp;
            robotwb.arm_att_h_exp.y=robotwb.arm_att_b_grasp.y;
            robotwb.arm_att_h_exp.z=robotwb.arm_att_b_grasp.z;

            robotwb.arm_epos_h_exp.x+=robotwb.arm_epos_h_grasp_off.x;
            robotwb.arm_epos_h_exp.y+=robotwb.arm_epos_h_grasp_off.y;
            robotwb.arm_epos_h_exp.z+=robotwb.arm_epos_h_grasp_off.z;

            robotwb.arm_epos_h_exp.z+=grasp_up_z-grasp_deep;//<--------------------

            if(cnt_p++>50){cnt_p=0;

             printf("targe_pos=z=%.3f %.3f\n",robotwb.arm_epos_h_exp.z,robotwb.arm_base_height);
            }
            robotwb.cap_set=0.1;
            if((ocu.key_a==1&&!ocu_a_reg)||timer_grasp>1){
                timer_grasp=0;
                robotwb.grasp_mission++;
            }

        break;
        case 5://grasp2-capture
            timer_grasp+=dt;
            robotwb.arm_epos_h_exp=robotwb.arm_epos_h_grasp;
            robotwb.arm_att_h_exp.y=robotwb.arm_att_b_grasp.y;
            robotwb.arm_att_h_exp.z=robotwb.arm_att_b_grasp.z;

            robotwb.arm_epos_h_exp.x+=robotwb.arm_epos_h_grasp_off.x;
            robotwb.arm_epos_h_exp.y+=robotwb.arm_epos_h_grasp_off.y;
            robotwb.arm_epos_h_exp.z+=robotwb.arm_epos_h_grasp_off.z;

            robotwb.arm_epos_h_exp.z+=grasp_up_z-grasp_deep;//<--------------------

            if(cnt_p++>50){cnt_p=0;

             printf("targe_pos=z=%.3f %.3f\n",robotwb.arm_epos_h_exp.z,robotwb.arm_base_height);
            }
            robotwb.cap_set=0.1;
            if((ocu.key_a==1&&!ocu_a_reg)||timer_grasp>1){
                timer_grasp=0;
                robotwb.grasp_mission++;
            }
        break;
        case 6://reset
            timer_grasp+=dt;
            robotwb.arm_epos_h_exp=robotwb.arm_epos_h_grasp;
            robotwb.arm_epos_h_exp.z=robotwb.arm_epos_h_grasp.z+grasp_reset_z;
            robotwb.arm_att_h_exp.x=0;
            robotwb.arm_att_h_exp.y=0;
            robotwb.arm_att_h_exp.z=0;
            if(timer_grasp>8){
                timer_grasp=0;
                robotwb.grasp_mission=0;
                robotwb.arm_control_mode=0;
                robotwb.grasp_action=0;
                printf("grasp done!\n");
            }
        break;
        case 7:
            robotwb.arm_epos_h_exp.x=0.15;
            robotwb.arm_epos_h_exp.y=-0.05;
            robotwb.arm_epos_h_exp.z=robotwb.arm_epos_h_grasp.z+grasp_reset_z;
            robotwb.arm_att_h_exp.x=0;
            robotwb.arm_att_h_exp.y=0;
            robotwb.arm_att_h_exp.z=0;
            timer_grasp+=dt;

            if(timer_grasp>8){
                timer_grasp=0;
                robotwb.grasp_mission=0;
                robotwb.arm_control_mode=0;
                robotwb.grasp_action=0;
                printf("reset done!\n");
            }

        break;
    }
    ocu_x_reg=ocu.key_x;
    ocu_key_reg=ocu.key_st;
    ocu_a_reg=ocu.key_a;
    ocu_ud_reg=ocu.key_ud;
}

void grasp_mission_auto(float dt){
    float grasp_up_z=-0.07;
    float grasp_deep=0.03;
    float grasp_reset_z=0.05;
    static float timer_grasp=0;
    static int ocu_a_reg=0,ocu_ud_reg=0,ocu_key_reg=0,ocu_x_reg;
    static int init=0;
    static int cnt_p=0;
    if(!init){
        init=1;
       robotwb.arm_epos_h_grasp_off.x=0.0117;
       robotwb.arm_epos_h_grasp_off.y=-0.04;
       robotwb.arm_epos_h_grasp_off.z=0;
       robotwb.grasp_mission=0;
       robotwb.grasp_action=0;
    }
    if(ocu.key_ud==-1&&!ocu_ud_reg){
       robotwb.grasp_mission=0;
       robotwb.grasp_action=0;
    }

    switch(robotwb.grasp_mission){
        case 0:
            if(ocu.key_st&&robotwb.grasp_action==0){
                robotwb.grasp_action=1;
                printf("enable robotwb.grasp_action! z=%f\n",robotwb.arm_base_height);
                robotwb.grasp_mission++;
            }
        break;
        case 1://check
            if(robotwb.grasp_action==2&&robotwb.arm_epos_h_grasp.x!=0&&robotwb.arm_epos_h_grasp.y!=0)
                timer_grasp+=dt;
            robotwb.arm_epos_h_exp.x=0.225;
            robotwb.arm_epos_h_exp.y=-0.29;
            robotwb.arm_epos_h_exp.z=0.2;
            robotwb.arm_att_h_exp.x=0;
            robotwb.arm_att_h_exp.y=0;
            robotwb.arm_att_h_exp.z=0;
            if((ocu.key_a==1&&!ocu_a_reg)||timer_grasp>5){
                timer_grasp=0;
                robotwb.grasp_mission++;
            }
        break;
        case 2://locate
               timer_grasp+=dt;
               robotwb.arm_att_h_exp.y=robotwb.arm_att_h_exp.y*0.2+0.8*robotwb.arm_att_b_grasp_app.y;
               robotwb.arm_att_h_exp.z=robotwb.arm_att_b_grasp_app.z;
               //robotwb.arm_att_h_exp.x=robotwb.arm_att_b_grasp.x;

               robotwb.cap_set=1;//open
#if 1
               robotwb.arm_epos_h_grasp_off.x+=dead(ocu.rc_spd_w[0],0.001)*0.0005;
               robotwb.arm_epos_h_grasp_off.y-=dead(ocu.rc_spd_w[1],0.001)*0.0005;
               robotwb.arm_epos_h_grasp_off.x=LIMIT(robotwb.arm_epos_h_grasp_off.x,-0.1,0.1);
               robotwb.arm_epos_h_grasp_off.y=LIMIT(robotwb.arm_epos_h_grasp_off.y,-0.1,0.1);
#endif
               robotwb.arm_epos_h_exp=robotwb.arm_epos_h_grasp_app;
               robotwb.arm_epos_h_exp.x+=robotwb.arm_epos_h_grasp_off.x;
               robotwb.arm_epos_h_exp.y+=robotwb.arm_epos_h_grasp_off.y;
               robotwb.arm_epos_h_exp.z+=robotwb.arm_epos_h_grasp_off.z;
                if(cnt_p++>50){cnt_p=0;
                 printf("off grasp=%f %f %f\n",robotwb.arm_epos_h_grasp_off.x,robotwb.arm_epos_h_grasp_off.y,robotwb.arm_epos_h_grasp_off.z);
                 printf("targe_pos=z=%.3f %.3f\n",robotwb.arm_epos_h_exp.z,robotwb.base_height);
                }
               robotwb.arm_epos_h_exp.z+=grasp_up_z;//<--------------------
               if(robotwb.arm_epos_h_exp.z<-0.1)
                   robotwb.arm_epos_h_exp.z=-0.1;

               if((ocu.key_a==1&&!ocu_a_reg)||timer_grasp>8){
                   timer_grasp=0;
                   robotwb.grasp_mission++;
               }else if(ocu.key_y==1){
                   timer_grasp=0;
                   robotwb.grasp_mission=6;
               }
        break;
        case 3://grasp1
            timer_grasp+=dt;
            robotwb.arm_epos_h_exp=robotwb.arm_epos_h_grasp;
            robotwb.arm_att_h_exp.y=robotwb.arm_att_b_grasp.y;
            robotwb.arm_att_h_exp.z=robotwb.arm_att_b_grasp.z;
            //robotwb.arm_att_h_exp.x=robotwb.arm_att_b_grasp.x;

            robotwb.arm_epos_h_exp.x+=robotwb.arm_epos_h_grasp_off.x;
            robotwb.arm_epos_h_exp.y+=robotwb.arm_epos_h_grasp_off.y;
            robotwb.arm_epos_h_exp.z+=robotwb.arm_epos_h_grasp_off.z;

            robotwb.arm_epos_h_exp.z+=grasp_up_z-grasp_deep;//<--------------------
            if(cnt_p++>50){cnt_p=0;

             printf("targe_pos=z=%.3f %.3f\n",robotwb.arm_epos_h_exp.z,robotwb.arm_base_height);
            }
            if((ocu.key_a==1&&!ocu_a_reg)||timer_grasp>2){
                timer_grasp=0;
                robotwb.grasp_mission++;
            }
        break;
        case 4://grasp2-capture
            timer_grasp+=dt;
            robotwb.arm_epos_h_exp=robotwb.arm_epos_h_grasp;
            robotwb.arm_att_h_exp.y=robotwb.arm_att_b_grasp.y;
            robotwb.arm_att_h_exp.z=robotwb.arm_att_b_grasp.z;

            robotwb.arm_epos_h_exp.x+=robotwb.arm_epos_h_grasp_off.x;
            robotwb.arm_epos_h_exp.y+=robotwb.arm_epos_h_grasp_off.y;
            robotwb.arm_epos_h_exp.z+=robotwb.arm_epos_h_grasp_off.z;

            robotwb.arm_epos_h_exp.z+=grasp_up_z-grasp_deep;//<--------------------

            if(cnt_p++>50){cnt_p=0;

             printf("targe_pos=z=%.3f %.3f\n",robotwb.arm_epos_h_exp.z,robotwb.arm_base_height);
            }
            robotwb.cap_set=0.1;
            if((ocu.key_a==1&&!ocu_a_reg)||timer_grasp>1){
                timer_grasp=0;
                robotwb.grasp_mission++;
            }

        break;
        case 5://up
            timer_grasp+=dt;
            robotwb.arm_epos_h_exp.x=0.15;
            robotwb.arm_epos_h_exp.y=-0.05;
            robotwb.arm_epos_h_exp.z=0.2;
            robotwb.arm_att_h_exp.x=0;
            robotwb.arm_att_h_exp.y=0;
            robotwb.arm_att_h_exp.z=0;
            if(timer_grasp>8){
                timer_grasp=0;
                robotwb.grasp_mission++;
                printf("grasp 1!\n");
            }
        break;
        case 6://put
            timer_grasp+=dt;
            robotwb.arm_epos_h_exp.x=0.225;
            robotwb.arm_epos_h_exp.y=-0.29;
            robotwb.arm_epos_h_exp.z=0.1;
            robotwb.arm_att_h_exp.x=0;
            robotwb.arm_att_h_exp.y=0;
            robotwb.arm_att_h_exp.z=0;
            if(timer_grasp>4){
                timer_grasp=0;
                robotwb.grasp_mission++;
                printf("grasp done!\n");
            }
        break;
        case 7://discap
            robotwb.cap_set=1;//open
            timer_grasp+=dt;
            robotwb.arm_epos_h_exp.x=0.225;
            robotwb.arm_epos_h_exp.y=-0.29;
            robotwb.arm_epos_h_exp.z=0.1;
            robotwb.arm_att_h_exp.x=0;
            robotwb.arm_att_h_exp.y=0;
            robotwb.arm_att_h_exp.z=0;
            if(timer_grasp>2){
                timer_grasp=0;
                robotwb.grasp_mission++;
                robotwb.grasp_action=0;
                printf("reset done!\n");
            }
        break;
        case 8://reset
            robotwb.arm_epos_h_exp.x=0.225;
            robotwb.arm_epos_h_exp.y=-0.29;
            robotwb.arm_epos_h_exp.z=0.2;
            robotwb.arm_att_h_exp.x=0;
            robotwb.arm_att_h_exp.y=0;
            robotwb.arm_att_h_exp.z=0;
            timer_grasp+=dt;
            robotwb.grasp_action=1;
            if(timer_grasp>3){
                timer_grasp=0;
                robotwb.grasp_mission=1;
                printf("reset done!\n");
            }
        break;







        case 20://reset force
            robotwb.arm_epos_h_exp.x=0.15;
            robotwb.arm_epos_h_exp.y=-0.05;
            robotwb.arm_epos_h_exp.z=robotwb.arm_epos_h_grasp.z+grasp_reset_z;
            robotwb.arm_att_h_exp.x=0;
            robotwb.arm_att_h_exp.y=0;
            robotwb.arm_att_h_exp.z=0;
            timer_grasp+=dt;

            if(timer_grasp>8){
                timer_grasp=0;
                robotwb.grasp_mission=0;
                robotwb.arm_control_mode=0;
                robotwb.grasp_action=0;
                printf("reset done!\n");
            }

        break;
    }
    ocu_x_reg=ocu.key_x;
    ocu_key_reg=ocu.key_st;
    ocu_a_reg=ocu.key_a;
    ocu_ud_reg=ocu.key_ud;
}
