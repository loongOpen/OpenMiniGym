#include "gait_math.h"
#include "eso.h"
#include "locomotion_header.h"

void  Gait_Stand_Active(){

    robotwb.robot_mode=STAND_RC;

    robotwb.head_control_mode=HEAD_FREE;
    robotwb.arm_control_mode=ARM_M_H;

    robotwb.arm_epos_h_exp.x=0.3235;
    robotwb.arm_epos_h_exp.y=-0.15;
    robotwb.arm_epos_h_exp.z=0;

    robotwb.arm_att_h_exp.x=0;
    robotwb.arm_att_h_exp.y=0;
    robotwb.arm_att_h_exp.z=0;

    robotwb.head_att_exp[0]=robotwb.head_att_exp[1]=0;
    robotwb.cap_set=1;//open

    robotwb.yaw_lock=0;
    printf("Active Stande Mode!\n");
}


void Gait_Stand_Update(float dt)
{



}
