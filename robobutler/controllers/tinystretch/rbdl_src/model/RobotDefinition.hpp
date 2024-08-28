#ifndef ROBOT_DEFINITION
#define ROBOT_DEFINITION

#include "common.hpp"
#if RUN_WEBOTS
#define THIS_COM "/home/pi/Documents/tinystrech/protos/TinyStretch/urdf/TinyStretch_arm1.urdf"
#else
#define THIS_COM "/home/odroid/robobutler/urdf/TinyStretch_arm1.urdf"
#endif
#define NUM_DOF 6
#define NUM_Q NUM_DOF+6+1
#define NUM_DQ NUM_DOF+6

namespace robot{
    constexpr int num_q = NUM_DOF+6+1;//6+1+dof
    constexpr int num_qdot = NUM_DOF+6;//6+dof
    constexpr int num_act_joint = NUM_DOF;
    constexpr int num_virtual = 6;
};

namespace robot_link{
    constexpr int base_link = 2;
    constexpr int arm_base_link = 3;
    constexpr int arm0_link = 4;
    constexpr int arm1_link = 5;
    constexpr int arm2_link = 6;
    constexpr int arm3_link = 7;
    constexpr int arm4_link = 8;
};
namespace robot_joint{
    constexpr int virtual_X = 0;
    constexpr int virtual_Y = 1;
    constexpr int virtual_Z = 2;
    constexpr int virtual_Rx = 3;
    constexpr int virtual_Ry = 4;
    constexpr int virtual_Rz = 5;

    constexpr int arm_base_joint = 6;
    constexpr int arm0_joint = 7;
    constexpr int arm1_joint = 8;
    constexpr int arm2_joint = 9;
    constexpr int arm3_joint = 10;
    constexpr int arm4_joint = 11;
    constexpr int virtual_Rw = 12;
};
extern const char* link_name[];

#endif

