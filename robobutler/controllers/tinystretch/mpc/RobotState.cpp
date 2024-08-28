#include "RobotState.h"
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include "base_struct.h"
using std::cout;
using std::endl;
RobotState robot_rs;
void RobotState_set(flt* p_, flt* v_, flt* q_, flt* w_, flt* r_,flt yaw_)
{
    for(u8 i = 0; i < 3; i++)
    {
		robot_rs.p(i) = p_[i];
		robot_rs.v(i) = v_[i];
		robot_rs.w(i) = w_[i];
    }
	robot_rs.q.w() = q_[0];
	robot_rs.q.x() = q_[1];
	robot_rs.q.y() = q_[2];
	robot_rs.q.z() = q_[3];
	robot_rs.yaw = yaw_;

    for(u8 rs = 0; rs < 3; rs++)//修改了腿数量
        for(u8 c = 0; c < OUTPUT_GROUP; c++)
			robot_rs.r_feet(rs,c) = r_[rs*OUTPUT_GROUP + c];

	robot_rs.R = robot_rs.q.toRotationMatrix();

	yaw_ = 0;
    fpt yc = cos(yaw_);
    fpt ys = sin(yaw_);

	robot_rs.R_yaw <<  yc,  -ys,   0,//线性化旋转航向矩阵
					   ys,  yc,   0,
					   0,   0,   1;

    Matrix<fpt,3,1> Id;
	robot_rs.m = Mw+ robotwb.mess_payload;
	float Ix = 0.45f + robot_rs.m * robotwb.mess_off[Yr] * robotwb.mess_off[Yr];
	float Iy = 1.6f + robot_rs.m * robotwb.mess_off[Xr] * robotwb.mess_off[Xr];
	float r_off = sqrt(robotwb.mess_off[Xr] * robotwb.mess_off[Xr] + robotwb.mess_off[Yr] * robotwb.mess_off[Yr]);
	float Iz = 1.0f + robot_rs.m * r_off* r_off;
	Id << Ix, Iy, Iz;
	robot_rs.I_body.diagonal() = Id;
}

void RobotState_print()
{
   cout<<"Robot State:"<<endl<<"Position\n"<< robot_rs.p.transpose()
       <<"\nVelocity\n"<< robot_rs.v.transpose()<<"\nAngular Veloctiy\n"
       << robot_rs.w.transpose()<<"\nRotation\n"<< robot_rs.R<<"\nYaw Rotation\n"
       << robot_rs.R_yaw<<"\nFoot Locations\n"<< robot_rs.r_feet<<"\nInertia\n"<< robot_rs.I_body<<endl;
}



