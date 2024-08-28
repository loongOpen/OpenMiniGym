//#ifndef _RobotState
//#define _RobotState
#pragma once
#include <Eigen/Dense>
#include "common_types.h"
#define STATE_SIZE      13
#define OUTPUT_SIZE     10  //5*2  fx fy fz my mz
#define COND_SIZE       14  //7*2  fxu -fxu  fyu  -fyu fz  my mz
#define COND_SIZE_N     18  //9*2
#define OUTPUT_GROUP    2

#define FAST_MPC        1 //开启后MPC_PREDIC_LEN无法上10
#define MPC_PREDIC_LEN  8

#define EN_X_DRAG       1
using Eigen::Matrix;
using Eigen::Quaternionf;

	void RobotState_set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);
	void RobotState_print();

	typedef struct {
		Matrix<fpt, 3, 1> p, v, w;
		Matrix<fpt, 3, OUTPUT_GROUP> r_feet;//修改了腿数量
		Matrix<fpt, 3, 3> R;
		Matrix<fpt, 3, 3> R_yaw;
		Matrix<fpt, 3, 3> I_body;
		Quaternionf q;
		fpt yaw;
		fpt m = 6;
	}RobotState;

	extern  RobotState robot_rs;

