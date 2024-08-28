#ifndef _solver_mpc
#define _solver_mpc


#include <Eigen/Dense>
#include "common_types.h"
#include "convexMPC_interface.h"
#include <iostream>
#include <stdio.h>

using Eigen::Matrix;
using Eigen::Quaternionf;
using Eigen::Quaterniond;

void solve_mpc(update_data_t* update, problem_setup* setup);
void quat_to_rpy(Quaternionf q, Matrix<fpt,3,1>& rpy);
void ct_ss_mats(Matrix<fpt, 3, 3> I_world, fpt m, Matrix<fpt, 3, 2> r_feet, //修改了腿数量
	Matrix<fpt, 3, 3> R_yaw, Matrix<fpt, 13, 13>& A, Matrix<fpt, 13, 10>& Bm, //修改了控制矩阵
	float x_drag);
void resize_qp_mats(s16 horizon);
void c2qp(Matrix<fpt,13,13> Ac, Matrix<fpt,13,10> Bc,//修改了控制矩阵
	fpt dt,s16 horizon);
fpt* get_q_soln();
#endif
