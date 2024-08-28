#include "SolverMPC.h"
#include "common_types.h"
#include "convexMPC_interface.h"
#include "RobotState.h"
//#include <JCQP/QpProblem.h>
#include <Eigen/Dense>
#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>
#include <qpOASES.hpp>

#include <stdio.h>
#include <time.h>

//#define K_PRINT_EVERYTHING
#define BIG_NUMBER 5e10//
//big enough to act like infinity, small enough to avoid numerical weirdness.
using std::cout;
using std::endl;
using Eigen::Dynamic;
//????STATE_SIZE???? OUTPUT_SIZE-??OUTPUT_SIZE  ??????????????25=STATE_SIZE+12-??STATE_SIZE+OUTPUT_SIZE=STATE_SIZE+OUTPUT_SIZE    ???????20-??  4??????????????5=20  fx fy fz  -??  ?????5???????+2???????? =2*??5+7??-??COND_SIZE
Matrix<fpt, STATE_SIZE * MPC_PREDIC_LEN, STATE_SIZE> A_qp;//A????
Matrix<fpt, STATE_SIZE * MPC_PREDIC_LEN, OUTPUT_SIZE * MPC_PREDIC_LEN> B_qp;//B->OUTPUT_SIZE???????????3????+2?????    STATE_SIZE*OUTPUT_SIZE
Matrix<fpt, STATE_SIZE, OUTPUT_SIZE> Bdt;//B????????? b??Matrix<fpt, STATE_SIZE, OUTPUT_SIZE> Bdt
Matrix<fpt, STATE_SIZE, STATE_SIZE> Adt;//A????????? ????
Matrix<fpt, STATE_SIZE + OUTPUT_SIZE, STATE_SIZE + OUTPUT_SIZE> ABc, expmm;
Eigen::DiagonalMatrix<fpt, STATE_SIZE * MPC_PREDIC_LEN> S;//????  ?????? STATE_SIZE*STATE_SIZE
//Matrix<fpt, STATE_SIZE * MPC_PREDIC_LEN, STATE_SIZE * MPC_PREDIC_LEN > S;
Matrix<fpt, STATE_SIZE * MPC_PREDIC_LEN, 1> X_d;//??????????
Matrix<fpt, COND_SIZE * MPC_PREDIC_LEN, 1> U_b;//???????????  1????7????????????5???? 2??????  2????COND_SIZE??
Matrix<fpt, COND_SIZE * MPC_PREDIC_LEN, OUTPUT_SIZE * MPC_PREDIC_LEN, Eigen::RowMajor> fmat;//???????


Matrix<fpt, OUTPUT_SIZE * MPC_PREDIC_LEN, OUTPUT_SIZE * MPC_PREDIC_LEN, Eigen::RowMajor> qH;//OUTPUT_SIZE*OUTPUT_SIZE
Matrix<fpt, OUTPUT_SIZE * MPC_PREDIC_LEN, 1> qg;//OUTPUT_SIZE*1
Matrix<fpt, OUTPUT_SIZE * MPC_PREDIC_LEN, OUTPUT_SIZE * MPC_PREDIC_LEN> eye_10h;// OUTPUT_SIZE*OUTPUT_SIZE  OUTPUT_SIZE?????????????????????

//qH = 2 * (B_qp.transpose()*S*B_qp + update->alpha*eye_10h); OUTPUT_SIZE*OUTPUT_SIZE
//qg = 2 * B_qp.transpose()*S*   OUTPUT_SIZE*STATE_SIZE   (A_qp*x_0 - X_d);STATE_SIZE*1    OUTPUT_SIZE*1

qpOASES::real_t* H_qpoases;
qpOASES::real_t* g_qpoases;
qpOASES::real_t* A_qpoases;
qpOASES::real_t* lb_qpoases;
qpOASES::real_t* ub_qpoases;
qpOASES::real_t* q_soln;

qpOASES::real_t* H_red;
qpOASES::real_t* g_red;
qpOASES::real_t* A_red;
qpOASES::real_t* lb_red;
qpOASES::real_t* ub_red;
qpOASES::real_t* q_red;
u8 real_allocated = 0;

char var_elim[6000];
char con_elim[6000];


float t_min(float a, float b)
{
	if (a < b) return a;
	return b;
}

float sq(float a)
{
	return a * a;
}

fpt* get_q_soln()
{
	return q_soln;
}

s8 near_zero(fpt a)
{
	return (a < 0.000001 && a > -.000001);
}

s8 near_one(fpt a)
{
	return near_zero(a - 1);
}
void matrix_to_real(qpOASES::real_t* dst, Matrix<fpt, Dynamic, Dynamic> src, s16 rows, s16 cols)
{
	s32 a = 0;
	for (s16 r = 0; r < rows; r++)
	{
		for (s16 c = 0; c < cols; c++)
		{
			dst[a] = src(r, c);
			a++;
		}
	}
}

void quat_to_rpy(Quaternionf q, Matrix<fpt, 3, 1>& rpy)
{
	//edge case!
	fpt as = t_min(-2.*(q.x()*q.z() - q.w()*q.y()), .99999);
	rpy(0) = atan2(2.f*(q.x()*q.y() + q.w()*q.z()), sq(q.w()) + sq(q.x()) - sq(q.y()) - sq(q.z()));
	rpy(1) = asin(as);
	rpy(2) = atan2(2.f*(q.y()*q.z() + q.w()*q.x()), sq(q.w()) - sq(q.x()) - sq(q.y()) + sq(q.z()));
	//printf("rpy=%f %f %f\n", rpy(0), rpy(1), rpy(2));
}

inline Matrix<fpt, 3, 3> cross_mat(Matrix<fpt, 3, 3> I_inv, Matrix<fpt, 3, 1> r)
{
	Matrix<fpt, 3, 3> cm;
	cm << 0.f, -r(2), r(1),
		r(2), 0.f, -r(0),
		-r(1), r(0), 0.f;
	return I_inv * cm;
}

void resize_qp_mats(s16 horizon)
{
	int mcount = 0;
	int h2 = horizon * horizon;
	mcount = 0;
	A_qp.setZero();
	B_qp.setZero();
	S.setZero();
	X_d.setZero();
	U_b.setZero();
	fmat.setZero();
	qH.setZero();
	eye_10h.setIdentity();

	if (real_allocated)
	{
		free(H_qpoases);
		free(g_qpoases);
		free(A_qpoases);
		free(lb_qpoases);
		free(ub_qpoases);
		free(q_soln);
		free(H_red);
		free(g_red);
		free(A_red);
		free(lb_red);
		free(ub_red);
		free(q_red);
	}

	H_qpoases = (qpOASES::real_t*)malloc(OUTPUT_SIZE * OUTPUT_SIZE * horizon*horizon * sizeof(qpOASES::real_t));
	mcount += OUTPUT_SIZE * OUTPUT_SIZE * h2;
	g_qpoases = (qpOASES::real_t*)malloc(OUTPUT_SIZE * 1 * horizon * sizeof(qpOASES::real_t));
	mcount += OUTPUT_SIZE * horizon;
	A_qpoases = (qpOASES::real_t*)malloc(OUTPUT_SIZE * COND_SIZE * horizon*horizon * sizeof(qpOASES::real_t));
	mcount += OUTPUT_SIZE * COND_SIZE * h2;
	lb_qpoases = (qpOASES::real_t*)malloc(COND_SIZE * 1 * horizon * sizeof(qpOASES::real_t));
	mcount += COND_SIZE * horizon;
	ub_qpoases = (qpOASES::real_t*)malloc(COND_SIZE * 1 * horizon * sizeof(qpOASES::real_t));
	mcount += COND_SIZE * horizon;
	q_soln = (qpOASES::real_t*)malloc(OUTPUT_SIZE * horizon * sizeof(qpOASES::real_t));
	mcount += OUTPUT_SIZE * horizon;

	H_red = (qpOASES::real_t*)malloc(OUTPUT_SIZE * OUTPUT_SIZE * horizon*horizon * sizeof(qpOASES::real_t));
	mcount += OUTPUT_SIZE * OUTPUT_SIZE * h2;
	g_red = (qpOASES::real_t*)malloc(OUTPUT_SIZE * 1 * horizon * sizeof(qpOASES::real_t));
	mcount += OUTPUT_SIZE * horizon;
	A_red = (qpOASES::real_t*)malloc(OUTPUT_SIZE * COND_SIZE * horizon*horizon * sizeof(qpOASES::real_t));
	mcount += OUTPUT_SIZE * COND_SIZE * h2;
	lb_red = (qpOASES::real_t*)malloc(COND_SIZE * 1 * horizon * sizeof(qpOASES::real_t));
	mcount += COND_SIZE * horizon;
	ub_red = (qpOASES::real_t*)malloc(COND_SIZE * 1 * horizon * sizeof(qpOASES::real_t));
	mcount += COND_SIZE * horizon;
	q_red = (qpOASES::real_t*)malloc(OUTPUT_SIZE * horizon * sizeof(qpOASES::real_t));
	mcount += OUTPUT_SIZE * horizon;
	real_allocated = 1;
}

//???????????????? ???B????
void ct_ss_mats(Matrix<fpt, 3, 3> I_world, fpt m, Matrix<fpt, 3, 2> r_feet, Matrix<fpt, 3, 3> R_yaw,
	Matrix<fpt, STATE_SIZE, STATE_SIZE>& A, Matrix<fpt, STATE_SIZE, OUTPUT_SIZE>& Bm, float x_drag)
{
	A.setZero();
	A(3, 9) = 1.f;
	A(4, 10) = 1.f;
	A(5, 11) = 1.f;

	A(11, 9) = 0;//;//??????????????????
	A(11, 12) = 1.f;
	A.block(0, 6, 3, 3) = R_yaw.transpose();//A?????

	Bm.setZero();
	Matrix<fpt, 3, 3> I_inv = I_world.inverse();

	Matrix<fpt, 3, 2> L_foot;//??????????? My Mz
	L_foot << 0, 0,
		      1, 0,
		      0, 1;
	for (s16 b = 0; b < 2; b++)//  fx fy fz my mz
	{
		Bm.block(6, b * OUTPUT_SIZE / OUTPUT_GROUP, 3, 3) = cross_mat(I_inv, r_feet.col(b));
		Bm.block(9, b * OUTPUT_SIZE / OUTPUT_GROUP, 3, 3) = Matrix<fpt, 3, 3>::Identity() / m;
		//b???  ???????????
		Bm.block(6, b * OUTPUT_SIZE / OUTPUT_GROUP + 3, 3, 2) = I_inv * L_foot * 1;
		//Bm.block(6, 6+ b * 2, 3, 2) = I_inv * L_foot * 1;
	}

	//cout << "A:" << endl;
	//cout << A << endl;
    //cout << "Bm:" << endl;cout << Bm << endl;
}

//????????????qp??????
//?????????????????????????????§Ö???????????????????????????????????????
void c2qp(Matrix<fpt, STATE_SIZE, STATE_SIZE> Ac, Matrix<fpt, STATE_SIZE, OUTPUT_SIZE> Bc, fpt dt, s16 horizon)
{
	ABc.setZero();
	ABc.block(0, 0, STATE_SIZE, STATE_SIZE) = Ac;
	ABc.block(0, STATE_SIZE, STATE_SIZE, OUTPUT_SIZE) = Bc;
	ABc = dt * ABc;
	expmm = ABc.exp();
	Adt = expmm.block(0, 0, STATE_SIZE, STATE_SIZE);
	Bdt = expmm.block(0, STATE_SIZE, STATE_SIZE, OUTPUT_SIZE);
#if 0
	cout << "Adt: \n" << Adt << "\nBdt:\n" << Bdt << endl;
#endif
	if (horizon > 19) {
		throw std::runtime_error("horizon is too long!");
	}

	Matrix<fpt, STATE_SIZE, STATE_SIZE> powerMats[20];
	powerMats[0].setIdentity();
	for (int i = 1; i < horizon + 1; i++) {
		powerMats[i] = Adt * powerMats[i - 1];
	}

	for (s16 r = 0; r < horizon; r++)
	{
		A_qp.block(STATE_SIZE * r, 0, STATE_SIZE, STATE_SIZE) = powerMats[r + 1];//Adt.pow(r+1);
		for (s16 c = 0; c < horizon; c++)
		{
			if (r >= c)
			{
				s16 a_num = r - c;
				B_qp.block(STATE_SIZE * r, OUTPUT_SIZE * c, STATE_SIZE, OUTPUT_SIZE) = powerMats[a_num] /*Adt.pow(a_num)*/ * Bdt;
			}
		}
	}

#if 0
	cout << "AQP:\n" << A_qp << "\nBQP:\n" << B_qp << endl;
#endif
}

//----------------------------------??????
Matrix<fpt, STATE_SIZE, 1> x_0;//?????????
Matrix<fpt, 3, 3> I_world;
Matrix<fpt, STATE_SIZE, STATE_SIZE> A_ct;
Matrix<fpt, STATE_SIZE, OUTPUT_SIZE> B_ct_r;

void solve_mpc(update_data_t* update, problem_setup* setup)
{
	RobotState_set(update->p, update->v, update->q, update->w, update->r, update->yaw);

	//roll pitch yaw
	Matrix<fpt, 3, 1> rpy;
	quat_to_rpy(robot_rs.q, rpy);

	//initial state (STATE_SIZE state representation)
	x_0 << rpy(2), rpy(1), rpy(0), robot_rs.p, robot_rs.w, robot_rs.v, -9.8f;
	//cout << "x_0 :"<<x_0.transpose() << endl;
	I_world = robot_rs.R_yaw * robot_rs.I_body * robot_rs.R_yaw.transpose(); //original
	//printf("s0\n");
	//cout << robot_rs.R_yaw << endl;
	//cout << robot_rs.I_body << endl;
	ct_ss_mats(I_world, robot_rs.m, robot_rs.r_feet, robot_rs.R_yaw, A_ct, B_ct_r, update->x_drag);//good
	//printf("s1\n");
	//cout << robot_rs.r_feet << endl;
	//QP matrices
	c2qp(A_ct, B_ct_r, setup->dt, setup->horizon);
	//printf("s2\n");
	//weights
	Matrix<fpt, STATE_SIZE, 1> full_weight;//12+1????????1??????????
	for (u8 i = 0; i < 12; i++)
		full_weight(i) = update->weights[i];
	full_weight(12) = 0.f;//?????????????0
	S.diagonal() = full_weight.replicate(setup->horizon, 1);
	//cout<<S<<endl;
	//??????trajectory
	for (s16 i = 0; i < setup->horizon; i++)
	{
		for (s16 j = 0; j < 12; j++)
			X_d(STATE_SIZE * i + j, 0) = update->traj[12 * i + j];
		X_d(STATE_SIZE * i + 12, 0) = -9.81;
	}
	//cout << X_d << endl;
	//???????
	s16 k = 0;
	float small_rate = 0.01;
	for (s16 i = 0; i < setup->horizon; i++)
	{
		for (s16 j = 0; j < OUTPUT_GROUP; j++)
		{
			U_b(COND_SIZE / OUTPUT_GROUP * k + 0) = BIG_NUMBER;
			U_b(COND_SIZE / OUTPUT_GROUP * k + 1) = BIG_NUMBER;
			U_b(COND_SIZE / OUTPUT_GROUP * k + 2) = BIG_NUMBER;
			U_b(COND_SIZE / OUTPUT_GROUP * k + 3) = BIG_NUMBER;
#if 0
			U_b(COND_SIZE / OUTPUT_GROUP * k + 4) = update->gait[i * 2 + j] * setup->f_max;
			U_b(COND_SIZE / OUTPUT_GROUP * k + 5) = update->gait[i * 2 + j] * setup->my_max;//???????
			U_b(COND_SIZE / OUTPUT_GROUP * k + 6) = update->gait[i * 2 + j] * setup->mz_max;//???????
#else
			if (update->Gflag[j] == 0) {
				U_b(COND_SIZE / OUTPUT_GROUP * k + 4) = small_rate;// update->Gflag[j] * setup->f_max;
				U_b(COND_SIZE / OUTPUT_GROUP * k + 5) = small_rate;//update->Gflag[j] * setup->my_max;//???????
				U_b(COND_SIZE / OUTPUT_GROUP * k + 6) = small_rate;//update->Gflag[j] * setup->mz_max;//???????
				//U_b(COND_SIZE / OUTPUT_GROUP * k + 7) = small_rate;// update->Gflag[j] * setup->my_max;//???????
				//U_b(COND_SIZE / OUTPUT_GROUP * k + 8) = small_rate;//update->Gflag[j] * setup->mz_max;//???????
			}
			else {
				U_b(COND_SIZE / OUTPUT_GROUP * k + 4) = update->Gflag[j] * setup->f_max;
				U_b(COND_SIZE / OUTPUT_GROUP * k + 5) = update->Gflag[j] * setup->my_max;//???????
				U_b(COND_SIZE / OUTPUT_GROUP * k + 6) = update->Gflag[j] * setup->mz_max;//???????
				//U_b(COND_SIZE / OUTPUT_GROUP * k + 7) = update->Gflag[j] * setup->my_max;//???????
				//U_b(COND_SIZE / OUTPUT_GROUP * k + 8) = update->Gflag[j] * setup->mz_max;//???????
			}
#endif
			k++;
		}
	}
	//printf("s3 setup->my_max=%f %f\n", setup->my_max, setup->mz_max);
	//????????
	fpt mu = 1.f / setup->mu;
	Matrix<fpt, COND_SIZE / OUTPUT_GROUP, OUTPUT_SIZE / OUTPUT_GROUP> f_block;//7???????? ??5?????????? fx fy fz my mz
	//        fx fy fz my mz
	f_block << mu, 0, 1.f, 0, 0,
				-mu, 0, 1.f, 0, 0,
				0, mu, 1.f, 0, 0,
				0, -mu, 1.f, 0, 0,
				0, 0, 1.f, 0, 0,//??????
				0, 0, 0, 1.f, 0,//????my
				0, 0, 0, 0, 1.f//????mz
	   // 0, 0,   0,    -1.f, 0,//????my
	   // 0, 0,   0,    0,  -1.f//????mz
		;

	for (s16 i = 0; i < setup->horizon * OUTPUT_GROUP; i++)//????????????
	{
		fmat.block(i * COND_SIZE / OUTPUT_GROUP, i * OUTPUT_SIZE / OUTPUT_GROUP,
			COND_SIZE / OUTPUT_GROUP, OUTPUT_SIZE / OUTPUT_GROUP) = f_block; //7*5???????? *2
	}
	//cout << fmat << endl;

	for (s16 i = 0; i < COND_SIZE * setup->horizon; i++)//???????????
		lb_qpoases[i] = 0.0f;

	k = 0;
#if 1
	for (s16 i = 0; i < setup->horizon; i++)//?????????
	{
		for (s16 j = 0; j < OUTPUT_GROUP; j++)
		{
			lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 0] = 0;
			lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 1] = 0;
			lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 2] = 0;
			lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 3] = 0;
#if 0
			lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 4] = update->gait[i * 2 + j] * setup->f_min;
			lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 5] = update->gait[i * 2 + j] * -setup->my_max;//???????
			lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 6] = update->gait[i * 2 + j] * -setup->mz_max;//???????
#else
			if (update->Gflag[j] == 0) {
				lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 4] = small_rate;//update->Gflag[j] * setup->f_min;//??§³??
				lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 5] = small_rate;//update->Gflag[j] * -setup->my_max;//??§³my
				lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 6] = small_rate;//update->Gflag[j] * -setup->mz_max;//??§³mz
				//lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 7] = small_rate;//update->Gflag[j] * -setup->my_max;//??§³my
				//lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 8] = small_rate;//update->Gflag[j] * -setup->mz_max;//??§³mz
			}
			else {
				lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 4] = update->Gflag[j] * setup->f_min;//??§³??
				lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 5] = update->Gflag[j] * -setup->my_max;//??§³my
				lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 6] = update->Gflag[j] * -setup->mz_max;//??§³mz
				//lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 7] = update->Gflag[j] * -setup->my_max;//??§³my
				//lb_qpoases[COND_SIZE / OUTPUT_GROUP * k + 8] = update->Gflag[j] * -setup->mz_max;//??§³mz
			}
#endif
			k++;
		}
	}
#endif

#if 1//??????? ??????????
	Matrix<fpt, OUTPUT_SIZE * MPC_PREDIC_LEN, OUTPUT_SIZE * MPC_PREDIC_LEN> alpht_eye = update->alpha*eye_10h;

	for (s16 i = 0; i < setup->horizon; i++)//
	{
		alpht_eye(OUTPUT_SIZE * i + 3, OUTPUT_SIZE * i + 3) = update->alpha_end; //
		alpht_eye(OUTPUT_SIZE * i + 4, OUTPUT_SIZE * i + 4) = update->alpha_end; //
		alpht_eye(OUTPUT_SIZE * i + 8, OUTPUT_SIZE * i + 8) = update->alpha_end; //
		alpht_eye(OUTPUT_SIZE * i + 9, OUTPUT_SIZE * i + 9) = update->alpha_end; //
	}
	//cout << update->alpha*eye_10h << endl;
#endif

	qH = 2 * (B_qp.transpose()*S*B_qp + alpht_eye);
	qg = 2 * B_qp.transpose()*S*(A_qp*x_0 - X_d);
	//printf("s4\n");
	matrix_to_real(H_qpoases, qH, setup->horizon * OUTPUT_SIZE, setup->horizon * OUTPUT_SIZE);
	matrix_to_real(g_qpoases, qg, setup->horizon * OUTPUT_SIZE, 1);
	matrix_to_real(A_qpoases, fmat, setup->horizon * COND_SIZE, setup->horizon * OUTPUT_SIZE);
	matrix_to_real(ub_qpoases, U_b, setup->horizon * COND_SIZE, 1);

	//printf(" setup->f_min=%f ", setup->f_min);
	//for (s16 i = 0; i < 36; i++)
	// printf("%d ", update->gait[i]);
	//printf("\n");

	s16 num_constraints = COND_SIZE * setup->horizon;//???
	s16 num_variables = OUTPUT_SIZE * setup->horizon;//???? ?????????? fx fy fz my mz *2

	qpOASES::int_t nWSR = 100;

	int new_vars = num_variables;
	int new_cons = num_constraints;

	for (int i = 0; i < num_constraints; i++)
		con_elim[i] = 0;

	for (int i = 0; i < num_variables; i++)
		var_elim[i] = 0;

	//printf("s51 num_variables=%d %d\n", new_vars, new_cons);
	for (int i = 0; i < num_constraints; i++)
	{
		if (!(near_zero(lb_qpoases[i]) && near_zero(ub_qpoases[i])))
			continue;
		float* c_row = &A_qpoases[i*num_variables];
		for (int j = 0; j < num_variables; j++)
		{
			if (near_one(c_row[j]))
			{
				new_vars -= OUTPUT_SIZE / OUTPUT_GROUP;//??????-5	????œý?????????????? 1?????????
				new_cons -= COND_SIZE / OUTPUT_GROUP;//?????-7	?????????????????§¹

				int cs = (j * COND_SIZE / OUTPUT_GROUP) / (OUTPUT_SIZE / OUTPUT_GROUP) - OUTPUT_SIZE / OUTPUT_GROUP;
#if 1
				for (int k = 0; k < OUTPUT_SIZE / OUTPUT_GROUP; k++)
					var_elim[j - k] = 1;

				for (int k = 0; k < COND_SIZE / OUTPUT_GROUP; k++)
					con_elim[cs + k] = 1;
#else
				var_elim[j - 4] = 1;//?????????1 ????*5 ????
				var_elim[j - 3] = 1;//
				var_elim[j - 2] = 1;//
				var_elim[j - 1] = 1;
				var_elim[j] = 1;
				con_elim[cs] = 1;
				con_elim[cs + 1] = 1;//?????????1 ???*7 ????
				con_elim[cs + 2] = 1;
				con_elim[cs + 3] = 1;
				con_elim[cs + 4] = 1;
				con_elim[cs + 5] = 1;
				con_elim[cs + 6] = 1;
#endif
			}
		}
	}
	//printf("s6=%d\n", setup->horizon);
	//printf("s7 %d %d |%d %d\n", new_vars, new_cons, update->Gflag[0], update->Gflag[1]);
	if (new_cons > 0 && new_vars > 0)
	{
		int var_ind[OUTPUT_SIZE * MPC_PREDIC_LEN];//????OUTPUT_SIZE*win
		int con_ind[COND_SIZE * MPC_PREDIC_LEN];//???COND_SIZE*win
		int vc = 0;
		for (int i = 0; i < num_variables; i++)
		{
			if (!var_elim[i])
			{
				if (!(vc < new_vars))
				{
					printf("BAD ERROR 1\n");
				}
				var_ind[vc] = i;
				vc++;
			}
		}
		vc = 0;
		for (int i = 0; i < num_constraints; i++)
		{
			if (!con_elim[i])
			{
				if (!(vc < new_cons))
				{
					printf("BAD ERROR 1\n");
				}
				con_ind[vc] = i;
				vc++;
			}
		}
		for (int i = 0; i < new_vars; i++)
		{
			int olda = var_ind[i];
			g_red[i] = g_qpoases[olda];
			for (int j = 0; j < new_vars; j++)
			{
				int oldb = var_ind[j];
				H_red[i*new_vars + j] = H_qpoases[olda*num_variables + oldb];
			}
		}

		for (int con = 0; con < new_cons; con++)
		{
			for (int st = 0; st < new_vars; st++)
			{
				float cval = A_qpoases[(num_variables*con_ind[con]) + var_ind[st]];
				A_red[con*new_vars + st] = cval;
			}
		}
		for (int i = 0; i < new_cons; i++)
		{
			int old = con_ind[i];
			ub_red[i] = ub_qpoases[old];
			lb_red[i] = lb_qpoases[old];
		}


		qpOASES::QProblem problem_red(new_vars, new_cons);
		//printf("s71\n");
		qpOASES::Options op;
		op.setToMPC();
		//op.setToFast();
		//op.enableEqualities = qpOASES::BT_TRUE;
		//op.setToReliable();
		op.printLevel = qpOASES::PL_NONE;
		problem_red.setOptions(op);
#if 0
		int rval = problem_red.init(H_qpoases, g_qpoases, A_qpoases, NULL, NULL, lb_qpoases, ub_qpoases, nWSR);
		(void)rval;
		int rval2 = problem_red.getPrimalSolution(q_red);
#else
		int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
		(void)rval;
		int rval2 = problem_red.getPrimalSolution(q_red);
#endif
		if (rval2 != qpOASES::SUCCESSFUL_RETURN)
			printf("failed to solve!\n");

		vc = 0;

		for (int i = 0; i < num_variables; i++)
		{
			if (var_elim[i])
			{
				q_soln[i] = 0.0f;
			}
			else
			{
				q_soln[i] = q_red[vc];
				vc++;
			}
		}
		//printf("s8 finish\n");
	}
}
