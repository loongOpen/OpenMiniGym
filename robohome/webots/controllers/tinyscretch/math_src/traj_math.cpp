#include "include.h"
#include "eso.h"
#include "locomotion_header.h"
#include "gait_math.h"
#if !RUN_WEBOTS
#include "arm_math.h"
#endif


//------------------Slip model
void SLIP_Traj_Reset(int phase) {//set slip model init state
	slip_mode.l0 = robotwb.exp_pos_n.z*1.2;
	slip_mode.kp = vmc_robot_p.pos_z_trot.kp;
	slip_mode.kd = vmc_robot_p.pos_z_trot.kd;

	slip_mode.stance_flag = 1;
	if (phase == 0) {//which leg is in stance
		slip_mode.foot_base.x = vmc[0].epos_n.x / 2 + vmc[3].epos_n.x / 2;
		slip_mode.foot_base.y = vmc[0].epos_n.y / 2 + vmc[3].epos_n.y / 2;
		slip_mode.foot_base.z = 0;
	}
	else {
		slip_mode.foot_base.x = vmc[1].epos_n.x / 2 + vmc[2].epos_n.x / 2;
		slip_mode.foot_base.y = vmc[1].epos_n.y / 2 + vmc[2].epos_n.y / 2;
		slip_mode.foot_base.z = 0;
	}
	slip_mode.pos.x = 0-slip_mode.foot_base.x;
	slip_mode.pos.y = 0-slip_mode.foot_base.y;
	slip_mode.pos.z = vmc_all.pos_n.z-slip_mode.foot_base.z;

	slip_mode.l0 = sqrt(slip_mode.pos.x*slip_mode.pos.x+ slip_mode.pos.z*slip_mode.pos.z);

	slip_mode.vel.x = vmc_all.param.tar_spd_use_rc.x;// vmc_all.spd_n.x;
	slip_mode.vel.y = vmc_all.param.tar_spd_use_rc.y;// vmc_all.spd_n.y;
	slip_mode.vel.z = 0;// vmc_all.spd_n.z;

	slip_mode.acc.x = 0;
	slip_mode.acc.y = 0;
	slip_mode.acc.z = -9.81;
	printf("rst pos slip:%f %f %f\n", slip_mode.pos.x, slip_mode.pos.y, slip_mode.pos.z);
	printf("rst vel slip:%f %f %f\n", slip_mode.vel.x, slip_mode.vel.y, slip_mode.vel.z);
}

void SLIP_Traj_Model(float dt) {
	static int init = 0;
	float dt_gain = 5;
	if (!init) {
		init = 1;
		SLIP_Traj_Reset(0);
		slip_mode.pos.z = 0.7*MAX_Z;
	}
	//l1=sqrt(q(1)^2+q(2)^2); % spring length
	//u=0;%高度控制
	//a1=K*(l_0-l1)/mass+u;       % acceleration of point mass
	//dqdt(3,1) = a1/l1*q(1);%ddx
	//dqdt(4,1) = a1/l1*q(2) -q(4)*D- g;%ddy

	float l1 = sqrt(slip_mode.pos.x*slip_mode.pos.x + slip_mode.pos.z*slip_mode.pos.z);
	//float a1 = slip_mode.kp / Mw * (slip_mode.l0 - l1);
	//float l2 = sqrt(slip_mode.pos.y*slip_mode.pos.y + slip_mode.pos.z*slip_mode.pos.z);
	//float a2 = slip_mode.kp / Mw * (slip_mode.l0 - l2);
	float d_l = (slip_mode.l0 - l1);
	float force_s = slip_mode.kp*d_l;
	float sita_x = atan2f(slip_mode.pos.x, slip_mode.pos.z)*57.3;
	float force_x = -sind(sita_x)*force_s;
	float force_z = cosd(sita_x)*force_s;
	//printf("slip %f %f d_l=%f ff=%f sita=%f\n", slip_mode.l0, l1,d_l, force_s, sita_x);
	//if (l1 != 0)
	if (slip_mode.stance_flag) {
			if (l1 < slip_mode.l0*1.1) {//stance
				slip_mode.acc.x = force_x / Mw;
				slip_mode.acc.z = force_z / Mw - 9.81; //slip_mode.acc.z = slip_mode.pos.z*a1 / l1 - slip_mode.vel.z*slip_mode.kd - 9.81;
				slip_mode.force.x = force_x;
				slip_mode.force.z = force_z;
			}
			else
			{
				if (slip_mode.stance_flag)
					slip_mode.stance_flag = 0;
				slip_mode.acc.x = 0;
				slip_mode.acc.z = -9.81;
				slip_mode.force.x = 0;
				slip_mode.force.z = 0;
			}

		slip_mode.acc.x = LIMIT(slip_mode.acc.x, -12, 12);
		slip_mode.acc.y = LIMIT(slip_mode.acc.y, -12, 12);
		slip_mode.acc.z = LIMIT(slip_mode.acc.z, -12, 12);
		slip_mode.vel.x += dt * dt_gain* slip_mode.acc.x;
		slip_mode.vel.y += dt * dt_gain* slip_mode.acc.y;
		slip_mode.vel.z += dt * dt_gain* slip_mode.acc.z;

		slip_mode.pos.x += dt * dt_gain*slip_mode.vel.x + 0.5*dt*dt*dt_gain*dt_gain*slip_mode.acc.x;
		slip_mode.pos.y += dt * dt_gain*slip_mode.vel.y + 0.5*dt*dt*dt_gain*dt_gain*slip_mode.acc.y;
		slip_mode.pos.z += dt * dt_gain*slip_mode.vel.z + 0.5*dt*dt*dt_gain*dt_gain*slip_mode.acc.z;
	}
	else {
		slip_mode.force.x = 0;
		slip_mode.force.z = 0;
	}
	//printf("pos %f %f %f\n", slip_mode.pos.x, slip_mode.pos.y, slip_mode.pos.z);
	//printf("acc %f %f %f\n", slip_mode.acc.x, slip_mode.acc.y, slip_mode.acc.z);
}


//-------------------------------------------------jerk trajectory planner------------------------------------------
_TRA traj[10];
void GenerateTrajectory(float  p0, float v0, float a0, float pf, float vf, float af, float Tf, char defined[3], float*a, float*b, float*g, float *cost) {
	char accGoalDefined = defined[0];
	char posGoalDefined = defined[1];
	char velGoalDefined = defined[2];
	//define starting position:
	float  delta_a = af - a0;
	float  delta_v = vf - v0 - a0 * Tf;
	float  delta_p = pf - p0 - v0 * Tf - 0.5*a0*Tf*Tf;

	// %powers of the end time:
	float  T2 = Tf * Tf;
	float  T3 = T2 * Tf;
	float  T4 = T3 * Tf;
	float  T5 = T4 * Tf;

	//%solve the trajectories, depending on what's constrained:
	if (posGoalDefined && velGoalDefined && accGoalDefined)
	{
		*a = (60 * T2*delta_a - 360 * Tf*delta_v + 720 * 1 * delta_p) / T5;
		*b = (-24 * T3*delta_a + 168 * T2*delta_v - 360 * Tf*delta_p) / T5;
		*g = (3 * T4*delta_a - 24 * T3*delta_v + 60 * T2*delta_p) / T5;
	}
	else if (posGoalDefined && velGoalDefined)
	{
		*a = (-120 * Tf*delta_v + 320 * delta_p) / T5;
		*b = (72 * T2*delta_v - 200 * Tf*delta_p) / T5;
		*g = (-12 * T3*delta_v + 40 * T2*delta_p) / T5;
	}
	else if (posGoalDefined && accGoalDefined)
	{
		*a = (-15 * T2*delta_a + 90 * delta_p) / (2 * T5);
		*b = (15 * T3*delta_a - 90 * Tf*delta_p) / (2 * T5);
		*g = (-3 * T4*delta_a + 30 * T2*delta_p) / (2 * T5);
	}
	else if (velGoalDefined && accGoalDefined)
	{
		*a = 0;
		*b = (6 * Tf*delta_a - 12 * delta_v) / T3;
		*g = (-2 * T2*delta_a + 6 * Tf*delta_v) / T3;
	}
	else if (posGoalDefined)

	{
		*a = 20 * delta_p / T5;
		*b = -20 * delta_p / T4;
		*g = 10 * delta_p / T3;
	}
	else if (velGoalDefined)
	{
		*a = 0;
		*b = -3 * delta_v / T3;
		*g = 3 * delta_v / T2;
	}
	else if (accGoalDefined)
	{
		*a = 0;
		*b = 0;
		*g = delta_a / Tf;
	}
	else {
		//%Nothing to do!
		*a = 0;
		*b = 0;
		*g = 0;
	}

	//%Calculate the cost:
	*cost = *g* *g + *b* *g*Tf + *b* *b*T2 / 3.0 + *a* *g*T2 / 3.0 + *a* *b*T3 / 4.0 + *a* *a*T4 / 20.0;
}

void get_trajecotry(float p0, float v0, float a0, float a, float b, float g, float t, float *pos, float *spd, float *acc, float *jerk) {
	*pos = p0 + v0 * t + (1 / 2.0)*a0*t*t + (1 / 6.0)*g*t*t*t + (1 / 24.0)*b*t*t*t*t + (1 / 120.0)*a*t*t*t*t*t;
	*jerk = g + b * t + (1 / 2.0)*a*t*t;
	*acc = a0 + g * t + (1 / 2.0)*b*t*t + (1 / 6.0)*a*t*t*t;
	*spd = v0 + a0 * t + (1 / 2.0)*g*t*t + (1 / 6.0)*b*t*t*t + (1 / 24.0)*a*t*t*t*t;
}


void plan_tra(_TRA *tra)
{
	char i, j;
	float cost[3];
	for (i = 0; i < 3; i++)
	{
		GenerateTrajectory(tra->ps[i], tra->vs[i], tra->as[i], tra->pe[i], tra->ve[i], tra->ae[i], tra->Time,
			tra->defined, &tra->param[i * 3 + 0], &tra->param[i * 3 + 1], &tra->param[i * 3 + 2], &cost[i]);
	}
	tra->cost_all = cost[0] + cost[1] + cost[2];
}

void get_tra(_TRA *tra, float t)
{
	char i;
	float acc, jerk;
	for (i = 0; i < 3; i++)
		get_trajecotry(tra->ps[i], tra->vs[i], tra->as[i],
			tra->param[i * 3 + 0], tra->param[i * 3 + 1], tra->param[i * 3 + 2]
			, t,
			&tra->pt[i], &tra->vt[i], &acc, &jerk);
}


//--------------------------------步态规划
int swing_jerk_planner_2d_3point(VMC* in, float lift_spd, float td_spd, float mid_spd_w, float T_sw)
{
	static char init[4];
	char flag[3];
	char i;
	int spd_flag[2] = { 1,1 };
	float cost;
	float spd_av = 0;

	in->param.end_planner.Tsw = T_sw;
	in->param.end_planner.leg_h = in->delta_ht;//抬腿高度
	in->param.end_planner.leg_dis = sqrtf(pow(in->st_pos.x - in->tar_pos.x, 2)
		+ pow(in->st_pos.y - in->tar_pos.y, 2)
		+ pow(in->st_pos.z - in->tar_pos.z, 2));
	if (in->tar_pos.x > in->st_pos.x)
		spd_flag[Xr] = 1;
	else
		spd_flag[Xr] = -1;

	in->param.end_planner.max_spd = in->param.end_planner.leg_dis / in->param.end_planner.Tsw;

	in->param.end_planner.T12 = in->param.end_planner.Tsw*0.5;
	in->param.end_planner.T23 = in->param.end_planner.Tsw - in->param.end_planner.T12;


	//%1 start
	in->param.end_planner.p1[0] = in->st_pos.x;
	in->param.end_planner.p1[1] = in->st_pos.y;
	in->param.end_planner.p1[2] = in->st_pos.z;

	in->param.end_planner.v1[0] = in->spd.x;
	in->param.end_planner.v1[1] = 0;
	in->param.end_planner.v1[2] = lift_spd;

	//%2 ------------------------mid--------------------------
	in->param.end_planner.p2[0] = in->st_pos.x / 2 + in->tar_pos.x / 2;
	in->param.end_planner.p2[1] = in->st_pos.y / 2 + in->tar_pos.y / 2;
	in->param.end_planner.p2[2] = in->st_pos.z / 2 + in->tar_pos.z / 2 + in->param.end_planner.leg_h;//中点高度
	in->param.end_planner.v2[0] = spd_flag[Xr] * in->param.end_planner.max_spd*mid_spd_w;
	in->param.end_planner.v2[1] = 0;
	in->param.end_planner.v2[2] = 0;

	//%3 end
	in->param.end_planner.p3[0] = in->tar_pos.x;
	in->param.end_planner.p3[1] = in->tar_pos.y;
	in->param.end_planner.p3[2] = in->tar_pos.z;
	in->param.end_planner.v3[0] = in->spd.x;
	in->param.end_planner.v3[1] = 0;
	in->param.end_planner.v3[2] = -td_spd;

	flag[0] = 1;
	flag[1] = 1;//SPD  中点速度加速度连续
	flag[2] = 0;
	for (i = 0; i < 3; i++) {
		GenerateTrajectory(
			in->param.end_planner.p1[i],
			in->param.end_planner.v1[i],
			in->param.end_planner.a1[i],
			in->param.end_planner.p2[i],
			in->param.end_planner.v2[i],
			in->param.end_planner.a2[i],
			in->param.end_planner.T12, flag,
			&in->param.end_planner.a12[i],
			&in->param.end_planner.b12[i],
			&in->param.end_planner.g12[i], &cost);
	}

	flag[0] = 1;
	flag[1] = 1;
	flag[2] = 0;
	for (i = 0; i < 3; i++) {
		GenerateTrajectory(
			in->param.end_planner.p2[i],
			in->param.end_planner.v2[i],
			in->param.end_planner.a2[i],
			in->param.end_planner.p3[i],
			in->param.end_planner.v3[i],
			in->param.end_planner.a3[i],
			in->param.end_planner.T23, flag,
			&in->param.end_planner.a23[i],
			&in->param.end_planner.b23[i],
			&in->param.end_planner.g23[i], &cost);
	}
}

int get_swing_jerk_2d_3point(VMC* in, float time_now) {
	float time = time_now;
	if (time_now != 0)
		time = time_now;
	time = LIMIT(time, 0, in->param.end_planner.Tsw);
	float pos, acc, spd, jerk;
	char i = 0;
	for (i = 0; i < 3; i++) {
		if (time <= in->param.end_planner.T12)
			get_trajecotry(
				in->param.end_planner.p1[i],
				in->param.end_planner.v1[i],
				in->param.end_planner.a1[i],
				in->param.end_planner.a12[i],
				in->param.end_planner.b12[i],
				in->param.end_planner.g12[i],
				time,
				&pos, &acc, &spd, &jerk);
		else
			get_trajecotry(
				in->param.end_planner.p2[i],
				in->param.end_planner.v2[i],
				in->param.end_planner.a2[i],
				in->param.end_planner.a23[i],
				in->param.end_planner.b23[i],
				in->param.end_planner.g23[i],
				time - in->param.end_planner.T12,
				&pos, &acc, &spd, &jerk);

		in->param.end_planner.pos_now[i] = pos;
		in->param.end_planner.spd_now[i] = spd;
		in->param.end_planner.acc_now[i] = acc;
	}
	return 1;
}



//---------------------------------------------多项式--------------------------------------
static float c0[4][3], c3[4][3], c4[4][3], c5[4][3], c6[4][3];
float swing_x_pos_weight = 0.5;
void cal_curve_from_pos_new(VMC *in, END_POS t_pos, float sw_z, float desire_time)
{
	char id = in->param.id;
	float pos_now[3], pos_tar[3];
	pos_now[Xr] = in->st_pos.x; pos_now[Yr] = in->st_pos.y; pos_now[Zr] = in->st_pos.z;
	pos_tar[Xr] = t_pos.x; pos_tar[Yr] = t_pos.y; pos_tar[Zr] = t_pos.z;
	float t1 = desire_time / 2, t2 = desire_time;
	float p0[3], p1[3], p2[3];
	//----------start
	p0[0] = pos_now[Xr]; p0[1] = pos_now[Yr]; p0[2] = pos_now[Zr];
	//--------------end
	p2[0] = pos_tar[Xr]; p2[1] = pos_tar[Yr]; p2[2] = pos_tar[Zr];//z
	//-------------middle
	float k, b;
	float end_weight = swing_x_pos_weight;
	float att_off = -sind(vmc_all.att_rate_trig[PITr]);

	p1[0] = p0[0] * (1 - end_weight) + p2[0] * end_weight;	//3
	p1[1] = p0[1] * (1 - end_weight) + p2[1] * end_weight;	//3

	p1[2] = p0[2] / 2 + p2[2] / 2 + sw_z;
	// p1[2] = LIMIT(p1[2], MAX_Z, MIN_Z);
	float p1_p0[3], p0_p2[3];
	int i;
	for (i = 0; i < 3; i++)
	{
		c0[id][i] = p0[i]; p1_p0[i] = p1[i] - p0[i]; p0_p2[i] = p0[i] - p2[i];
	}
	float t1_3 = powf(t1, 3), t1_4 = powf(t1, 4), t1_5 = powf(t1, 5), t1_6 = powf(t1, 6);
	float t2_2 = powf(t2, 2), t2_3 = powf(t2, 3), t2_5 = powf(t2, 5), t2_6 = powf(t2, 6);
	float temp1 = 0; temp1 = 1 / (t1_3*powf((t1 - t2), 3)*t2_3);
	for (i = 0; i < 3; i++) {
		c3[id][i] = -1 * temp1*(t2_6*(p1_p0[i]) + 5 * t1_4*t2_2 * 3 * (p0_p2[i]) + 2 * t1_6 * 5 * (p0_p2[i]) - 3 * t1_5*t2 * 8 * (p0_p2[i]));
		c4[id][i] = temp1 / t2 * (3 * t2_6*(p1_p0[i]) + 15 * t1_3*t2_3*(p0_p2[i]) - 27 * t1_5*t2*(p0_p2[i]) + t1_6 * 15 * (p0_p2[i]));
		c5[id][i] = -temp1 / t2_2 * 3 * (t2_6*(p1_p0[i]) + 2 * t1_6*(p0_p2[i]) + t1_3 * t2_3 * 8 * (p0_p2[i]) - t1_4 * t2_2 * 9 * (p0_p2[i]));
		c6[id][i] = temp1 / t2_2 * (t2_5*(p1_p0[i]) + 6 * t1_5*(p0_p2[i]) + 10 * t1_3*t2_2*(p0_p2[i]) - t1_4 * t2 * 15 * (p0_p2[i]));
	}
}

END_POS cal_pos_tar_from_curve(VMC *in, float desire_time, float dt)
{
	char id = in->param.id;
	END_POS epos;
	float cal_curve[3];
	float time_now = LIMIT(in->param.time_trig, 0, desire_time);
	char i;
	for (i = 0; i < 3; i++)
		cal_curve[i] = c0[id][i] + c3[id][i] * powf(time_now, 3) + c4[id][i] * powf(time_now, 4) + c5[id][i] * powf(time_now, 5) + c6[id][i] * powf(time_now, 6);
	epos.x = cal_curve[Xr];
	epos.y = cal_curve[Yr];
	epos.z = cal_curve[Zr];
	return epos;
}

//--------------------------------------5次多项式----------------------------------
void traj_plan5(float T_traj, float P_st, float V_st, float P_ed, float V_ed, float Acc_st, float Acc_ed, float *param) {
	//%时间
	float t0 = 0;
	float t1 = T_traj;
	//%位置和速度（a）
	float q0 = P_st;
	float q1 = P_ed;
	float v0 = V_st;
	float v1 = V_ed;
	float acc0 = Acc_st;
	float acc1 = Acc_ed;
	//%利用公式（1-25）求系数
	float h = q1 - q0;
	float T = t1 - t0;
	param[1 - 1] = q0;
	param[2 - 1] = v0;
	param[3 - 1] = 1.0 / 2 * acc0;
	param[4 - 1] = 1.0 / (2 * T*T*T)*(20 * h - (8 * v1 + 12 * v0)*T + (acc1 - 3 * acc0) / (T*T));
	param[5 - 1] = 1.0 / (2 * T*T*T*T)*((-30 * h + (14 * v1 + 16 * v0)*T) + (3 * acc0 - 2 * acc1) / (T*T));
	param[6 - 1] = 1.0 / (2 * T*T*T*T*T)*(12 * h - 6 * (v1 + v0)*T + (acc1 - acc0) / (T*T));
	//printf("v0=%f v1=%f\n",v0,v1);
}


void get_traj5(float param[6], float t_now, float *q, float *v, float *acc) {
	//%轨迹生成
	float t0 = 0;
	float t = t_now;

	//%位置
	*q = param[1 - 1] + param[2 - 1] * powf((t - t0), 1) + param[3 - 1] * powf((t - t0), 2) + param[4 - 1] * powf((t - t0), 3) + //...
		param[5 - 1] * powf(t - t0, 4) + param[6 - 1] * pow(t - t0, 5);
	//printf("t=%f q=%f %f %f %f %f %f %f\n", t,*q, param[0], param[1], param[2], param[3], param[4], param[5]);
	//%速度
	*v = param[2 - 1] + 2 * param[3 - 1] * powf((t - t0), 1) + 3 * param[4 - 1] * powf((t - t0), 2) + 4 * param[5 - 1] * powf(t - t0, 3) + //...
		5 * param[6 - 1] * powf(t - t0, 4);
	//%加速度
	*acc = 2 * param[3 - 1] + 6 * param[4 - 1] * powf((t - t0), 1) + 12 * param[5 - 1] * powf(t - t0, 2) + 20 * param[6 - 1] * powf(t - t0, 3);
}


int swing_jerk_planner_3point(VMC* in, float lift_spd, float td_spd, float sw_z, float T_sw)
{
	static char init[4];
	char flag[3];
	char i;
	float cost;
	float spd_av = 0;

	in->param.end_planner.Tsw = T_sw;
	in->param.end_planner.leg_h = sw_z;

	//%1 start
	in->param.end_planner.p1[0] = in->st_pos.x;
	in->param.end_planner.p1[1] = in->st_pos.y;
	in->param.end_planner.p1[2] = in->st_pos.z;

	in->param.end_planner.v1[0] = in->st_spd.x ;
	in->param.end_planner.v1[1] = in->st_spd.y ;
	if (lift_spd != 0)
		in->param.end_planner.v1[2] = in->st_spd.z;
	else
		in->param.end_planner.v1[2] = lift_spd;

	//%2 ------------------------mid--------------------------
	in->param.end_planner.p2[0] = in->st_pos.x / 2 + in->tar_pos.x / 2;
	in->param.end_planner.p2[1] = in->st_pos.y / 2 + in->tar_pos.y / 2;
	//in->param.end_planner.p2[2] = in->st_pos.z / 2 + in->tar_pos.z / 2 + in->param.end_planner.leg_h;//中点高度
	in->param.end_planner.p2[2] = in->st_pos.z + in->param.end_planner.leg_h;//中点高度

	//摆动高度限制
	in->param.end_planner.p2[2] = LIMIT(in->param.end_planner.p2[2],
		MAX_Z + in->flag_fb*vmc_all.H / 2 * sind(vmc_all.ground_att_est[PITr]),
		MIN_Z + in->flag_fb*vmc_all.H / 2 * sind(vmc_all.ground_att_est[PITr]));

	in->param.end_planner.v2[0] = 0;
	in->param.end_planner.v2[1] = 0;
	in->param.end_planner.v2[2] = 0;

	//%3 end
	in->param.end_planner.p3[0] = in->tar_pos.x;
	in->param.end_planner.p3[1] = in->tar_pos.y;
	in->param.end_planner.p3[2] = in->tar_pos.z;
	in->param.end_planner.v3[0] = vmc_all.spd_n.x * -1;
	in->param.end_planner.v3[1] = vmc_all.spd_n.y * -1;
	in->param.end_planner.v3[2] = -td_spd;

	//Xr
	traj_plan5(in->param.end_planner.Tsw,//直接V0~V3
		in->param.end_planner.p1[0],
		in->param.end_planner.v1[0],
		in->param.end_planner.p3[0],
		in->param.end_planner.v3[0],
		0,
		0,
		&in->param.end_planner.param_x[0]);

	//Zr
	traj_plan5(in->param.end_planner.Tsw / 2,//1
		in->param.end_planner.p1[2],
		in->param.end_planner.v1[2],
		in->param.end_planner.p2[2],
		0,//摆动中点Z速度为0
		0,
		0,
		&in->param.end_planner.param_z1[0]);

	traj_plan5(in->param.end_planner.Tsw / 2,//2
		in->param.end_planner.p2[2],
		0,//摆动中点Z速度为0
		in->param.end_planner.p3[2],
		in->param.end_planner.v3[2],
		0,
		0,
		&in->param.end_planner.param_z2[0]);
	return 1;
}

int get_swing_jerk_3point(VMC* in, float time_now) {
	float time = time_now;
	if (time_now != 0)
		time = time_now;
	time = LIMIT(time, 0, in->param.end_planner.Tsw);

	get_traj5(in->param.end_planner.param_x, time,
		&in->param.end_planner.pos_now[0],
		&in->param.end_planner.spd_now[0],
		&in->param.end_planner.acc_now[0]);

	if (time <= in->param.end_planner.Tsw / 2)
		get_traj5(in->param.end_planner.param_z1, time,
			&in->param.end_planner.pos_now[2],
			&in->param.end_planner.spd_now[2],
			&in->param.end_planner.acc_now[2]);
	else
		get_traj5(in->param.end_planner.param_z2, time - in->param.end_planner.Tsw / 2,
			&in->param.end_planner.pos_now[2],
			&in->param.end_planner.spd_now[2],
			&in->param.end_planner.acc_now[2]);
	//if(in->param.id==0)
	//	printf("%d %f %f %f\n", in->param.id, time,in->param.end_planner.pos_now[0], in->param.end_planner.pos_now[2]);
	return 1;
}


float foot_trap_thr = 0.23;
int foot_trap_detector(VMC* in,char mode,float dt)
{
	if (mode ==1) {//复位
		in->param.trap_cnt = 0;
		in->param.is_trap = 0;
		in->param.trap_mov_dis[Xr] = in->param.trap_mov_dis[Yr] = in->param.trap_mov_dis[Zr] = in->param.trap_mov_dis[3] = 0;
		in->param.trap_mov_dis_norm[Xr] = in->param.trap_mov_dis_norm[Yr] = in->param.trap_mov_dis_norm[Zr] = in->param.trap_mov_dis_norm[3] = 0;
	}

#if EN_HUMAN
	if (_hu_model.hu_mode != HU_MODE1&& _hu_model.hu_mode != HU_MODE2) {
		in->param.is_trap = 0;
		return 0;
	}
#elif EN_VISION
	in->param.is_trap = 0;
	return 0;
#endif


	float plan_spd_use = vmc_all.param.tar_spd_use_rc.x;//参考速度选择
	//plan_spd_use = vmc_all.spd_n.x;
	float sw_len_norm = fabs(plan_spd_use) * (vmc_all.stance_time + vmc_all.delay_time[2] * 2 + vmc_all.gait_delay_time) / 2;
	float dis = sqrt(powf(vmc[in->param.id].tar_pos.x - vmc[in->param.id].epos_n.x, 2)
		+ powf(vmc[in->param.id].tar_pos.z - vmc[in->param.id].epos_n.z, 2)*0);

	if (in->param.is_trap == 0&& mode==2) {//采集

		//实际运动
		in->param.trap_mov_dis[Xr] += fabs(in->spd_n.x)*dt;
		in->param.trap_mov_dis[Yr] += fabs(in->spd_n.y)*dt;
		in->param.trap_mov_dis[Zr] += fabs(in->spd_n.z)*dt;
		in->param.trap_mov_dis[3]  +=sqrt(pow(in->spd_n.x,2)+ pow(in->spd_n.y, 2)+ pow(in->spd_n.z, 2))*dt;
		//标称运动
		in->param.trap_mov_dis_norm[Xr] += fabs(in->epos_spdd_n.x)*dt;
		in->param.trap_mov_dis_norm[Yr] += fabs(in->epos_spdd_n.y)*dt;
		in->param.trap_mov_dis_norm[Zr] += fabs(in->epos_spdd_n.z)*dt;
		in->param.trap_mov_dis_norm[3] += sqrt(pow(in->epos_spdd_n.x, 2) + pow(in->epos_spdd_n.y, 2) + pow(in->epos_spdd_n.z, 2))*dt;
	}

	float rate1 = 0.8;
    if (mode == 3  )//触碰重规划
	{
		float rate[4];
		if (in->param.id == 2&&0) {
			printf("Leg real [%d]:: x=%f len=%f\n", in->param.id, in->param.trap_mov_dis[Xr],in->param.trap_mov_dis[3]);
			printf("Leg norm [%d]:: x=%f len=%f\n", in->param.id, in->param.trap_mov_dis_norm[Xr],in->param.trap_mov_dis_norm[3]);
			printf("Leg rate [%d]:: x=%f len=%f\n", in->param.id, in->param.trap_mov_dis[Xr]/in->param.trap_mov_dis_norm[Xr], in->param.trap_mov_dis[3]/in->param.trap_mov_dis_norm[3]);
			printf("Force=%f spd_x=%f\n", fabs(robotwb.Leg[in->param.id].force_est_n_output.x), fabs(in->epos_spdd_n.x));
		}
		rate[0] = in->param.trap_mov_dis[Xr] / in->param.trap_mov_dis_norm[Xr];
		rate[3] = in->param.trap_mov_dis[3] / in->param.trap_mov_dis_norm[3];
		if (rate[3] < rate1&&fabs(robotwb.Leg[in->param.id].force_est_n_output.x)>5.5&& fabs(in->epos_spdd_n.x)<0.5)//x移动距离比真实的小
		{
			printf("Leg %d is in Xtrap\n", in->param.id);
			return 1;
		}

		if (in->param.is_trap) {//成功处理了Trap
			in->param.is_trap = 0;
			printf("foot[%d] trap is correct\n", in->param.id);
		}
	}

#if !EN_HUMAN
	if (mode == 3&&
		((plan_spd_use>0&& vmc_all.ground_att_est[PITr]>3)|| (plan_spd_use < 0 && vmc_all.ground_att_est[PITr] <- 3)))//摆动结束判断
	{
		//printf("Leg[%d]:: %f %f dis=%f\n", in->param.id, in->param.trap_mov_dis[Xr],  in->param.trap_mov_dis[Zr], dis);
		if (in->param.trap_mov_dis[Xr] < sw_len_norm*0.2&&0)//x移动距离比真实的小
		{
			printf("Leg %d is in Xtrap\n", in->param.id);
			return 1;
		}
		if (in->param.trap_mov_dis[Zr] < in->param.end_planner.leg_h*2*0.5 && in->param.is_trap==0)//z移动距离比真实的小
		{
			printf("Leg %d is in Ztrap\n", in->param.id);
			return 1;
		}
			
		if (dis < foot_trap_thr*sw_len_norm&&in->param.trap_mov_dis[Xr] < sw_len_norm*0.2&&0) {
			printf("Leg %d is in trap swing\n", in->param.id);
			return 1;
		}

		if (in->param.is_trap) {//成功处理了Trap
			in->param.is_trap = 0;
			printf("foot[%d] trap is correct\n", in->param.id);
		}
	}
#endif

	return 0;
}


float z_upstair_thr = 0.13;//台阶高度
int upstarir_check_num = 1;
int upstarir_keep_duty = 0;
int upstarir_gait_keep_duty = 1;
int upstair_detector(VMC* in, char mode, float dt) {//swing per trig
	char dig_leg = 0;
	char same_leg = 0;
	char dig_same_leg = 0;
	int spd_way = 0;
	if (vmc_all.tar_spd.x >= 0)
		spd_way = 1;
	else
		spd_way = -1;
	//printf("fabs(vmc_all.ground_att_est[PITr])=%f\n", fabs(vmc_all.ground_att_est[PITr]));
    if (fabs(vmc_all.ground_att_est[PITr]) < 8 ) {
		printf("Reset Guass map!!\n");

		in->param.is_up_stair = 0;
     }

#if EN_VISION&&!EN_HUMAN
	return 0;
#endif
	switch (in->param.id) {
		case 0:dig_leg = 2; same_leg = 1; dig_same_leg = 3; break;
		case 1:dig_leg = 3; same_leg = 0; dig_same_leg = 2;break;
		case 2:dig_leg = 0; same_leg = 3; dig_same_leg = 1; break;
		case 3:dig_leg = 1; same_leg = 2; dig_same_leg = 0; break;
	}
	//printf("[%d] %d %f %f %f\n", in->param.id, in->param.is_up_stair, in->epos_n.z + z_upstair_thr, vmc[dig_leg].epos_n.z, fabs(vmc_all.ground_att_est[PITr]));
	if (in->param.is_up_stair==0&&
		//fabs(vmc_all.ground_att_est[PITr]) > 5 &&
		in->epos_n.z + z_upstair_thr < vmc[dig_leg].epos_n.z) {//前腿可以后腿不行
		
		if (in->param.upstair_cnt++>= upstarir_check_num||1) {
			in->param.is_up_stair = 1;
			in->param.stair_pos = vmc[dig_leg].epos_n ;
			in->param.upstair_cnt = 0;
			in->param.duty_cnt = 0;
			in->param.upstarir_gait_keep_duty = upstarir_gait_keep_duty ;
			printf("Leg %d is in upstair swing\n", in->param.id);

			if (spd_way == 1 && (in->param.id == 0 || in->param.id == 2)) {//F
				vmc[dig_same_leg].param.is_up_stair = 2;//wait trig
				vmc[dig_same_leg].param.stair_pos = vmc[dig_leg].epos_n;
				vmc[dig_same_leg].param.upstair_cnt = 0;
				vmc[dig_same_leg].param.duty_cnt = 0;
				vmc[dig_same_leg].param.upstarir_gait_keep_duty = upstarir_gait_keep_duty ;

				vmc[dig_same_leg].param.pos_stair.x = vmc_all.pos_n.x;
				printf("**Back Side Leg is in upstair swing=%f\n", vmc[dig_same_leg].param.pos_stair.x);
			}			
			if (spd_way == -1 && (in->param.id == 1 || in->param.id == 3)) {//H
				vmc[dig_same_leg].param.is_up_stair = 2;//wait trig
				vmc[dig_same_leg].param.stair_pos = vmc[dig_leg].epos_n;
				vmc[dig_same_leg].param.upstair_cnt = 0;
				vmc[dig_same_leg].param.duty_cnt = 0;
				vmc[dig_same_leg].param.upstarir_gait_keep_duty = upstarir_gait_keep_duty;

				vmc[dig_same_leg].param.pos_stair.x = vmc_all.pos_n.x;
				printf("**Back Side Leg is in upstair swing=%f\n", vmc[dig_same_leg].param.pos_stair.x);
				printf("**Front Side Leg is in upstair swing\n");
			}
			return in->param.is_up_stair;
		}
	}
	else
		in->param.upstair_cnt=0;


	if (in->param.is_up_stair==1) {//单腿持续周期


		if (in->param.duty_cnt >= in->param.upstarir_gait_keep_duty) {
			in->param.duty_cnt = 0;
			in->param.is_up_stair = 0;
			printf("Leg %d is in upstair Finish\n", in->param.id);

		}
		in->param.duty_cnt++;
	}

	if (in->param.is_up_stair == 2&&0) {//单腿持续周期
		printf("vmc_all.pos_n.x=%f in->param.pos_stair.x=%f\n", vmc_all.pos_n.x, in->param.pos_stair.x);
		if ((vmc_all.pos_n.x - in->param.pos_stair.x) > -spd_way*vmc_all.H / 2 * 1) {
			in->param.is_up_stair = 1;
			in->param.upstair_cnt = 0;
			in->param.duty_cnt = 0;
			printf("Leg %d is in upstair delay-swing\n", in->param.id);
		}
	}
	return in->param.is_up_stair;
}

//5点Jerk
float lift_td_t_rate = 0.35;//0.25  <0.5
float lift_td_t_rateb = 0.35;
float lift_td_disx_rate[2] = { -0.1,-0.1 };
float lift_td_disz_rate[2] = { 0.2,0.2 };
float lift_td_disx_rateb[2] = { -0.1,-0.1 };
float lift_td_disz_rateb[2] = { 0.2,0.2 };
#if 1
int swing_jerk_planner_5point(VMC* in, float lift_spd, float td_spd, float sw_z, float T_sw)
{
	static char init[4];
	char flag[3];
	char i;
	float spd_flag[2] = { 1,1 };
	float cost;
	float spd_av = 0;
	float traj_t[5];
	float dtraj_t[5];
	float spd_average;
	float plan_spd_use[2] = { vmc_all.param.tar_spd_use_rc.x,vmc_all.param.tar_spd_use_rc.y };
	float lift_td_t_rate_use = 0.25;//0.25  <0.5
	float lift_td_disx_rate_use[2] = { 0.2,0.2 };
	float lift_td_disz_rate_use[2] = { 0.4,0.4 };

	plan_spd_use[Xr] = vmc_all.spd_n.x;
	plan_spd_use[Yr] = vmc_all.spd_n.y;
	if (plan_spd_use[Xr] < 0)//后退
		spd_flag[Xr] = -1;
	if (plan_spd_use[Yr] < 0)//后退
		spd_flag[Yr] = -1;
	spd_flag[Xr] = sign(in->tar_pos.x - in->st_pos.x);
	spd_flag[Yr] = sign(in->tar_pos.y - in->st_pos.y);

	float sw_len_norm_x = fabs(plan_spd_use[Xr]) * (vmc_all.stance_time + vmc_all.delay_time[2]) / 2;
	float sw_len_tar_x = fabs(plan_spd_use[Xr]) * (vmc_all.stance_time + vmc_all.delay_time[2]) / 2;
	float sw_len_norm_y = fabs(plan_spd_use[Yr]) * (vmc_all.stance_time + vmc_all.delay_time[2]) / 2;
	float sw_len_tar_y = fabs(plan_spd_use[Yr]) * (vmc_all.stance_time + vmc_all.delay_time[2]) / 2;

	//sw_len_norm_x = fabs(in->tar_pos.x - in->st_pos.x);//加上迈不到
	//sw_len_norm_y = fabs(in->tar_pos.y - in->st_pos.y);
	if (in->param.is_trap) {//足端陷阱处理
          {
			lift_td_t_rate_use = 0.4;//0.25  <0.5
			lift_td_disx_rate_use[0] = -0.135;
			lift_td_disz_rate_use[0] = 0.3;
			lift_td_disx_rate_use[1] = -0.12;
			lift_td_disz_rate_use[1] = lift_td_disz_rate[1];
			in->param.end_planner.Tsw = T_sw;
			in->param.end_planner.leg_h = sw_z;
		}
	}
	else {
		
		 if (spd_flag[Xr]==1) {
			 lift_td_t_rate_use = lift_td_t_rate;//0.25  <0.5
			 lift_td_disx_rate_use[0] = lift_td_disx_rate[0];
			 lift_td_disz_rate_use[0] = lift_td_disz_rate[0];
			 lift_td_disx_rate_use[1] = lift_td_disx_rate[1];
			 lift_td_disz_rate_use[1] = lift_td_disz_rate[1];
		 }
		 else {
			 lift_td_t_rate_use = lift_td_t_rateb;
			 lift_td_disx_rate_use[0] = lift_td_disx_rateb[0];
			 lift_td_disz_rate_use[0] = lift_td_disz_rateb[0];
			 lift_td_disx_rate_use[1] = lift_td_disx_rateb[1];
			 lift_td_disz_rate_use[1] = lift_td_disz_rateb[1];
		 
		 }
		 in->param.end_planner.Tsw = T_sw;
		 in->param.end_planner.leg_h = sw_z;
		 //printf("sw_z[leg_sel_trig] =%f\n", sw_z);
	}

	float sw_z_traj = in->st_pos.z / 2 + in->tar_pos.z / 2;//摆动中点基准高度
	//if(in->param.vision_sw_enable==1)
		sw_z_traj = in->tar_pos.z;



	traj_t[0] = in->param.end_planner.Tsw / 2 * lift_td_t_rate_use;//起点到后置
	traj_t[1] = in->param.end_planner.Tsw / 2;//后置到中点
	traj_t[2] = traj_t[1] + in->param.end_planner.Tsw / 2 * (1 - lift_td_t_rate_use);//中点到前置
	traj_t[3] = in->param.end_planner.Tsw;//前置到终点

	dtraj_t[0] = in->param.end_planner.Tsw / 2 * lift_td_t_rate_use;//起点到后置
	dtraj_t[1] = in->param.end_planner.Tsw / 2 * (1 - lift_td_t_rate_use);//后置到中点
	dtraj_t[2] = in->param.end_planner.Tsw / 2 * (1 - lift_td_t_rate_use);//中点到前置
	dtraj_t[3] = in->param.end_planner.Tsw / 2 * lift_td_t_rate_use;//前置到终点


	float dx = in->param.tar_epos_n.x - in->st_pos.x;
	float dy = in->param.tar_epos_n.y - in->st_pos.y;
	float dz = in->param.tar_epos_n.z - in->st_pos.z;
	float r_dx, r_dy, r_dz;
	float g_att = -my_deathzoom_2(vmc_all.ground_att_est[PITr] + FIX_SLOP_OFF, SLOP_DEAD);
	float sita_side1 = atan(vmc_all.param.param_vmc.side_off[Yr] /vmc_all.pos_n.z)*57.3;
	float l = sqrt(pow(vmc_all.param.param_vmc.side_off[Yr], 2) + pow(vmc_all.pos_n.z,2));
	float sita_side2 = asin(in->l3 / l)*57.3;
	//printf("sita_side=%f sita_side2=%f\n", sita_side1, sita_side2);
	float y_side = in->param.y_side;// tand(fabs(sita_side2 - sita_side1)*in->flag_rl)*in->param.end_planner.leg_h;//内收造成的偏距
	//%1 start离地点

	in->param.end_planner.p1[0] = in->st_pos.x;
	in->param.end_planner.p1[1] = in->st_pos.y;
	in->param.end_planner.p1[2] = in->st_pos.z;

	in->param.end_planner.v1[0] = in->st_spd.x;
	in->param.end_planner.v1[1] = in->st_spd.y;
	in->param.end_planner.v1[2] = in->st_spd.z;

	//%2 后置点
	in->param.end_planner.p2[0] = in->st_pos.x + -spd_flag[Xr] * sw_len_norm_x/  2 * lift_td_disx_rate_use[0];
	in->param.end_planner.p2[1] = in->st_pos.y + -spd_flag[Yr] * sw_len_norm_y / 2 * lift_td_disx_rate_use[0]+ y_side * lift_td_disz_rate_use[0];
	if(sw_z_traj> in->st_pos.z)
		in->param.end_planner.p2[2] = in->st_pos.z + in->param.end_planner.leg_h * lift_td_disz_rate_use[0];
	else
		in->param.end_planner.p2[2] = sw_z_traj + in->param.end_planner.leg_h * lift_td_disz_rate_use[0];

	spd_average = (sw_z * lift_td_disz_rate_use[0]) / dtraj_t[0];

	in->param.end_planner.v2[0] = 0;
	in->param.end_planner.v2[1] = 0;
	in->param.end_planner.v2[2] = spd_average;

#if EN_SW_SLOP_ETH //转地形坐标ETH  地形转换导致下台阶抬不起腿
	dx = in->param.end_planner.p2[0] - in->st_pos.x;
	dz = in->param.end_planner.p2[2] - in->st_pos.z;
	r_dx = cosd(g_att)*dx - sind(g_att)*dz;
	r_dz = sind(g_att)*dx + cosd(g_att)*dz;
	if ((g_att > 0 && vmc_all.param.tar_spd_use_rc.x > 0) || 0) {
		in->param.end_planner.p2[0] = in->st_pos.x + r_dx;
		in->param.end_planner.p2[2] = in->st_pos.x + r_dz;
	}
#endif

	//%3 ------------------------中点-------------------------
	float rate_mid = 0.5;
	in->param.end_planner.p3[0] = in->st_pos.x*(1- rate_mid ) + in->tar_pos.x*rate_mid;
	in->param.end_planner.p3[1] = in->st_pos.y *(1 - rate_mid) + in->tar_pos.y *rate_mid + y_side;
	in->param.end_planner.p3[2] =  sw_z_traj + in->param.end_planner.leg_h;//中点高度

#if EN_SW_SLOP_ETH //转地形坐标ETH
	dx = in->param.end_planner.p3[0] - in->st_pos.x;
	dz = in->param.end_planner.p3[2] - in->st_pos.z;
	r_dx = cosd(g_att)*dx - sind(g_att)*dz;
	r_dz = sind(g_att)*dx + cosd(g_att)*dz;
	if ((g_att > 0 && vmc_all.param.tar_spd_use_rc.x > 0) || 0) {
		in->param.end_planner.p3[0] = in->st_pos.x + r_dx;
		in->param.end_planner.p3[2] = in->st_pos.x + r_dz;
	}
#endif

	//摆动高度限制
	in->param.end_planner.p3[2] = LIMIT(in->param.end_planner.p3[2],
		MAX_Z + in->flag_fb*vmc_all.H / 2 * sind(vmc_all.ground_att_est[PITr]),
		MIN_Z + in->flag_fb*vmc_all.H / 2 * sind(vmc_all.ground_att_est[PITr]));

	in->param.end_planner.v3[0] = spd_flag[Xr] * sw_len_norm_x / T_sw * 2;
	in->param.end_planner.v3[1] = spd_flag[Yr] * sw_len_norm_y / T_sw * 2;
	in->param.end_planner.v3[2] = 0;

	//%4 前置点
	in->param.end_planner.p4[0] = in->tar_pos.x + spd_flag[Xr] * sw_len_norm_x / 2 * lift_td_disx_rate_use[1];
	in->param.end_planner.p4[1] = in->tar_pos.y + spd_flag[Yr] * sw_len_norm_y / 2 * lift_td_disx_rate_use[1]+ y_side *lift_td_disz_rate_use[1];
	in->param.end_planner.p4[2] = in->tar_pos.z + in->param.end_planner.leg_h * lift_td_disz_rate_use[1];

	spd_average = (sw_z * lift_td_disz_rate_use[1]) / dtraj_t[3];
	in->param.end_planner.v4[0] = 0;
	in->param.end_planner.v4[1] = 0;
	in->param.end_planner.v4[2] = -spd_average;

#if EN_SW_SLOP_ETH //转地形坐标ETH
	dx = in->param.end_planner.p4[0] - in->st_pos.x;
	dz = in->param.end_planner.p4[2] - in->st_pos.z;
	r_dx = cosd(g_att)*dx - sind(g_att)*dz;
	r_dz = sind(g_att)*dx + cosd(g_att)*dz;
	if ((g_att > 0 && vmc_all.param.tar_spd_use_rc.x > 0) || 0) {
		in->param.end_planner.p4[0] = in->st_pos.x + r_dx;
		in->param.end_planner.p4[2] = in->st_pos.x + r_dz;
	}
#endif

	//%5 终点
	in->param.end_planner.p5[0] = in->tar_pos.x;
	in->param.end_planner.p5[1] = in->tar_pos.y;
	in->param.end_planner.p5[2] = in->tar_pos.z;
	in->param.end_planner.v5[0] = 0;
	in->param.end_planner.v5[1] = 0;
	in->param.end_planner.v5[2] = -td_spd;

	//Xr-------------------
	traj_plan5(dtraj_t[0],//1
		in->param.end_planner.p1[0],
		in->param.end_planner.v1[0],
		in->param.end_planner.p2[0],
		in->param.end_planner.v2[0],
		0,
		0,
		&in->param.end_planner.param_traj_x[0][0]);

	traj_plan5(dtraj_t[1],//2
		in->param.end_planner.p2[0],
		in->param.end_planner.v2[0],
		in->param.end_planner.p3[0],
		in->param.end_planner.v3[0],
		0,
		0,
		&in->param.end_planner.param_traj_x[1][0]);

	traj_plan5(dtraj_t[2],//3
		in->param.end_planner.p3[0],
		in->param.end_planner.v3[0],
		in->param.end_planner.p4[0],
		in->param.end_planner.v4[0],
		0,
		0,
		&in->param.end_planner.param_traj_x[2][0]);

	traj_plan5(dtraj_t[3],//4
		in->param.end_planner.p4[0],
		in->param.end_planner.v4[0],
		in->param.end_planner.p5[0],
		in->param.end_planner.v5[0],
		0,
		0,
		&in->param.end_planner.param_traj_x[3][0]);

	//Yr----------------------------
	traj_plan5(dtraj_t[0],//1
		in->param.end_planner.p1[1],
		in->param.end_planner.v1[1],
		in->param.end_planner.p2[1],
		in->param.end_planner.v2[1],
		0,
		0,
		&in->param.end_planner.param_traj_y[0][0]);

	traj_plan5(dtraj_t[1],//2
		in->param.end_planner.p2[1],
		in->param.end_planner.v2[1],
		in->param.end_planner.p3[1],
		in->param.end_planner.v3[1],
		0,
		0,
		&in->param.end_planner.param_traj_y[1][0]);

	traj_plan5(dtraj_t[2],//3
		in->param.end_planner.p3[1],
		in->param.end_planner.v3[1],
		in->param.end_planner.p4[1],
		in->param.end_planner.v4[1],
		0,
		0,
		&in->param.end_planner.param_traj_y[2][0]);

	traj_plan5(dtraj_t[3],//4
		in->param.end_planner.p4[1],
		in->param.end_planner.v4[1],
		in->param.end_planner.p5[1],
		in->param.end_planner.v5[1],
		0,
		0,
		&in->param.end_planner.param_traj_y[3][0]);

	//Zr----------------------------------
	traj_plan5(dtraj_t[0],//1
		in->param.end_planner.p1[2],
		in->param.end_planner.v1[2],
		in->param.end_planner.p2[2],
		in->param.end_planner.v2[2],
		0,
		0,
		&in->param.end_planner.param_traj_z[0][0]);

	traj_plan5(dtraj_t[1],//2
		in->param.end_planner.p2[2],
		in->param.end_planner.v2[2],
		in->param.end_planner.p3[2],
		in->param.end_planner.v3[2],
		0,
		0,
		&in->param.end_planner.param_traj_z[1][0]);

	traj_plan5(dtraj_t[2],//3
		in->param.end_planner.p3[2],
		in->param.end_planner.v3[2],
		in->param.end_planner.p4[2],
		in->param.end_planner.v4[2],
		0,
		0,
		&in->param.end_planner.param_traj_z[2][0]);

	traj_plan5(dtraj_t[3],//4
		in->param.end_planner.p4[2],
		in->param.end_planner.v4[2],
		in->param.end_planner.p5[2],
		in->param.end_planner.v5[2],
		0,
		0,
		&in->param.end_planner.param_traj_z[3][0]);
	return 1;
}
#else
int swing_jerk_planner_5point(VMC* in, float lift_spd, float td_spd, float sw_z, float T_sw)
{
	static char init[4];
	char flag[3];
	char i;
	float spd_flag[2] = { 1,1 };
	float cost;
	float spd_av = 0;
	float traj_t[5];
	float dtraj_t[5];
	float spd_average;
	float plan_spd_use[2] = { vmc_all.param.tar_spd_use_rc.x,vmc_all.param.tar_spd_use_rc.y };
	float lift_td_t_rate_use = 0.25;//0.25  <0.5
	float lift_td_disx_rate_use[2] = { 0.2,0.2 };
	float lift_td_disz_rate_use[2] = { 0.4,0.4 };

	plan_spd_use[Xr] = vmc_all.spd_n.x;
	plan_spd_use[Yr] = vmc_all.spd_n.y;
	float sw_len_norm_x = fabs(plan_spd_use[Xr]) * (vmc_all.stance_time + vmc_all.delay_time[2]) / 2;
	float sw_len_tar_x = fabs(plan_spd_use[Xr]) * (vmc_all.stance_time + vmc_all.delay_time[2]) / 2;
	float sw_len_norm_y = fabs(plan_spd_use[Yr]) * (vmc_all.stance_time + vmc_all.delay_time[2]) / 2;
	float sw_len_tar_y = fabs(plan_spd_use[Yr]) * (vmc_all.stance_time + vmc_all.delay_time[2]) / 2;

	if (in->param.is_trap) {//足端陷阱处理
		if (_hu_model.hu_mode == HU_MODE1 || _hu_model.hu_mode == HU_MODE2) {
			//printf("1\n");
			lift_td_t_rate_use = 0.22;//0.25  <0.5
			lift_td_disx_rate_use[0] = 0.3;
			lift_td_disz_rate_use[0] = 0.4;//
			lift_td_disx_rate_use[1] = 0.2;
			lift_td_disz_rate_use[1] = lift_td_disz_rate[1];
			in->param.end_planner.Tsw = T_sw;
			in->param.end_planner.leg_h = sw_z + 0.15;
		}
		else {
			lift_td_t_rate_use = 0.4;//0.25  <0.5
			lift_td_disx_rate_use[0] = 0.35;
			lift_td_disz_rate_use[0] = 0.3;
			lift_td_disx_rate_use[1] = 0.2;
			lift_td_disz_rate_use[1] = lift_td_disz_rate[1];
			in->param.end_planner.Tsw = T_sw;
			in->param.end_planner.leg_h = sw_z;
		}
	}
	else {
		lift_td_t_rate_use = lift_td_t_rate;//0.25  <0.5
		lift_td_disx_rate_use[0] = lift_td_disx_rate[0];
		lift_td_disz_rate_use[0] = lift_td_disz_rate[0];
		lift_td_disx_rate_use[1] = lift_td_disx_rate[1];
		lift_td_disz_rate_use[1] = lift_td_disz_rate[1];
		in->param.end_planner.Tsw = T_sw;
		in->param.end_planner.leg_h = sw_z;
		//printf("sw_z[leg_sel_trig] =%f\n", sw_z);
	}

	float sw_z_traj = in->st_pos.z / 2 + in->tar_pos.z / 2;//摆动中点基准高度
	//if(in->param.vision_sw_enable==1)
	//	sw_z_traj = in->tar_pos.z;

	if (plan_spd_use[Xr] < 0)//后退
		spd_flag[Xr] = -1;
	if (plan_spd_use[Yr] < 0)//后退
		spd_flag[Yr] = -1;
	spd_flag[Xr] = sign(in->tar_pos.x - in->st_pos.x);
	spd_flag[Yr] = sign(in->tar_pos.y - in->st_pos.y);

	float dx = in->param.tar_epos_n.x - in->st_pos.x;
	float dy = in->param.tar_epos_n.y - in->st_pos.y;
	float dz = in->param.tar_epos_n.z - in->st_pos.z;
	float r_dx, r_dy, r_dz;
	float g_att = -my_deathzoom_2(vmc_all.ground_att_est[PITr] + FIX_SLOP_OFF, SLOP_DEAD);
	float sita_side1 = atan(vmc_all.param.param_vmc.side_off[Yr] / vmc_all.pos_n.z)*57.3;
	float l = sqrt(pow(vmc_all.param.param_vmc.side_off[Yr], 2) + pow(vmc_all.pos_n.z, 2));
	float sita_side2 = asin(in->l3 / l)*57.3;
	//printf("sita_side=%f sita_side2=%f\n", sita_side1, sita_side2);
	float y_side = tand(fabs(sita_side2 - sita_side1)*in->flag_rl)*in->param.end_planner.leg_h;//内收造成的偏距

	traj_t[0] = in->param.end_planner.Tsw / 2;//起点到后置
	traj_t[1] = in->param.end_planner.Tsw;//后置到中点

	dtraj_t[0] = in->param.end_planner.Tsw / 2;
	dtraj_t[1] = in->param.end_planner.Tsw / 2;

	int tri_curve = 0;
	float mix_x = 0;
	float tri_rate = 0.9;
	float min_x_strie = 0.2;
	float dead_x_strie = 0.2;
	float strie_x = in->tar_pos.x - in->st_pos.x;

	if (((in->param.id == 0 || in->param.id == 2) && strie_x > -dead_x_strie*MAX_X)
		|| ((in->param.id == 1 || in->param.id == 3) && strie_x < dead_x_strie*MAX_X)) {//向机体外侧  采用anymal三角轨迹
		tri_curve = 1;
        mix_x = in->tar_pos.x*tri_rate + in->st_pos.x*(1 - tri_rate);
		if ((in->param.id == 0 || in->param.id == 2)) {
			//printf("mix_x1=%f in->tar_pos.x=%f in->st_pos.x=%f\n", mix_x, in->tar_pos.x,in->st_pos.x);
			if (fabs(mix_x - in->st_pos.x) < min_x_strie * MAX_X) {
				mix_x = in->st_pos.x + min_x_strie * MAX_X;
				tri_curve = 2;
				//printf("sn\n");
			}
			//printf("mix_x2=%f in->st_pos.x=%f\n", mix_x,in->st_pos.x);
		}
		else {
			if (fabs(mix_x - in->st_pos.x) < min_x_strie * MAX_X) {
				tri_curve = 2;
				mix_x = in->st_pos.x - min_x_strie * MAX_X;
			}
		}
	}

	//%1 start离地点

	in->param.end_planner.p1[0] = in->st_pos.x;
	in->param.end_planner.p1[1] = in->st_pos.y;
	in->param.end_planner.p1[2] = in->st_pos.z;

	in->param.end_planner.v1[0] = in->st_spd.x;
	in->param.end_planner.v1[1] = in->st_spd.y;
	in->param.end_planner.v1[2] = lift_spd;
	if (tri_curve) {
		//%3 ------------------------中点-------------------------
		in->param.end_planner.p2[0] = mix_x;// in->st_pos.x / 2 + in->tar_pos.x / 2;
		in->param.end_planner.p2[1] = in->st_pos.y / 2 + in->tar_pos.y / 2 + y_side;
		in->param.end_planner.p2[2] = sw_z_traj + in->param.end_planner.leg_h;//中点高度

		//摆动高度限制
		in->param.end_planner.p2[2] = LIMIT(in->param.end_planner.p2[2],
			MAX_Z + in->flag_fb*vmc_all.H / 2 * sind(vmc_all.ground_att_est[PITr]),
			MIN_Z + in->flag_fb*vmc_all.H / 2 * sind(vmc_all.ground_att_est[PITr]));

		//in->param.end_planner.v2[0] = -in->st_spd.x;
		//in->param.end_planner.v2[1] = -in->st_spd.y;
		if (tri_curve == 2) {
			in->param.end_planner.v2[0] = spd_flag[Xr] * sw_len_norm_x / T_sw * 0;
			in->param.end_planner.v2[1] = spd_flag[Yr] * sw_len_norm_y / T_sw * 0;
			in->param.end_planner.v2[2] = 0;
		}
		else {
			in->param.end_planner.v2[0] = spd_flag[Xr] * sw_len_norm_x / T_sw * 5;
			in->param.end_planner.v2[1] = spd_flag[Yr] * sw_len_norm_y / T_sw * 5;
			in->param.end_planner.v2[2] = 0;
		}
	}else
	{
		//%3 ------------------------中点-------------------------
		in->param.end_planner.p2[0] = in->st_pos.x / 2 + in->tar_pos.x / 2;
		in->param.end_planner.p2[1] = in->st_pos.y / 2 + in->tar_pos.y / 2 + y_side;
		in->param.end_planner.p2[2] = sw_z_traj + in->param.end_planner.leg_h;//中点高度

		//摆动高度限制
		in->param.end_planner.p2[2] = LIMIT(in->param.end_planner.p2[2],
			MAX_Z + in->flag_fb*vmc_all.H / 2 * sind(vmc_all.ground_att_est[PITr]),
			MIN_Z + in->flag_fb*vmc_all.H / 2 * sind(vmc_all.ground_att_est[PITr]));

		//in->param.end_planner.v2[0] = -in->st_spd.x;
		//in->param.end_planner.v2[1] = -in->st_spd.y;
		in->param.end_planner.v2[0] = spd_flag[Xr] * sw_len_norm_x / T_sw * 5;
		in->param.end_planner.v2[1] = spd_flag[Yr] * sw_len_norm_y / T_sw * 5;
		in->param.end_planner.v2[2] = 0;
	}
	//%5 终点
	in->param.end_planner.p3[0] = in->tar_pos.x;
	in->param.end_planner.p3[1] = in->tar_pos.y;
	in->param.end_planner.p3[2] = in->tar_pos.z;
#if 0
	in->param.end_planner.v3[0] = -in->st_spd.x;
	in->param.end_planner.v3[1] = -in->st_spd.y;
#else
	in->param.end_planner.v3[0] = 0;
	in->param.end_planner.v3[1] = 0;
#endif
	in->param.end_planner.v3[2] = -td_spd;

	//Xr-------------------
	traj_plan5(dtraj_t[0],//1
		in->param.end_planner.p1[0],
		in->param.end_planner.v1[0],
		in->param.end_planner.p2[0],
		in->param.end_planner.v2[0],
		0,
		0,
		&in->param.end_planner.param_traj_x[0][0]);


	traj_plan5(dtraj_t[1],//3
		in->param.end_planner.p2[0],
		in->param.end_planner.v2[0],
		in->param.end_planner.p3[0],
		in->param.end_planner.v3[0],
		0,
		0,
		&in->param.end_planner.param_traj_x[1][0]);


	//Yr----------------------------
	traj_plan5(dtraj_t[0],//1
		in->param.end_planner.p1[1],
		in->param.end_planner.v1[1],
		in->param.end_planner.p2[1],
		in->param.end_planner.v2[1],
		0,
		0,
		&in->param.end_planner.param_traj_y[0][0]);

	traj_plan5(dtraj_t[1],//3
		in->param.end_planner.p2[1],
		in->param.end_planner.v2[1],
		in->param.end_planner.p3[1],
		in->param.end_planner.v3[1],
		0,
		0,
		&in->param.end_planner.param_traj_y[1][0]);
	//Zr----------------------------------
	traj_plan5(dtraj_t[0],//1
		in->param.end_planner.p1[2],
		in->param.end_planner.v1[2],
		in->param.end_planner.p2[2],
		in->param.end_planner.v2[2],
		0,
		0,
		&in->param.end_planner.param_traj_z[0][0]);

	traj_plan5(dtraj_t[1],//2
		in->param.end_planner.p2[2],
		in->param.end_planner.v2[2],
		in->param.end_planner.p3[2],
		in->param.end_planner.v3[2],
		0,
		0,
		&in->param.end_planner.param_traj_z[1][0]);
	return 1;
}
#endif
#if 1
int get_swing_jerk_5point(VMC* in, float time_now) {
	float time = time_now;
	float traj_t[5], dtraj_t[5];
	if (time_now != 0)
		time = time_now;
	time = LIMIT(time, 0, in->param.end_planner.Tsw);


	traj_t[0] = in->param.end_planner.Tsw / 2 * lift_td_t_rate;//起点到后置
	traj_t[1] = in->param.end_planner.Tsw / 2;//后置到中点
	traj_t[2] = traj_t[1] + in->param.end_planner.Tsw / 2 * (1 - lift_td_t_rate);//中点到前置
	traj_t[3] = in->param.end_planner.Tsw;//前置到终点

	dtraj_t[0] = in->param.end_planner.Tsw / 2 * lift_td_t_rate;//起点到后置
	dtraj_t[1] = in->param.end_planner.Tsw / 2 * (1 - lift_td_t_rate);//后置到中点
	dtraj_t[2] = in->param.end_planner.Tsw / 2 * (1 - lift_td_t_rate);//中点到前置
	dtraj_t[3] = in->param.end_planner.Tsw / 2 * lift_td_t_rate;//前置到终点

	if (time < traj_t[0]) {
		get_traj5(in->param.end_planner.param_traj_x[0], time,
			&in->param.end_planner.pos_now[0],
			&in->param.end_planner.spd_now[0],
			&in->param.end_planner.acc_now[0]);
		get_traj5(in->param.end_planner.param_traj_y[0], time,
			&in->param.end_planner.pos_now[1],
			&in->param.end_planner.spd_now[1],
			&in->param.end_planner.acc_now[1]);
		get_traj5(in->param.end_planner.param_traj_z[0], time,
			&in->param.end_planner.pos_now[2],
			&in->param.end_planner.spd_now[2],
			&in->param.end_planner.acc_now[2]);

	}else if (time >= traj_t[0]&& time <traj_t[1]) {
		get_traj5(in->param.end_planner.param_traj_x[1], time - traj_t[0],
			&in->param.end_planner.pos_now[0],
			&in->param.end_planner.spd_now[0],
			&in->param.end_planner.acc_now[0]);
		get_traj5(in->param.end_planner.param_traj_y[1], time - traj_t[0],
			&in->param.end_planner.pos_now[1],
			&in->param.end_planner.spd_now[1],
			&in->param.end_planner.acc_now[1]);
		get_traj5(in->param.end_planner.param_traj_z[1], time - traj_t[0],
			&in->param.end_planner.pos_now[2],
			&in->param.end_planner.spd_now[2],
			&in->param.end_planner.acc_now[2]);

	}else if (time >= traj_t[1]&& time <traj_t[2]) {
		get_traj5(in->param.end_planner.param_traj_x[2], time - traj_t[1],
			&in->param.end_planner.pos_now[0],
			&in->param.end_planner.spd_now[0],
			&in->param.end_planner.acc_now[0]);
		get_traj5(in->param.end_planner.param_traj_y[2], time - traj_t[1],
			&in->param.end_planner.pos_now[1],
			&in->param.end_planner.spd_now[1],
			&in->param.end_planner.acc_now[1]);
		get_traj5(in->param.end_planner.param_traj_z[2], time - traj_t[1],
			&in->param.end_planner.pos_now[2],
			&in->param.end_planner.spd_now[2],
			&in->param.end_planner.acc_now[2]);

	}
	else {
		get_traj5(in->param.end_planner.param_traj_x[3], time - traj_t[2],
			&in->param.end_planner.pos_now[0],
			&in->param.end_planner.spd_now[0],
			&in->param.end_planner.acc_now[0]);
		get_traj5(in->param.end_planner.param_traj_y[3], time - traj_t[2],
			&in->param.end_planner.pos_now[1],
			&in->param.end_planner.spd_now[1],
			&in->param.end_planner.acc_now[1]);
		get_traj5(in->param.end_planner.param_traj_z[3], time - traj_t[2],
			&in->param.end_planner.pos_now[2],
			&in->param.end_planner.spd_now[2],
			&in->param.end_planner.acc_now[2]);

	}
	//if(in->param.id==0)
	//	printf("%d %f %f %f\n", in->param.id, time,in->param.end_planner.pos_now[0], in->param.end_planner.pos_now[2]);
	return 1;
}
#else
int get_swing_jerk_5point(VMC* in, float time_now) {
	float time = time_now;
	float traj_t[5], dtraj_t[5];
	if (time_now != 0)
		time = time_now;
	time = LIMIT(time, 0, in->param.end_planner.Tsw);

	traj_t[0] = in->param.end_planner.Tsw / 2;//起点到后置
	traj_t[1] = in->param.end_planner.Tsw ;//后置到中点


	if (time < traj_t[0]) {
		get_traj5(in->param.end_planner.param_traj_x[0], time,
			&in->param.end_planner.pos_now[0],
			&in->param.end_planner.spd_now[0],
			&in->param.end_planner.acc_now[0]);
		get_traj5(in->param.end_planner.param_traj_y[0], time,
			&in->param.end_planner.pos_now[1],
			&in->param.end_planner.spd_now[1],
			&in->param.end_planner.acc_now[1]);
		get_traj5(in->param.end_planner.param_traj_z[0], time,
			&in->param.end_planner.pos_now[2],
			&in->param.end_planner.spd_now[2],
			&in->param.end_planner.acc_now[2]);
	}
	else  {
		get_traj5(in->param.end_planner.param_traj_x[1], time - traj_t[0],
			&in->param.end_planner.pos_now[0],
			&in->param.end_planner.spd_now[0],
			&in->param.end_planner.acc_now[0]);
		get_traj5(in->param.end_planner.param_traj_y[1], time - traj_t[0],
			&in->param.end_planner.pos_now[1],
			&in->param.end_planner.spd_now[1],
			&in->param.end_planner.acc_now[1]);
		get_traj5(in->param.end_planner.param_traj_z[1], time - traj_t[0],
			&in->param.end_planner.pos_now[2],
			&in->param.end_planner.spd_now[2],
			&in->param.end_planner.acc_now[2]);

	}
	
	//if(in->param.id==0)
	//	printf("%d %f %f %f\n", in->param.id, time,in->param.end_planner.pos_now[0], in->param.end_planner.pos_now[2]);
	return 1;
}
#endif

//-----------------------MIT Traj
#include "FootSwingTrajectory.h"

int swing_Bezier_planner(VMC* in, float lift_spd, float td_spd, float sw_z,float T_sw)
{
	static char init[4];
	char flag[3];
	char i;
	int spd_flag[2] = { 1,1 };
	float cost;
	float spd_av = 0;

	FootSwingTrajectory_init(&traj_swing[in->param.id]);
	Vect3  _p0;
	_p0.x = in->st_pos.x;
	_p0.y = in->st_pos.y;
	_p0.z = in->st_pos.z;
	setInitialPosition(&traj_swing[in->param.id], _p0);
	Vect3  pf;
	pf.x = in->tar_pos.x;
	pf.y = in->tar_pos.y;
	pf.z = in->tar_pos.z;
	//printf("st=%f %f %f end=%f %f %f\n", _p0.x, _p0.y, _p0.z, pf.x, pf.y, pf.z);
	setFinalPosition(&traj_swing[in->param.id], pf);
	setHeight(&traj_swing[in->param.id], sw_z);
	return 1;
}

int get_swing_Bezier(VMC* in, float t_sw, float time_now) {
	float time = time_now;
	if (time_now != 0)
		time = time_now;
	time = LIMIT(time, 0, t_sw);
	// velocity/acceleration:
	float swing_time = t_sw;
	float ph0;

	ph0 = time / t_sw;// *phasePerSecond;
	computeSwingTrajectoryBezier(&traj_swing[in->param.id], ph0, swing_time);//�ٷֱȺ� �ڶ�ʱ��
	Vect3 p0 = getPosition(&traj_swing[in->param.id]);
	Vect3 v0 = getVelocity(&traj_swing[in->param.id]);
	Vect3 a0 = getAcceleration(&traj_swing[in->param.id]);

	in->param.end_planner.pos_now[0] = p0.x;
	in->param.end_planner.pos_now[1] = p0.y;
	in->param.end_planner.pos_now[2] = p0.z;
	in->param.end_planner.spd_now[0] = v0.x;
	in->param.end_planner.spd_now[1] = v0.y;
	in->param.end_planner.spd_now[2] = v0.z;
	in->param.end_planner.acc_now[0] = a0.x;
	in->param.end_planner.acc_now[1] = a0.y;
	in->param.end_planner.acc_now[2] = a0.z;
	return 1;
}

