#include "include.h"
#include "locomotion_header.h"
#include "math.h"
#include "eso.h"
#include "gait_math.h"
//------------------------------运动学运算库
float FLT_SPD_END = 500;
void ik_hip_protect(void)
{
    float line_y=-0.07;
    float line_x=0.3;
    float max_height=0.5;
    if(robotwb.arm_epos_h_exp.y>line_y&&robotwb.arm_epos_h_exp.x<line_x){
        float dis_x=fabs(robotwb.arm_epos_h_exp.x-line_x);
        float dis_y=fabs(robotwb.arm_epos_h_exp.y-line_y);

        if(dis_x>dis_y)
            robotwb.arm_epos_h_exp.y=line_y;
        else
            robotwb.arm_epos_h_exp.x=line_x;
    }
   // printf("limit=%f %f\n",robotwb.arm_epos_h_exp.x,robotwb.arm_epos_h_exp.y);

   robotwb.arm_base_height_exp=LIMIT(robotwb.arm_base_height_exp,0,max_height);
}

void espd_to_neg_dq(int id, float dt)
{
	float flt_espd = 1;
	float gain_espd = 1;
	float temp[3] = { 0,0,0 };

	temp[0] = (-robotwb.Leg[id].jacobi_inv[0] * robotwb.Leg[id].tar_espd_h.x*vmc[id].param.invert_knee_epos[Xr]
		- robotwb.Leg[id].jacobi_inv[1] * robotwb.Leg[id].tar_espd_h.y//*robotwb.Leg[id].flag_rl
		+ (-robotwb.Leg[id].jacobi_inv[2] * robotwb.Leg[id].tar_espd_h.z))*gain_espd;
	temp[1] = (-robotwb.Leg[id].jacobi_inv[3] * robotwb.Leg[id].tar_espd_h.x*vmc[id].param.invert_knee_epos[Xr]
		- robotwb.Leg[id].jacobi_inv[4] * robotwb.Leg[id].tar_espd_h.y//*robotwb.Leg[id].flag_rl
		+ (-robotwb.Leg[id].jacobi_inv[5] * robotwb.Leg[id].tar_espd_h.z))*gain_espd;
	temp[2] = (-robotwb.Leg[id].jacobi_inv[6] * robotwb.Leg[id].tar_espd_h.x*vmc[id].param.invert_knee_epos[Xr]
		- robotwb.Leg[id].jacobi_inv[7] * robotwb.Leg[id].tar_espd_h.y//*robotwb.Leg[id].flag_rl
		+ (-robotwb.Leg[id].jacobi_inv[8] * robotwb.Leg[id].tar_espd_h.z))*gain_espd;

	robotwb.Leg[id].tar_sita_d[0] = temp[0] * flt_espd + (1 - flt_espd)*robotwb.Leg[id].tar_sita_d[0];
	robotwb.Leg[id].tar_sita_d[1] = temp[1] * flt_espd + (1 - flt_espd)*robotwb.Leg[id].tar_sita_d[1];
	robotwb.Leg[id].tar_sita_d[2] = temp[2] * flt_espd + (1 - flt_espd)*robotwb.Leg[id].tar_sita_d[2];
}

void force_to_tao(int id, float dt)
{
	float flt_t = 1;//滤波系数
	float gain_t = -1;//输出符号
	float temp[3] = { 0,0,0 };
	temp[0] = (robotwb.Leg[id].jacobi[0] * robotwb.Leg[id].tar_force_h.x*vmc[id].param.invert_knee_epos[Xr]
		+ (robotwb.Leg[id].jacobi[3] * robotwb.Leg[id].tar_force_h.y)
		+ (robotwb.Leg[id].jacobi[6] * robotwb.Leg[id].tar_force_h.z)
		)*gain_t;
	temp[1] = (robotwb.Leg[id].jacobi[1] * robotwb.Leg[id].tar_force_h.x*vmc[id].param.invert_knee_epos[Xr]
		+ (robotwb.Leg[id].jacobi[4] * robotwb.Leg[id].tar_force_h.y)
		+ (robotwb.Leg[id].jacobi[7] * robotwb.Leg[id].tar_force_h.z)
		)*gain_t;
	temp[2] = (robotwb.Leg[id].jacobi[2] * robotwb.Leg[id].tar_force_h.x*vmc[id].param.invert_knee_epos[Xr]
		+ (robotwb.Leg[id].jacobi[5] * robotwb.Leg[id].tar_force_h.y)
		+ (robotwb.Leg[id].jacobi[8] * robotwb.Leg[id].tar_force_h.z)
		)*gain_t;
	robotwb.Leg[id].taod[0] = temp[0] * flt_t + (1 - flt_t)*robotwb.Leg[id].taod[0];
	robotwb.Leg[id].taod[1] = temp[1] * flt_t + (1 - flt_t)*robotwb.Leg[id].taod[1];
	robotwb.Leg[id].taod[2] = temp[2] * flt_t + (1 - flt_t)*robotwb.Leg[id].taod[2];

	robotwb.Leg[id].taod[0] = limitw(robotwb.Leg[id].taod[0], -robotwb.Leg[id].limit_tao[0], robotwb.Leg[id].limit_tao[0]);
	robotwb.Leg[id].taod[1] = limitw(robotwb.Leg[id].taod[1], -robotwb.Leg[id].limit_tao[1], robotwb.Leg[id].limit_tao[1]);
	robotwb.Leg[id].taod[2] = limitw(robotwb.Leg[id].taod[2], -robotwb.Leg[id].limit_tao[2], robotwb.Leg[id].limit_tao[2]);
}

void force_to_tao_input(int id, Vect3 force_h, float dt)
{
	float flt_t = 1;
	float gain_t = -1;
	float temp[3] = { 0,0,0 };
	temp[0] = (robotwb.Leg[id].jacobi[0] * force_h.x*vmc[id].param.invert_knee_epos[Xr]
		+ (robotwb.Leg[id].jacobi[3] * force_h.y)
		+ (robotwb.Leg[id].jacobi[6] * force_h.z)
		)*gain_t;
	temp[1] = (robotwb.Leg[id].jacobi[1] * force_h.x*vmc[id].param.invert_knee_epos[Xr]
		+ (robotwb.Leg[id].jacobi[4] * force_h.y)
		+ (robotwb.Leg[id].jacobi[7] * force_h.z)
		)*gain_t;
	temp[2] = (robotwb.Leg[id].jacobi[2] * force_h.x*vmc[id].param.invert_knee_epos[Xr]
		+ (robotwb.Leg[id].jacobi[5] * force_h.y)
		+ (robotwb.Leg[id].jacobi[8] * force_h.z)
		)*gain_t;
	robotwb.Leg[id].taod[0] = temp[0] * flt_t + (1 - flt_t)*robotwb.Leg[id].taod[0];
	robotwb.Leg[id].taod[1] = temp[1] * flt_t + (1 - flt_t)*robotwb.Leg[id].taod[1];
	robotwb.Leg[id].taod[2] = temp[2] * flt_t + (1 - flt_t)*robotwb.Leg[id].taod[2];

	robotwb.Leg[id].taod[0] = limitw(robotwb.Leg[id].taod[0], -robotwb.Leg[id].limit_tao[0], robotwb.Leg[id].limit_tao[0]);
	robotwb.Leg[id].taod[1] = limitw(robotwb.Leg[id].taod[1], -robotwb.Leg[id].limit_tao[1], robotwb.Leg[id].limit_tao[1]);
	robotwb.Leg[id].taod[2] = limitw(robotwb.Leg[id].taod[2], -robotwb.Leg[id].limit_tao[2], robotwb.Leg[id].limit_tao[2]);
}

//正运动学
char estimate_end_state_new(float dt)//??????
{

    float q_in[6];
    END_POS end_pos_now,end_att_now;
    q_in[0]=robotwb.arm_base_height;
    q_in[1]=robotwb.arm_q[0][0];
    q_in[2]=robotwb.arm_q[0][1];
    q_in[3]=robotwb.arm_q[0][2];
    q_in[4]=robotwb.arm_q[0][3];
    q_in[5]=robotwb.arm_q[0][4];
    fk_pino(q_in, &end_pos_now, &end_att_now);
    if(end_att_now.x<-90)
        end_att_now.x=180+end_att_now.x;
    if(end_att_now.y>90)
        end_att_now.y=180-end_att_now.y;

    end_pos_now.z-=robotwb.base_z;

    robotwb.arm_epos_h=end_pos_now;
    robotwb.arm_att_h=end_att_now;
//    printf("att %f %f %f\n",end_att_now.x,end_att_now.y,end_att_now.z);
//    printf("pos %f %f %f\n",end_pos_now.x,end_pos_now.y,end_pos_now.z);
    robotwb.arm_epos_b.x=robotwb.arm_epos_h.x+robotwb.tf_b2h.x;
    robotwb.arm_epos_b.y=robotwb.arm_epos_h.y+robotwb.tf_b2h.y;
    robotwb.arm_epos_b.z=robotwb.arm_epos_h.z+robotwb.tf_b2h.z;

    robotwb.arm_att_h=end_att_now;
    robotwb.arm_att_b=robotwb.arm_att_h;
    robotwb.arm_att_n=robotwb.arm_att_b;

    robotwb.arm_epos_n.x = robotwb.Rb_n_noyaw[0][0] * robotwb.arm_epos_b.x
            + robotwb.Rb_n_noyaw[0][1] * robotwb.arm_epos_b.y
            + robotwb.Rb_n_noyaw[0][2] * robotwb.arm_epos_b.z;
    robotwb.arm_epos_n.y = robotwb.Rb_n_noyaw[1][0] * robotwb.arm_epos_b.x
            + robotwb.Rb_n_noyaw[1][1] * robotwb.arm_epos_b.y
            + robotwb.Rb_n_noyaw[1][2] * robotwb.arm_epos_b.z;
    robotwb.arm_epos_n.z = robotwb.Rb_n_noyaw[2][0] * robotwb.arm_epos_b.x
            + robotwb.Rb_n_noyaw[2][1] * robotwb.arm_epos_b.y
            + robotwb.Rb_n_noyaw[2][2] * robotwb.arm_epos_b.z;

    robotwb.arm_epos_ny.x = robotwb.Rb_n[0][0] * robotwb.arm_epos_b.x
            + robotwb.Rb_n[0][1] * robotwb.arm_epos_b.y
            + robotwb.Rb_n[0][2] * robotwb.arm_epos_b.z;
    robotwb.arm_epos_ny.y = robotwb.Rb_n[1][0] * robotwb.arm_epos_b.x
            + robotwb.Rb_n[1][1] * robotwb.arm_epos_b.y
            + robotwb.Rb_n[1][2] * robotwb.arm_epos_b.z;
    robotwb.arm_epos_ny.z = robotwb.Rb_n[2][0] * robotwb.arm_epos_b.x
            + robotwb.Rb_n[2][1] * robotwb.arm_epos_b.y
            + robotwb.Rb_n[2][2] * robotwb.arm_epos_b.z;

#if 0
    printf("pos  hx=%.2f hy=%.2f hz=%.2f\n",  robotwb.arm_epos_h.x, robotwb.arm_epos_h.y, robotwb.arm_epos_h.z);
    printf("body bx=%.2f by=%.2f bz=%.2f\n",  robotwb.arm_epos_b.x,robotwb.arm_epos_b.y,robotwb.arm_epos_b.z);
    printf("n    nx=%.2f ny=%.2f nz=%.2f\n",  robotwb.arm_epos_n.x,robotwb.arm_epos_n.y,robotwb.arm_epos_n.z);
    printf("ny   nx=%.2f ny=%.2f nz=%.2f\n",  robotwb.arm_epos_ny.x,robotwb.arm_epos_ny.y,robotwb.arm_epos_ny.z);
#endif
    static float spd_est_cnt=0;
    float end_sample_dt=0.005;
    spd_est_cnt+=dt;

    if (spd_est_cnt > end_sample_dt) {
        float d_x = (robotwb.arm_epos_h.x - robotwb.arm_epos_h_reg.x) / spd_est_cnt;
        float d_y = (robotwb.arm_epos_h.y - robotwb.arm_epos_h_reg.y) / spd_est_cnt;
        float d_z = (robotwb.arm_epos_h.z - robotwb.arm_epos_h_reg.z) / spd_est_cnt;

        robotwb.arm_vpos_h.x = LIMIT(d_x, -5, 5);
        robotwb.arm_vpos_h.y = LIMIT(d_y, -5, 5);
        robotwb.arm_vpos_h.z = LIMIT(d_z, -5, 5);
        robotwb.arm_epos_h_reg.x = robotwb.arm_epos_h.x;
        robotwb.arm_epos_h_reg.y = robotwb.arm_epos_h.y;
        robotwb.arm_epos_h_reg.z = robotwb.arm_epos_h.z;
        spd_est_cnt = 0;
    }

    DigitalLPF(robotwb.arm_vpos_h.x, &robotwb.arm_vpos_h_flt.x, FLT_SPD_END, dt);
    DigitalLPF(robotwb.arm_vpos_h.y, &robotwb.arm_vpos_h_flt.y, FLT_SPD_END, dt);
    DigitalLPF(robotwb.arm_vpos_h.z, &robotwb.arm_vpos_h_flt.z, FLT_SPD_END, dt);

    robotwb.arm_vpos_b=robotwb.arm_vpos_h_flt;

    robotwb.arm_vpos_n.x = robotwb.Rb_n_noyaw[0][0] * robotwb.arm_vpos_b.x
            + robotwb.Rb_n_noyaw[0][1] * robotwb.arm_vpos_b.y
            + robotwb.Rb_n_noyaw[0][2] * robotwb.arm_vpos_b.z;
    robotwb.arm_vpos_n.y = robotwb.Rb_n_noyaw[1][0] * robotwb.arm_vpos_b.x
            + robotwb.Rb_n_noyaw[1][1] * robotwb.arm_vpos_b.y
            + robotwb.Rb_n_noyaw[1][2] * robotwb.arm_vpos_b.z;
    robotwb.arm_vpos_n.z = robotwb.Rb_n_noyaw[2][0] * robotwb.arm_vpos_b.x
            + robotwb.Rb_n_noyaw[2][1] * robotwb.arm_vpos_b.y
            + robotwb.Rb_n_noyaw[2][2] * robotwb.arm_vpos_b.z;

    robotwb.arm_vpos_ny.x = robotwb.Rb_n[0][0] * robotwb.arm_vpos_b.x
            + robotwb.Rb_n[0][1] * robotwb.arm_vpos_b.y
            + robotwb.Rb_n[0][2] * robotwb.arm_vpos_b.z;
    robotwb.arm_vpos_ny.y = robotwb.Rb_n[1][0] * robotwb.arm_vpos_b.x
            + robotwb.Rb_n[1][1] * robotwb.arm_vpos_b.y
            + robotwb.Rb_n[1][2] * robotwb.arm_vpos_b.z;
    robotwb.arm_vpos_ny.z = robotwb.Rb_n[2][0] * robotwb.arm_vpos_b.x
            + robotwb.Rb_n[2][1] * robotwb.arm_vpos_b.y
            + robotwb.Rb_n[2][2] * robotwb.arm_vpos_b.z;
#if 0
    printf("vel  hx=%.2f hy=%.2f hz=%.2f\n",  robotwb.arm_vpos_h.x, robotwb.arm_vpos_h.y, robotwb.arm_vpos_h.z);
    printf("body bx=%.2f by=%.2f bz=%.2f\n",  robotwb.arm_vpos_b.x,robotwb.arm_vpos_b.y,robotwb.arm_vpos_b.z);
    printf("n    nx=%.2f ny=%.2f nz=%.2f\n",  robotwb.arm_vpos_n.x,robotwb.arm_vpos_n.y,robotwb.arm_vpos_n.z);
    printf("ny   nx=%.2f ny=%.2f nz=%.2f\n",  robotwb.arm_vpos_ny.x,robotwb.arm_vpos_ny.y,robotwb.arm_vpos_ny.z);
#endif

#if EN_ROTATE_END_COMPASS_EST&&1//旋转速度补偿
	Vect3 Hip_b;
	Vect3 com;
	com.x = com.y = com.z = 0;
    Hip_b.x = com.x - robotwb.arm_epos_b.x;
    Hip_b.y = com.y - robotwb.arm_epos_b.y;
    Hip_b.z = com.z - robotwb.arm_epos_b.z;

	Vect3 w_b;
    w_b.x = vmc_all.att_rate_trig[ROLr] * 0.0173 * 1;
	w_b.y = vmc_all.att_rate_trig[PITr] * 0.0173 * 1;
    w_b.z = vmc_all.att_rate_trig[YAWr] * 0.0173 * 1;
	float w_cross_b[3][3] = { 0 };
     vect3_2_cross(w_b, w_cross_b);
    Vect3 hip_rotate_spd_b, hip_rotate_spd_n;

    matrx33_mult_vect3(w_cross_b, Hip_b, &hip_rotate_spd_b);
    converV_b_to_n(hip_rotate_spd_b.x, hip_rotate_spd_b.y, hip_rotate_spd_b.z,
        &hip_rotate_spd_n.x, &hip_rotate_spd_n.y, &hip_rotate_spd_n.z);//转换速度直接输出到IMP  控制频率不够目前？？

    robotwb.arm_vpos_n.x -= hip_rotate_spd_n.x;
    robotwb.arm_vpos_n.y -= hip_rotate_spd_n.y;
    robotwb.arm_vpos_n.z -= hip_rotate_spd_n.z;
#endif

    //head pose
    END_POS head_att_off_h,head_att_off;
    head_att_off_h.x=robotwb.head_tf_h2head.zz;
    head_att_off_h.y=0;
    head_att_off_h.z=0;
    float att_rt_head[3];
    att_rt_head[0]=0;
    att_rt_head[1]=robotwb.head_att_exp[1];//pitch
    att_rt_head[2]=robotwb.head_att_exp[0];//yaw
    converV_n_to_b_w_yaw_temp(att_rt_head,
            head_att_off_h.x,head_att_off_h.y,head_att_off_h.z,
            &head_att_off.x,&head_att_off.y,&head_att_off.z);
   // printf("headp=%f %f %f\n",head_att_off.x,head_att_off.y,head_att_off.z);
    robotwb.head_pos_h.x=robotwb.head_tf_h2head.x+head_att_off.x;
    robotwb.head_pos_h.y=robotwb.head_tf_h2head.y+head_att_off.y;
    robotwb.head_pos_h.z=robotwb.head_tf_h2head.z+head_att_off.z+robotwb.arm_base_height;

    robotwb.head_pos_b.x=robotwb.head_pos_h.x+robotwb.tf_b2h.x;
    robotwb.head_pos_b.y=robotwb.head_pos_h.y+robotwb.tf_b2h.y;
    robotwb.head_pos_b.z=robotwb.head_pos_h.z+robotwb.tf_b2h.z;

    robotwb.head_pos_n.x = robotwb.Rb_n[0][0] * robotwb.head_pos_b.x
            + robotwb.Rb_n[0][1] * robotwb.head_pos_b.y
            + robotwb.Rb_n[0][2] * robotwb.head_pos_b.z;
    robotwb.head_pos_n.y = robotwb.Rb_n[1][0] * robotwb.head_pos_b.x
            + robotwb.Rb_n[1][1] * robotwb.head_pos_b.y
            + robotwb.Rb_n[1][2] * robotwb.head_pos_b.z;
    robotwb.head_pos_n.z = robotwb.Rb_n[2][0] * robotwb.head_pos_b.x
            + robotwb.Rb_n[2][1] * robotwb.head_pos_b.y
            + robotwb.Rb_n[2][2] * robotwb.head_pos_b.z;

	return 0;
}

char inv_end_state_new(END_POS epos,END_POS eatt, float q_out[6])
{

    float q_ik[6];
    ik_pino(epos,eatt,q_ik);
    q_out[0]=q_ik[0];//z
    q_out[1]=q_ik[1]*57.3;//
    q_out[2]=q_ik[2]*57.3;//
    q_out[3]=q_ik[3]*57.3;//
    q_out[4]=q_ik[4]*57.3;//
    q_out[5]=q_ik[5]*57.3;//

	return 1;
}

char cal_invjacobi(VMC *in)
{
	float det = 0;
	char i = 0;
	invet33(in->jacobi33, &det, in->ijacobi33);
	if (fabs(det) < 0.00001)
		return 0;
	return 1;
}

char cal_jacobi_new(VMC *in)
{
	float Jcobi[9];
#if 1
	Jcobi[0] =//in->param.invert_knee_epos[Xr]*
		-in->l1 * cosdw(in->sita1) - in->l2 * cosdw(180 - in->sita2 - in->sita1);//x p_D
	Jcobi[1] =//in->param.invert_knee_epos[Xr]*
		-in->l2 * cosdw(180 - in->sita2 - in->sita1);//x p_X
	Jcobi[2] =//in->param.invert_knee_epos[Xr]*
		0;//x p_T
	float h = in->l1 * cosdw(in->sita1) + in->l2 * cosdw(180 - in->sita2 - in->sita1)+in->l4;
	float h_s1 = -in->l1 * sindw(in->sita1) + in->l2 * sindw(180 - in->sita2 - in->sita1);
	float h_s2 = in->l2 * sindw(180 - in->sita2 - in->sita1);
	float h_s3 = 0;

	Jcobi[3] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
		h_s1 * sindw(in->sita3 * in->flag_rl);//y p_D
	Jcobi[4] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
		h_s2 * sindw(in->sita3 * in->flag_rl);//y p_X
	Jcobi[5] = in->flag_rl*//in->param.invert_knee_epos[Yr]*
		(-in->l3 * sindw(in->sita3 * in->flag_rl)*in->flag_rl +
			h_s3 * sindw(in->sita3 * in->flag_rl) +
			h * cosdw(in->sita3 * in->flag_rl)*in->flag_rl);//y p_T

	Jcobi[6] = -h_s1 * cosdw(in->sita3 * in->flag_rl);//z p_D
	Jcobi[7] = -h_s2 * cosdw(in->sita3 * in->flag_rl);//z p_X
	Jcobi[8] = -h_s3 * cosdw(in->sita3 * in->flag_rl)
		+ h * sindw(in->sita3 * in->flag_rl)*in->flag_rl
		+ in->l3 * cosdw(in->sita3)*in->flag_rl;//z p_T
#endif
	for (int i = 0; i < 9; i++)
		in->jacobi33[i] = Jcobi[i];
 
	return 1;
}
