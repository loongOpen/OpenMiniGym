#include "include.h"
#include "locomotion_header.h"
#include "gait_math.h"
#include "eso.h"
#include "ground_att_est_n.h"

robotTypeDef robotwb;
_ODOM_1 odom1;
//复位机器人状态估计
void reset_robot_statement(void)
{
     robotwb.base_rate_exp_rc=robotwb.base_rate_exp=0;
     robotwb.base_vel_b_exp_rc.x=robotwb.base_vel_b_exp.x=0;
     robotwb.base_pos_n_w.x=robotwb.base_pos_n_w.y=0;
     robotwb.base_pos_n.x=robotwb.base_pos_n.y=0;

}

void touchdown_check(robotTypeDef* rob, float dt)//zhaodi 三次采样着地判断
{

}
 
void estimate_GRF(float dt)
{

}
 
void state_estimator(float dt)//guji
{
	static float st;
	static char init;
	static float time[5];
	float temp_odom_spd[3] = { 0 };
	float att_use[3], err[3], cog_off_hover, temp_end_dis;
	char i;
	if (!init)
	{
		TD4_init(&odom_td[Xr], 15, 15, 15, 15);
		TD4_init(&odom_td[Yr], 15, 15, 15, 15);
		TD4_init(&odom_td[Zr], 15, 15, 15, 15);
	}

    robotwb.wheel_touch_cnt=0;
    for(int i=0;i<4;i++)
        if(robotwb.wheel_touch[i])
            robotwb.wheel_touch_cnt++;

    float temp= robotwb.wheel_v[0]*robotwb.wheel_touch[0]
               +robotwb.wheel_v[1]*robotwb.wheel_touch[1]
               +robotwb.wheel_v[2]*robotwb.wheel_touch[2]
               +robotwb.wheel_v[3]*robotwb.wheel_touch[3];
    if(robotwb.wheel_touch_cnt>0)
        DigitalLPFw(LIMIT(temp/robotwb.wheel_touch_cnt, -3, 3), &robotwb.base_vel_b_w.x, 100, dt);


    robotwb.base_vel_n_w.x = robotwb.Rb_n[0][0] * robotwb.base_vel_b_w.x
            + robotwb.Rb_n[0][1] * robotwb.base_vel_b_w.y
            + robotwb.Rb_n[0][2] * robotwb.base_vel_b_w.z;
    robotwb.base_vel_n_w.y = robotwb.Rb_n[1][0] * robotwb.base_vel_b_w.x
            + robotwb.Rb_n[1][1] * robotwb.base_vel_b_w.y
            + robotwb.Rb_n[1][2] * robotwb.base_vel_b_w.z;
    robotwb.base_vel_n_w.z = robotwb.Rb_n[2][0] * robotwb.base_vel_b_w.x
            + robotwb.Rb_n[2][1] * robotwb.base_vel_b_w.y
            + robotwb.Rb_n[2][2] * robotwb.base_vel_b_w.z;



    float w_l=robotwb.wheel_v[0]*robotwb.wheel_touch[0]+robotwb.wheel_v[2]*robotwb.wheel_touch[2];
    if(robotwb.wheel_touch[0]||robotwb.wheel_touch[2])
    w_l/=(robotwb.wheel_touch[0]+robotwb.wheel_touch[2]);
    float w_r=robotwb.wheel_v[1]*robotwb.wheel_touch[1]+robotwb.wheel_v[3]*robotwb.wheel_touch[3];
    if(robotwb.wheel_touch[0]||robotwb.wheel_touch[2])
    w_r/=(robotwb.wheel_touch[1]+robotwb.wheel_touch[3]);

    robotwb.base_rate_w=((w_r-w_l)/robotwb.wheel_w)/2;
    robotwb.base_rate_imu=robotwb.now_rate.yaw/57.3;
    float flt_w=0.2;
    robotwb.base_rate_fushion=flt_w*robotwb.base_rate_w+(1-flt_w)*robotwb.base_rate_imu;
    //Kf fushion
    robotwb.base_pos_n_w.x+=robotwb.base_vel_n_w.x*dt;
    robotwb.base_pos_n_w.y+=robotwb.base_vel_n_w.y*dt;
    robotwb.base_pos_n_w.z=robotwb.base_height;


    robotwb.base_pos_n=robotwb.base_pos_n_w;
#if 0
    static int cnt_p=0;
    if(cnt_p++>10){cnt_p=0;
    printf("base_vel[%d]:exp=%f now_b=%f\n",robotwb.wheel_touch_cnt,robotwb.base_vel_b_exp_rc.x,robotwb.base_vel_b_w.x);
    printf("base_vel: n=%f  %f  %f\n",robotwb.base_vel_n_w.x,robotwb.base_vel_n_w.y,robotwb.base_vel_n_w.z);
    printf("pos_vel: n=%f  %f  %f\n",robotwb.base_pos_n_w.x,robotwb.base_pos_n_w.y,robotwb.base_pos_n_w.z);
    printf("yaw_rate: exp=%.3f w=%.3f imu=%.3f fushion=%.3f\n",robotwb.base_rate_exp,robotwb.base_rate_w,robotwb.base_rate_imu,robotwb.base_rate_fushion);
    }
#endif
}
 double Pn_td[4][3] = { 0 };
//地形估计
void estimate_ground_att(float dt) {

}

void pay_load_estimation(float dt)
{

}
