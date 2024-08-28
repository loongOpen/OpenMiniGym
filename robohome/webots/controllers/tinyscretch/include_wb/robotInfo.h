#include "wbInterface.h"
#include "assert.h"

#define Www          0.05*2 
#define Hw           0.19*2
#define L1w          0.2
#define L2w          0.2
#define L3w          0.084
#define Mw           10.0

#define gw           9.8
#define piw          3.1415926
#define rad(x)      (x*piw/180.0)    //将角度化为弧度
#define deg(x)      (x*180.0/piw)    //将弧度化为角度
#define RAD_TO_DEGw  57.3
#define DEG_TO_RADw  0.0173

#define PITrw 0
#define ROLrw 1
#define YAWrw 2
#define Xrw 0
#define Yrw 1
#define Zrw 2
#define FRw 0
#define HRw 1
#define FLw 2
#define HLw 3
#define Fw 0
#define Bw 1   

typedef struct
{
  /* 三维坐标 */
  float x;
  float y;
  float z;
  /* 矢状面极坐标 */
  float r;      //r = sqrt(x*x+z*z)
  float theta;
}pointTypeDef;

//机器人欧拉角结构体
typedef struct
{
  float roll;       //横滚，x轴
  float pitch;      //俯仰，y轴
  float yaw;        //偏航，z轴
}eulerAngleTypeDef;

//机器人相对于世界坐标系位置
typedef struct
{
	float x;
	float y;
	float z;
}robPosTypeDef;   

typedef struct
{
  float x;   
  float y;   
  float z;   
}Vect3;

typedef struct
{
  float exp,now,now_reg,err,err_dead,err_reg;   
  float p_out;   
  float i_out;
  float d_out;
  float max_out;
  float kp,ki,kd;   
}PIDs;

typedef struct
{
  float kp_pos,ki_pos,kd_pos;
  float kp_force,ki_force,kd_force;
}POS_FORCE_P;

typedef struct {
	char en_force_control_cal,en_force_control_out;
    PIDs q_pid_sw,q_pid_st;
    PIDs f_pos_pid_st[3];
    PIDs f_pid_st[3];
	PIDs zeros;
	//---
	float touch_z_thr[5];
	//--
	float max_t;
	float max_i;
	
}POS_FORCE_PARM;

extern POS_FORCE_PARM pos_force_p;

typedef struct {
  PIDs pos_x,pos_z;
  PIDs spd_x,spd_yaw;
  PIDs att_pit,att_rol,att_yaw;
	float mess_scale;
}VMC_ROBOT_PARM;

extern VMC_ROBOT_PARM vmc_robot_p;

//单腿信息结构体
typedef struct 
{
  int id;

  Vect3 epos_h,epos_h_reg;
  Vect3 tar_epos_h,tar_epos_n;
  Vect3 force_err_n,force_imp_spd_h;
  float pos_taod[3],fb_force_taod[3],ff_force_taod[3];
	

  Vect3 epos_b;
  Vect3 epos_n;

  Vect3 espd_h,espd_n;

  Vect3 tar_espd_h;
  Vect3 tar_espd_n;

  Vect3 tar_force_h;
  Vect3 tar_force_n,tar_force_dis_n,tar_force_dis_n_qp, tar_force_dis_g;
  
  Vect3 force_est_h;
  Vect3 force_est_n;
  
  Vect3 force_est_h_output;
  Vect3 force_est_n_output;

  Vect3 dforce_est_h_output;
  Vect3 dforce_est_n_output;

  Vect3 force_est_h_output_reg;
  Vect3 force_est_n_output_reg;

  PIDs q_pid_sw[3],q_pid_st[3];
  PIDs f_pos_pid_st[3],f_pos_pid_sw[3];
  PIDs f_pid_st[3],f_pid_sw[3];
  PIDs q_pid[3],f_pid[3],f_pos_pid[3];
  int swing_state;
  float sita[3],sita_reg[3],sita_r,r;
  float limit_sita[3];
  float sita_d[3],sita_dd[3];
  float err_sita[3];
  float tar_sita[3];
  float tar_sita_d[3];

  float jacobi[9],jacobi_inv[9];
  
  int flag_fb,flag_rl;

  float pos[3];         //测量得到的当前关节角度            弧度制
  float taom[3];        //实时测量的扭矩
  float taom_output[3];        //实时测量的扭矩
  float taod[3];        //期望扭矩
  float taod_ff[3];        //期望扭矩
  float tao_q_i[3];
  float limit_tao[3];
  bool  is_ground;
  bool  is_touch,is_touch_est;       //是否触地 true : 触地 false : 离地
  float touch_z_thr[5];
  int touch_cnt[5][2];
  int touch_cnto[5];
  int trig_state,ground_state,st_torque_need_init;
  Vect3 st_pos;
  Vect3 epos_td_n,epos_lf_n;
  Vect3 epos_sw_end_n;
  Vect3 epos_td_hn,epos_lf_hn;
  Vect3 epos_spdd_n,epos_spdd_b;
  Vect3 epos_sw_st_n,epos_sw_st_b;
  Vect3 epos_nn;
  Vect3 odom_st;
  float delta_ht;
  float time_trig;
  float cnt_ss;
}LegTypeDef;



typedef struct {
	char connect;
	int loss_cnt;
	char mode;
	char up_mode;
	float rc_spd_b[3], rc_rate_b[3];

	char cmd_robot_state;
	float rc_spd_w[2], rc_att_w[2], rate_yaw_w;
	int key_ud, key_lr;
	char key_x, key_y, key_a, key_b, key_ll, key_rr, key_st, key_back;
	int key_ud_reg, key_lr_reg;
	char key_x_reg, key_y_reg, key_a_reg, key_b_reg, key_ll_reg, key_rr_reg, key_st_reg, key_back_reg;
	float curve[20];
	char sbus_conncect;
	char sbus_power_sw, sbus_power_sw_reg;
	int sbus_rc_main[4];
	int sbus_rc_main_reg[4];
	int sbus_mode, sbus_mode_reg;
	int sbus_mode_e, sbus_mode_e_reg;
	float sbus_height, sbus_height_reg;
	int sbus_aux[4];
	int sbus_aux_reg[4];

}_OCU;

//机器人结构体 包含整个机器人的所有信息
typedef struct 
{
  int gait_level;
  int state_gait;
  int gait_mode;
  _OCU ocu;
  //stateMachineTypeDef stateMachine;   //机器人状态机

  eulerAngleTypeDef   IMU_now;        //机器人当前欧拉角
  eulerAngleTypeDef   ground_att;
  eulerAngleTypeDef   IMU_last;       //机器人上次欧拉角
  eulerAngleTypeDef   IMU_dot;        //机器人欧拉角速度

  robPosTypeDef       GPS_now;        //机器人当前三维位置
  robPosTypeDef       GPS_last;       //机器人上次三维位置
  robPosTypeDef       GPS_dot;        //机器人当前三维速度

  eulerAngleTypeDef   exp_att;        
  eulerAngleTypeDef   now_att;        
  eulerAngleTypeDef   now_rate;        
  Vect3               exp_spd_b;
  Vect3               exp_pos_b;
  Vect3               exp_spd_n;
  Vect3               exp_pos_n;
  Vect3               cog_spd_b;
  Vect3               cog_pos_b;
  Vect3               cog_spd_n;
  Vect3               cog_pos_n,cog_pos_nn;
  int ground_num;
  int ground_num_rl[2];
  int ground_num_fb[2];
  LegTypeDef          Leg[4];         //四条腿
  uint32_t            t    ;          //时间计量,某些操作需要以时间为分割点
  Vect3 exp_force;
  Vect3 exp_torque;

  float Rb_n[3][3];
  float Rn_b[3][3]; 
  float Rn_g[3][3]; 

  float ground_att_cmd[3];

  float MIN_X;
  float MAX_X;
  float MIN_Y;
  float MAX_Y;
  float MIN_Z;
  float MAX_Z;
  float MAX_SPD[3];

//param
  int use_ground_sensor;
  float gait_time;
  float stance_time;
  float kp_trig[2];
  float leg_off[2];
  Vect3 ankle_pos_n[4],cog_spd_nn;
}robotTypeDef;


extern robotTypeDef robotwb;
void record_thread(char sel, float dt);
void robot_int(void);
void robotInfoUpdata(robotTypeDef* rob, float dt);
void robotEstimator(robotTypeDef* rob, float dt);
void forwark_KI(float dt);
void legInfoUpdata(robotTypeDef* rob, float dt);
void get_IMU_Angle(robotTypeDef* rob, float dt);
void readAllMotorPos(robotTypeDef* rob, float dt);

void converV_n_to_bw(Vect3 vn, Vect3* vb);
void converV_n_to_gw(Vect3 vn, Vect3* vg);
void converV_b_to_legw(int id, Vect3 vb, Vect3* vl);
void converV_b_to_leg_ow(char leg, float xb, float yb, float zb, float *xl, float *yl, float *zl);
void converV_leg_to_bw(int id, Vect3 vl, Vect3* vb);
void converV_b_to_nw(Vect3 vb, Vect3* vn);
void force_n_to_bw(Vect3 fn, Vect3* fb);
void converV_b_to_n_RTw(float RT[3][3], float yaw, float xb, float yb, float zb, float *xn, float *yn, float *zn);
void converV_n_to_b_w_yaWww(float yaw, float xn, float yn, float zn, float *xb, float *yb, float *zb);

void inv_KI(int id, Vect3 epos_h, float *s0, float *s1, float *s2);
void set_motor_q(int id);
void set_motor_t(int id);

void espd_to_neg_dq(int id, float dt);
void force_to_tao(int id, float dt);
void force_to_tao_input(int id, Vect3 force_h, float dt);

void force_dis_n(void);
