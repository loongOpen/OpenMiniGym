#ifndef __BASE_STRUCT_H__
#define __BASE_STRUCT_H__
#include "include.h"
#include "math.h"
typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef signed   char  int8;
typedef unsigned short uint16;
typedef unsigned int   uint32;
typedef float  fp32;
typedef double fp64;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef int16_t s16;
typedef int8_t  s8;
// 左侧容易拉倒？？？！！bug  //void force_n_to_bw(Vect3 fn,Vect3* fb){ 内部写错
extern  double Pn_td[4][3];

#define EN_ARM_H                  1
#define EN_ROT_Q4                 1
#define EN_RECORDER				  1
#define EN_H_MAP_UPDATE           1
#define EN_VISION                 1
#define EN_HUMAN                  0
//--------------------------------VMC库宏定义----------------------------------
#define EN_VISION_STOP_GAIT       0
#define USE_MOM_LIP               0

#define SW_USE_POS_N              0 //摆动转换全局
#define SWING_IN_NN               0 //摆动考虑里程计

#define USE_ZMP                   0

#define EN_SW_SLOP_ETH            0  //ETH台阶轨迹转换
#define FIX_SLOP_OFF              0  //度
#define SLOP_DEAD                 6 //度
#define EN_KF_MIT                 1 //倾斜后前后运动

#define EN_RT_WITHOUT_ROLL        1 //高落差会摔倒

#define SWING_USE_IK              0
#define USE_MPC                   1
#define USE_MPC_QP                0
#define USE_QP                    1//Jog只用QP
#define MPC_GAIT_QP_FORCE         0

#define EN_SW_BOUND               0

#define EN_ROTATE_END_COMPASS_EST 1
#define EN_ROTATE_END_COMPASS_SW  1

#define EN_ST_TOUCH           1

#define EN_PLAN_USE_JERK      5
#define EN_PLAN_USE_MIT       1
#define USE_SWING_N           0

#define FIX_ST_TIME           1   //固定支撑时间  不开好点或者1
#define MIN_ST_TIME_RATE      0.6 //% 能保证摔倒起来
#define FORCE_FB_USE_REAL     1   //足底力使用真实IQ轴估计

#define EN_ONLINE_SW_PLAN     1   //在线修正X方向落足点
#define RE_PLAN_DT            0.005//s越短越好
#define SW_LOW_RATE1		  0.65
#define SW_LOW_RATE2		  0.75
#define SW_TD_CHECK_RATE      0.6
#define SW_TD_CHECK_RATE_S    0.9
#define SW_TD_OVER_TIME       0.3//s
#define TD_NO_SHOCK_TIME      0.005//s

#define F_CONTROL_WITH_ROLL   0   //补偿横滚力的输出  仿真里没区别
#define F_EST_WITH_ROLL       1   //估计力使用横滚补偿  not good
#define F_SWING_WITH_ROLL     1
#define ROLL_LIMIT_COM		  25

#define USE_FORCE_REAL_GROUND 0   //TORT不会抖  0则高度保证但有姿态误差会明显大抖  采用老分配与2腿分配效果差不多仿真  实物2腿横滚没力

#define KIN_5LINE_FK          1
#define KIN_5LINE_IK          1  //Bug
#define KIN_5LINE_J           1  //相比老的不太行 not good
//新状态机：新规划 使用反馈差于不使用 但是下台阶时不太行    GROUND_AFTER_TRIG=0|| 老规划  不行和实物类似
//老状态机：：新规划 使用反馈好 GROUND_AFTER_TRIG=0  l||老规划 可以 GROUND_AFTER_TRIG=0
//->使用老规划和老状态机 不使用反馈   ->新状态机+老规划
#define GROUND_AFTER_TRIG       1   //TROT 着地就有力控 不等待另一个腿落地   规划和老状态机--使用后估计高度不跳变 不容易摔倒
#define EN_TORT_LOAD_FORCE4 	1   //TROT对角等待使用LOAD力 否则着地直接使用力分配  导致侧翻无法复原！！！ 不能与GROUND_AFTER_TRIG一起使用
#define EN_TORT_LOAD_FORCE5     1   //摆动故障等待使用LOAD力

#define EN_Q_I_MIT_MODE       0   //角度积分
#define EN_END_SPD_MODE       1   //站立等末端采用速度控制

#define ODOM_USE_1     		  0	//使用丁博士的加速度里程计

#define TEST_TROT_SW 		  0 //摆动测试<<------------------------
#define EN_SW 			      1 //TROT能摆动
#define SW_WITH_REAL_FB       0 //使用反馈容易发散
#define SWING_USE_SPD_MODE    1    //摆动使用位置微分速度  仿真中不使用比较好 低摆动线程  低摆动线程只能使用SWING_USE_SPD_MODE==0用雅克比直接映射

#define G_EST_TIME            0.05 //10Hz
#define DEAD_G_ATT            0.3 //degree
#define FLT_GROUND_ATT_EST    3.5   //Hz
#define EN_F_T_G              0

#define TEST_FF_MODE  0 //测试模块前提
#define TSET_F_IMP  0		//力导纳输出测试
#define TSET_F_FF   1		//力前馈输出
#define TSET_F_IF   0		//位力输出
#define TEST_F_FB_OUT 0 //力导纳位置反馈测试

#define STAND_GROUND_CHECK_TEST 1   //使能力控站立下的着地测试
#define SINGLE_LEG_TEST 0						//单腿测试仅支持站立
#define SINGLE_LEG_ID   2					//单腿测试ID

#define EN_GROUND_CHECK 1  			//使能 步态使用 着地判断
#define EN_ATT_GROUND_CONTROL 1 //使能 步态使用 地形估计
#define EN_TORQUE_CONTROL     1 //使能步态力矩输出<<------------------修改这使能机器人

#define GROUND_USE_EST 1		 //使用足底力估计Touch状态 或者使用着地传感器
#define USE_FPOS_CONTROL 1   //使用力控  <<------------------修改这改变空中位置模式

#define Q_NOW_USE_SET 0     //使用当前角度作为反馈

#define MIN_SPD_ST 0.003
#define MIN_SPD_ST_RAD 1
#define T_RST 1.5
#define MAX_FSPD 0.35   //??????????m/s
#define POS_DEAD 0.05  //????????m
#define YAW_POS_MAX 25

#define USE_ESO_OBSEVER   		0//			???ESO????????????
#define ODOM_SPD_MODE         1//			1????????????
#define END_SPD_USE_POS       1//			1????????λ?????  1WS
#define ODOM_SPD_USE_LEG_AV   1//     ???????????????????
//-------------------------------机器人物理参数----------------------
//#define MOCO_ML_LST
#define MOCO_ML
#define FOOT_K       4
#define Www          0.17//跨关节宽度
#define Hw           0.04*1//机体高度质心偏差
#define Hw1          0.04*0//机体高度质心偏差
#define L1w          0.25
#define L2w          0.261
#define L3w          0.022
#define L4w          0.116//跨关节高度到侧展旋转
#define R_W          0.05//足端半径
#define Mw           11.0
#define I_2_Nm       0.0326 //Nm/A 力矩系数
#define KNEE_Q_OFF   0//17


#define SAFE_PITCH 25
#define SAFE_ROLL  25
#define SAFE_Q     3
//-----------------------------------------数学参数--------------------
#define gw           9.8
#define piw          3.1415926
#define rad(x)      (x*piw/180.0)    //将角度化为弧度
#define deg(x)      (x*180.0/piw)    //将弧度化为角度
#define RAD_TO_DEGw  (180.0/piw)//57.3
#define DEG_TO_RADw  (piw/180.0)//0.0173

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

//-------------------------------------------------------------
#define Ls 0
#define Rs 1
#define FL 0
#define BL 1
#define Xr 0
#define Yr 1
#define Zr 2
#define PITr 0
#define ROLr 1
#define YAWr 2
#define YAWrr 3

#define MODE_SPD 1
#define MODE_POS 2
#define MODE_RAD 3
#define MODE_ATT 4
#define MODE_BODY 5
#define MODE_GLOBAL 6
#define MODE_ATT_YAW_ONLY 7
#define MODE_ATT_PR_ONLY 8
#define MODE_FAST_WAY 1
#define NMODE_FAST_WAY 0

#define S_PI 3.14159267
#define RAD_TO_DEG 180/S_PI
#define DEG_TO_RAD S_PI/180

typedef struct
{
    float x;
    float y;
    float z;
}Vect3;

typedef struct
{
    float x;
    float y;
    float z;
    float zz;
}END_POS;


//---------------------------------基头文件--------------------------
typedef struct {
    float fx, fy, fz;
    float slip_timer;
    float slip_sin;
    float slip_F[5];

    END_POS acc;
    END_POS vel;
    END_POS pos;
    END_POS force;
    float kp;
    float kd;
    float l0;
    END_POS foot_base;
    int stance_flag;
}_SLIP;

extern _SLIP slip_mode;

extern float MAX_SPD, MAX_SPD_X,MAX_SPD_Y,MAX_SPD_RAD, MIN_Z, MAX_Z, MIN_X, MAX_X, MIN_Y, MAX_Y;

typedef struct
{
    float pos_now[5];
    float spd_now[5];
    float acc_now[5];
    float Tsw;//=0.25;
    float leg_dis;//=0.5;
    float leg_h;//=0.25;
    float spd_lift;//=0.8;
    float max_spd;//=leg_dis/Tsw;
    float limit_spd;//= max_spd*4;
    char flag[5];
    float T12;//=Tsw*0.2;%???????
    float T45;//=T12;
    float T23;//=(Tsw-T12*2)/2;
    float T34;//=T23;

    float p1[5], p2[5], p3[5], p4[5], p5[5];
    float v1[5], v2[5], v3[5], v4[5], v5[5];
    float a1[5], a2[5], a3[5], a4[5], a5[5];
    float a12[5], b12[5], g12[5];
    float a23[5], b23[5], g23[5];
    float a34[5], b34[5], g34[5];
    float a45[5], b45[5], g45[5];
    float param_x[6], param_y[6], param_z1[6], param_z2[6];
    float param_traj_x[10][6], param_traj_y[10][6], param_traj_z[10][6];
}END_PLANNER;

extern float td_time;
extern float lf_time;
typedef struct
{
    char id, trig_state, ground_state, invert_knee;
    char q_now_use_tar;
    int invert_knee_epos[2];
    float sita1_off, sita2_off, sita3_off;
    float spd_dj[5];
    float lift_spd, td_spd;
    float delta_h, delta_h_att_off;
    float kp_trig;
    float time_trig;
    float sw_time_b;
    float spd_est_cnt;
    END_POS tar_epos, tar_epos_b, tar_epos_n, tar_epos_n_reg, tar_epos_h, tar_epos_h_reg, tar_epos_b_reg;
    int sita_flag[4], sita_flag_ocu[4];
    float st_time_used;
    float sw_time_used;
    END_PLANNER end_planner;

    //-------------------
    float trap_cnt;
    float trap_mov_dis_norm[4];
    float trap_mov_dis[4];
    float trap_mov_spd;
    float trap_mov_spd_norm;
    float trap_sw_t_fix;
    char is_trap;

    //-------------------
    float up_stair_cnt;
    int upstair_cnt;
    int duty_cnt;
    int upstarir_gait_keep_duty;
    char is_up_stair;
    Vect3 pos_stair;
    END_POS stair_pos;
    int vision_sw_enable;
    float swingTimeRemaining;
    float posTimeRemaining[5];
    float y_side;
}PARAM;

typedef struct
{
    float l1, l2, l3, l4, r;
    float tar_sita1, tar_sita2, tar_sita3, tar_sita4, tar_sita5, tar_foot_q_n;
    float sita1, sita2, sita3, sita4, sita5, sita;
    float sita_leg_knee_b,sita_foot_b;
    float sita_leg_knee_n, sita_foot_n;
    float dsita1, dsita2, dsita3, dsita4, dsita5, dsita;
    float alfa, beta;
    float sita_reg[5];
    END_POS epos, epos_b, epos_n, epos_reg, epos_regn, epos_vm, epos_n_b, tar_epos_n_sw, tar_epos_b, tar_epos_b_reg;
    END_POS cog_n;
    END_POS eposc, epos_bc, epos_nc;
    END_POS spd, spd_n, spd_hip_n,spd_b, spd_o;
    END_POS tar_epos_h, tar_epos_h_reg_spd,tar_epos_h_reg, tar_epos, tar_epos_n, tar_spd, tar_pos, tar_spd_sw, slip_epos_n, opt_epos_n;
    char slip_epos_n_flt_init;
    END_POS st_pos, epos_td_n, epos_lf_n, epos_sw_end_n;
    END_POS st_spd;
    END_POS epos_td_hn, epos_lf_hn;
    END_POS epos_spdd_n, epos_spdd_b;
    END_POS epos_sw_st_n, epos_sw_st_b;
    END_POS epos_nn;
    END_POS odom_st;
    END_POS sw_cog_st;
    float tar_h, h, delta_ht;
    float jacobi33[9], ijacobi33[9];
    float jacobi22[4], ijacobi22[4];
    char ground, ground_s, ground_noshock, is_touch, is_touch_mea;
    float ground_force[5];
    float force[5], force_b[5], force_n[5], force_cmd[5];
    int flag_fb, flag_rl;
    float cnt_ss;
    float st_phase, st_phase_timer;
    float sw_phase, sw_phase_timer;
    float td_time_last;
    PARAM param;
    int cube_lock;
}VMC;
//-----------------------------------------------------------

#define RC_REMOTE    0
#define RC_SBUS      1
#define RC_REMOTE_UP 2

#define CUSTOM  0
#define MOCO8_PRO 1
#define MOCO8_MID 2
#define MOCO8_CHEETHA 3
#define MOCO8_LS3 4
#define MOCO12_PRO 5
#define MOCO12_LS3 6
#define MOCO12_SPOT 7
#define MOCO12_SPOTMINI 8
#define MOCO12_HYQ 9
#define MOCO12_ANYMAL 10


#define IDLE  0
#define TROT 1
#define F_TROT 2
#define WALK  3
#define STAND_RC 4
#define STAND_IMU  5
#define STAND_PUSH 6
#define RECOVER 7
#define FALLING 8
#define CLIMB  9
#define WALK_E 10
#define CRAWL 11
#define PRONK  12
#define BOUND 13
#define ROLLING 14
#define G_ETL  15

#define M_SAFE 0
#define M_STAND_RC  1
#define M_STAND_IMU 2
#define M_STAND_PUSH 3
#define M_TROT 4
#define M_F_TROT 5
#define M_WALK 6
#define M_RECOVER 7
#define M_FALLING 8
#define M_CLIMB 9
#define M_WALKE 10
#define M_CRAWL 11
#define M_PRONK 12
#define M_BOUND 13
#define M_C_TROT  14
#define M_ROLLING  15
#define M_ETL  16

#define PARALE 0
#define LOOP 1
#define PARALE_LOOP 2

typedef struct
{
    float pid_pit[5];
    float pid_rol[5];
    float pid_yaw[5];
    float pid_vx[5];
    float pid_vy[5];
    float pid_posxy[5];
    float pid_posz[5];
    float move_com_off[2];
    float move_att_off[2];
    float leg_off[2];
    float side_off[2], side_off_stand[2], side_off_walk[2];
    float stance_xy_kp[2];
    float stance_zoff_kp;
    float stance_time[2];
    float swing_hight;
    float swing_spdkp[2];
    float posz_idle[5];
    float slip_p[5];
    float ground_seek_spd;
}PARAM_VMC;

typedef struct
{
    PARAM_VMC param_vmc, param_vmc_default;
    float test_pos_flt[5], test_att_flt[5];
    int leg_dof;
    int leg_type;
    int robot_type;
    int dj_type;
    int rc_type;
    int robot_mode;
    float sw_com_off[5];
    float cof_off_all[4];
    float end_sample_dt;
    float gain_control_x, move_check_x;
    float safe_sita[5];
    END_POS gait_sw_end_reg_n[4];
    char trot_sw_end_flag[4];
    char param_save, cmd_use_ocu, send_mask;
    char stand_trot_switch_flag;
    char stand_switch_flag[2];
    char stand_switch_cnt[2];
    float ground_force[4][5];//足底传感器
    float encoder_spd[2];
    float cog_off_use[4];
    END_POS tar_spd_use[2], tar_spd_use_rc, climb_off;
    float MAX_Z, MIN_Z, MAX_X, MIN_X, MAX_Y, MIN_Y, MAX_PIT, MAX_ROL, MAX_YAW;
    char en_gait_switch, en_fall_protect;
    char have_cmd, have_cmd_rc[2], have_cmd_sdk[2][4], en_sdk;
    char control_mode, rc_mode[2];
    char cal_flag[5];
    char smart_control_mode[5];//pos/spd   high  att/rad
    int key[10];
    int rc_input[10];
    int is_trap = 0;
    float soft_weight;
}PARAM_ALL;

typedef struct
{
    u8 key_right;
    char side_flip,side_flip_done;
    char force_update_mpc;
    float version[2];
    int your_key[5];
    int lisence_test[5];
    int your_key_all;
    int board_id[4];
    int gait_mode, gait_mode_reg;
    float sita_test[5];
    //---------------------------------------
    char trot_state, trot_phase;
    u8 ground[2][4];
    END_POS tar_pos, tar_spd, tar_spd_rc;
    float tar_att[5], tar_att_rate[5], tar_att_off[5], tar_att_bias[5], ground_off[2], tar_pos_off[5];
    //------------------------------------------
    float kp_trig[2];
    float cog_off[6], off_leg_dis[5];
    //
    float l1, l2, l3, l4, W, H, mess;
    float gait_time[4];
    float gait_alfa;//0~1
    float delay_time[5], gait_delay_time, stance_time, stance_time_auto;
    //
    float att_trig[4], att_ctrl[4], att_rate_trig[5], att_rate_swing[5],att_rate_ctrl[5], att[5];
    float att_rate[5], att_vm_b[5], att_vm[5], att_rate_vm[5], att_vmo[5], acc[4];
    float ground_att[5], ground_att_est[5], ground_att_cmd[5];
    float ground_norm[5], ground_plane_p[5];
    float body_spd[4];
    float body_spd_b[2][4];
    END_POS pos, pos_n, pos_n_fp, pos_vm_leg, pos_vm_leg_n;
    END_POS cog_pos_n, zmp_pos_n, cog_spd_n, zmp_spd_n;
    END_POS pos_n_b[2], spd_n_b[2];

    END_POS spd, spd_n, spd_ng;
    END_POS acc_n, acc_nn, acc_b;

    END_POS odom_st, ankle_pos_b[4],ankle_pos_n[4];
    END_POS spd_n_kf, pos_n_kf , pos_nn_kf,spd_nn_kf;
    END_POS climb_off, cog_n;
    float yaw_force, exp_yaw_rate;
    float Rb_n[3][3];
    float Rn_b[3][3];
    float Rn_b_noroll[3][3];
    float Rb_n_noroll[3][3];
    float Rn_g[3][3];
    float Rg_n[3][3];
    float acc_norm;
    float ground_height, ground_height_spd;
    char ground_num, ground_num_touch, leg_power, power_state;
    char use_ground_sensor_new;
    u8 err, unmove, hand_hold, fall, fly, fall_self;
    u8 trot_air_test;
    char ground_per_trot;
    float ground_per_trot_timer;
    PARAM_ALL param;
    float tar_spd_walk[2];
}VMC_ALL;

extern VMC vmc[4], vmc_virtual[4];
extern VMC_ALL vmc_all;

//---------------------------------------------------------------------------------------------------
typedef struct
{
    long mcuID[4];
    int board_id_test[5];
    int board_license_test[5];
    int board_license_check[5], board_ido[5];
    char key_right;
}_LISENCE;
extern _LISENCE lisens_vmc;
void get_license(void);

typedef struct{
 char en_record;
 char is_touch[4],is_ground[4];
 char force_en_flag[4];
 char leg_state[4];
 END_POS epos_n_tar[4];
 END_POS epos_n_now[4];
 END_POS depos_n_tar[4];
 END_POS depos_n_now[4];
 float sita_tar[4][3];
 float sita_now[4][3];
 END_POS GRF_n_tar[4];
 END_POS GRF_n_now[4];
 float com_n_tar[3];
 float com_n_now[3];
 float dcom_n_tar[3];
 float dcom_n_now[3];
 float ground_att_now[3];
 float att_now[3];
 float datt_now[3];
 float att_tar[3];
 float datt_tar[3];
 float temp_record[10];
}_RECORD;

typedef struct{
    char connect;
    int loss_cnt;
    char mode;
    char up_mode;
    float rc_spd_b[3],rc_rate_b[3];

    char cmd_robot_state;
    float rc_spd_w[2],rc_att_w[2],rate_yaw_w;
    int key_ud,key_lr,key_x,key_y,key_a,key_b,key_ll,key_rr,key_st,key_back,key_ud_reg,key_lr_reg,key_x_reg,key_y_reg,key_a_reg,key_b_reg,key_ll_reg,key_rr_reg,key_st_reg,key_back_reg;
    float curve[20];

    char sbus_conncect;
    char sbus_power_sw,sbus_power_sw_reg;
    int sbus_rc_main[4];
    int sbus_rc_main_reg[4];
    int sbus_mode,sbus_mode_reg;
    int sbus_mode_e,sbus_mode_e_reg;
    float sbus_height,sbus_height_reg;
    float sbus_ch[6];
    int sbus_aux[6];
    int sbus_aux_reg[6];
    int sbus_cal_motor_zero;
    int sbus_motor_init,sbus_motor_init_reg;
    int sbus_cal_mems;
    int sbus_control_mode,sbus_control_mode_reg;
    int sbus_height_switch,sbus_height_switch_reg;
    int sbus_power_off,sbus_power_off_reg;
    int sbus_power_switch;
    int sbus_smart_en;
    int sbus_gait_sel;
    int sbus_auto_forward;
    int sbus_trig_aux_left;
    int sbus_trig_aux_right;
    float sbus_height_rate;
    float sbus_spd_rate[3];
    int cnt_zero_cal;

    //yunzhuo
    int key_a_yun,key_b_yun,key_c_yun,key_d_yun;
    int sel_e_yun,sel_f_yun;
    float rc_g_yun,rc_h_yun;
  _RECORD record;
}_OCU;
extern _OCU ocu;

typedef struct {
    float high_leg_end;
    float gait_duty;
    float max_spd;
    float max_rad;
    float cog_off[2];
    float kp_trig[5];
}_GAIT_PARM;
extern _GAIT_PARM tort_p, walk_p, ftort_p, init_p, bound_p, pace_p, climb_p, custom_gait, crawl_p, pronk_p, bound_p;

//-----------------------------------------机器结构体--------------------------------------------
typedef struct
{
    double roll;       //横滚，x轴
    double pitch;      //俯仰，y轴
    double yaw;        //偏航，z轴
    double yaw_off;
}eulerAngleTypeDef;

typedef struct
{
    double x;
    double y;
    double z;
}robPosTypeDef;

typedef struct
{
  float exp,now,now_reg;
    float err,err_dead,err_reg;
  float p_out;
  float i_out;
  float d_out;
  float max_out;
  float kp_o,kp,ki,kd,vff;
  float kp_o_d[3],kp_d[3],ki_d[3],kd_d[3],vff_d[3];

  float kp_sw,ki_sw,kd_sw;
  float kp_st,ki_st,kd_st;

  float kp_sw_d[3],ki_sw_d[3],kd_sw_d[3];
  float kp_st_d[3],ki_st_d[3],kd_st_d[3];
  char param_sel;
}PIDs;


typedef struct
{
    float kp_pos, ki_pos, kd_pos;
    float kp_force, ki_force, kd_force;
}POS_FORCE_P;

typedef struct
{
    float st_lf, st_td;
    float trot_lf, trot_sw, trot_td;
    int check_td, check_lf;
    float check_spd;
}_TD_CHECK_PARAM;

//单腿信息结构体
typedef struct
{
    int id;
    Vect3 epos_h, epos_h_reg;
    Vect3 tar_epos_h, tar_epos_n;
    Vect3 force_err_n, force_imp_spd_h, force_imp_spd_n;
    float pos_taod[5], fb_force_taod[5], ff_force_taod[5];

    Vect3 epos_b;
    Vect3 epos_n;
    Vect3 epos_bc;
    Vect3 epos_nc;

    Vect3 espd_h, espd_n;

    Vect3 tar_espd_h;
    Vect3 tar_espd_n;

    Vect3 tar_force_h;
    Vect3 tar_force_n;
    Vect3 tar_force_dis_n, tar_force_dis_g;
    Vect3 tar_force_dis_n_reg;

    Vect3 force_est_h;
    Vect3 force_est_n;

    Vect3 tau_est_h_output;

    Vect3 force_est_h_output;
    Vect3 force_est_n_output;
    float force_est_n_length, force_est_h_length,espd_norm_b;

    Vect3 dforce_est_h_output;
    Vect3 dforce_est_n_output;
    Vect3 tar_force_dis_n_qp;
    Vect3 tar_force_dis_n_mpc;
    Vect3 tar_torque_dis_n_mpc;
    Vect3 force_est_h_output_reg;
    Vect3 force_est_n_output_reg;
    Vect3 F_n_ff;

    Vect3 exp_pos_b[3],exp_pos_n_b[3];
    PIDs q_pid_sw,q_pid_st;
    PIDs f_pos_pid_st[3],f_pos_pid_sw[3];
    PIDs f_pid_st[3],f_pid_sw[3];
    PIDs q_pid,f_pid[3],f_pos_pid[3];


    float sita[5], sita_reg[5], sita_r, r, alfa, beta;
    float sitaf[5];
    float limit_sita[5];
    float sita_d[5], sita_dd[5];
    float err_sita[5];
    float tar_sita[5];
    float tar_sita_d[5];

    float jacobi[9], jacobi_inv[9];

    float tar_sita_up[3];
    float tar_sita_d_up[3];

    int flag_fb, flag_rl;

    float pos[5];         //测量得到的当前关节角度            弧度制
    float taom[5];        //实时测量的扭矩
    float tao_bias[5];        //实时测量的扭矩
    float taom_output[5];        //实时测量的扭矩
    float taod[5];        //期望扭矩
    float taod_mess[5];
    float taod_ff[5];        //期望扭矩
    float w_force_taod[5];
    float tao_q_i[5];
    float limit_tao[5];
    char  is_ground;
    char  is_touch, is_touch_est;       //是否触地 true : 触地 false : 离地
    _TD_CHECK_PARAM touch_z_param;

    int touch_cnt[5][2];
    int touch_cnto[5];
    int trig_state, ground_state, st_torque_need_init;
    Vect3 st_pos;
    Vect3 epos_td_n, epos_lf_n;
    Vect3 epos_sw_end_n;
    Vect3 epos_td_hn, epos_lf_hn;
    Vect3 epos_spdd_n, epos_spdd_b;
    Vect3 epos_sw_st_n, epos_sw_st_b;
    Vect3 epos_nn;
    Vect3 odom_st;
    float delta_ht;
    float time_trig;
    float cnt_ss;

    float contactStates;
    float swingStates;
    int mpc_table[20];
    int mpc_table_o[20];
    int mpc_table_s[20];
}LegTypeDef;

typedef struct {
    char en_force_control_cal, en_force_control_out;
    PIDs q_pid_sw, q_pid_st_trot, q_pid_st_stance,q_pid_init;
    PIDs f_pos_pid_st[5];
    PIDs f_pid_st[5];
    PIDs zeros;
    float load_fz;
    float td_before_fz;
    float motor_i_b[2];
    //---
    _TD_CHECK_PARAM touch_z_param_st;
    _TD_CHECK_PARAM touch_z_param_sw;
    _TD_CHECK_PARAM touch_z_param_td;
    _TD_CHECK_PARAM touch_z_param_trot_st;
    _TD_CHECK_PARAM touch_z_param_bound_st;
    _TD_CHECK_PARAM touch_z_param_pronk;
    _TD_CHECK_PARAM touch_z_param_climb;
    _TD_CHECK_PARAM touch_z_param_bound;
    //--
    float t_to_i;
    float max_t,max_t_d[3];
    float max_i;

}POS_FORCE_PARM;

extern POS_FORCE_PARM pos_force_p;

typedef struct {
    PIDs pos_x, pos_y, pos_z;
    PIDs spd_x, spd_y, spd_yaw;
    PIDs att_pit, att_rol, att_yaw;

    PIDs att_pit_trot, att_rol_trot, att_yaw_trot, att_rol_bound, att_yaw_bound;
    PIDs att_pit_walk, att_rol_walk, att_yaw_walk;
    PIDs spd_x_walk, pos_z_trot, pos_x_walk, pos_z_walk;
    Vect3 stand_off;
    float mess_scale;
    float sw_deltah;
    float ground_mu;
}VMC_ROBOT_PARM;

extern VMC_ROBOT_PARM vmc_robot_p;

//机器人结构体 包含整个机器人的所有信息
#define ARM_M_H 0
#define ARM_M_N 1
#define ARM_M_E 2

#define HEAD_FREE 0
#define HEAD_LOOK_B 1
#define HEAD_LOOK_E 2
#define HEAD_RECODE 3
typedef struct
{
    char gait_level;
    char beep_state;
    LegTypeDef  Leg[4];
    LegTypeDef  Leg_virtual[2];//双足虚拟腿
    Vect3 vect3_zero;
    eulerAngleTypeDef   IMU_now, IMU_now_o, IMU_now_e_off,IMU_dot_o;        //机器人当前欧拉角
    eulerAngleTypeDef   IMU_last;       //机器人上次欧拉角
    eulerAngleTypeDef   IMU_dot;        //机器人欧拉角速度
    eulerAngleTypeDef   ground_att;

    eulerAngleTypeDef   exp_att;
    eulerAngleTypeDef   now_att;
    eulerAngleTypeDef   now_rate;
    eulerAngleTypeDef   now_rate_reg;
    eulerAngleTypeDef   exp_rate;
    Vect3               exp_spd_b;
    Vect3               exp_pos_b;
    Vect3               exp_spd_n,exp_spd_ng;
    Vect3               exp_pos_n;
    Vect3               cog_spd_b;
    Vect3               cog_pos_b;
    Vect3               cog_spd_n;
    Vect3               cog_pos_n;
    Vect3				cog_pos_nn;
    Vect3				cog_F_ff;
    Vect3				cog_T_ff;
    Vect3               cog_acc_n,cog_acc_b;

    int ground_num;
    int ground_num_rl[2];
    int ground_num_fb[2];

    Vect3 exp_force;
    Vect3 exp_torque;
    Vect3 exp_torque_inv;
    Vect3 exp_force_i;
    Vect3 exp_torque_i;
    //
    Vect3 exp_pos_n_b[2];
    Vect3 exp_force_b[2];
    Vect3 exp_torque_b[2];
    Vect3 exp_force_i_b[2];
    Vect3 exp_torque_i_b[2];

    Vect3 max_force,min_leg_force,max_leg_force;
    Vect3 max_torque_foot;
    Vect3 max_err_force;
    Vect3 max_torque;

    float mess_est;
    float mess_payload;
    float mess_off[5];
    float I_off[5];
    float Rn_b_noroll[3][3];
    float Rb_n_noroll[3][3];
    float Rn_g[3][3];
    float Rg_n[3][3];
    float MIN_X;
    float MAX_X;
    float MIN_Y;
    float MAX_Y;
    float MIN_Z;
    float MAX_Z;
    //param
    int use_ground_sensor;
    float gait_time;
    float stance_time;
    float kp_trig[2];
    float leg_off[2];
    Vect3 ankle_pos_n[4], cog_spd_nn;
    _OCU ocu;

    int _nIterations;
    float dtMPC;


    //
    int start_record;
    int reset_z_mode;
    int soft_reset;
    int record_mode;
    int gait_mode;
    int state_gait;
    int cmd_robot_state;
    int robot_mode;
    int yaw_lock;
    int arm_control_mode;
    int head_control_mode;
    int wheel_control_mode;
    int grasp_action;
    int grasp_mission;
    int reach_waypoint;
    END_POS base_vel_b_exp;
    float base_rate_exp;
    int reset_arm;
    float max_epos_z;
    float min_epos_z;
    float max_q_z;
    float min_q_z;

    int good_waypoint;
    END_POS tar_waypoint;
    END_POS tar_waypoint_reg;
    END_POS way_point[10];
    END_POS base_vel_b_exp_rc;
    float base_rate_exp_rc;
    END_POS base_vel_b_exp_rc_flt;
    float base_rate_exp_rc_flt;

    END_POS base_pos_n_w,base_pos_n,base_pos_n_off;
    END_POS base_vel_b_w;
    END_POS base_vel_n_w;
    END_POS base_vel_b_w_fushion;
    END_POS base_vel_n_w_fushion;
    float base_rate_w;
    float base_rate_imu;
    float base_rate_fushion;
    float base_vel_sdk[3];

    END_POS arm_epos_h_exp,arm_epos_h_exp_flt,arm_epos_h_exp_reg;
    END_POS arm_epos_h,arm_epos_h_reg;
    END_POS arm_epos_h_grasp,arm_epos_h_grasp_off,arm_epos_h_grasp_reg;
    END_POS arm_epos_h_grasp_app;
    END_POS arm_att_h_exp,arm_att_h_exp_flt;
    END_POS arm_att_h;

    END_POS arm_epos_b_exp;
    END_POS arm_epos_b;
    END_POS arm_att_b_exp;
    END_POS arm_att_b;
    END_POS arm_att_b_grasp,arm_att_b_grasp_app;

    END_POS arm_epos_e_exp;
    END_POS arm_epos_n_exp;
    END_POS arm_epos_n;

    END_POS arm_epos_ny_exp;
    END_POS arm_epos_ny;

    END_POS arm_att_n_exp;
    END_POS arm_att_n;

    END_POS arm_vpos_h_exp,arm_vpos_b_exp;
    END_POS arm_vpos_h,arm_vpos_h_flt;
    END_POS arm_vpos_b;
    END_POS arm_vpos_n,arm_vpos_ny;

    END_POS tf_b2h;

    END_POS head_tf_h2head,head_pos_h,head_pos_b,head_pos_n,head_pos_e;
    END_POS head_look_pos_h,head_look_pos_b,head_look_pos_n,head_look_pos_e;

    float arm_q_exp_sdk[2][6];
    float arm_q_exp[2][6];
    float arm_q_exp_flt[2][6];
    float wheel_dq_exp[4];

    int wheel_touch_cnt;
    int wheel_touch[4];
    float wheel_q[4],wheel_q_reg[4];
    float wheel_dq[4];
    float wheel_v[4];

    float wheel_h;
    float wheel_w;
    float wheel_r;
    float base_z;

    float head_att_exp[2];

    float cap_set;
    float cap_set_sdk[2];
    float cap_set_flt;

    float arm_base_height_exp;
    float arm_base_height_exp_flt;
    float arm_base_height,arm_base_height_reg,arm_base_dheight,arm_base_tau;
    float q_reg_based_height;
    float dq_2_dv_base;
    float dv_2_dq_base;
    float dtau_t_basetau;

    float arm_q[2][6];
    float arm_dq[2][6];
    float arm_q_reg[2][6];
    float arm_t[2][6];

    float Rb_n_noyaw[3][3];
    float Rb_n[3][3];
    float Rn_b_noyaw[3][3];
    float Rn_b[3][3];
    //arm
    float Rn_b_noatt_grip[3][3];
    float Rb_n_noatt_grip[3][3];
    float Rb_h_noatt_grip[3][3];
    float Rh_b_noatt_grip[3][3];

    float safe_arm_q[2][6];
    float safe_height;
    float base_height;

    float e_yaw_off=0;
}robotTypeDef;
extern robotTypeDef robotwb;

typedef struct
{
   char reset_world;
   END_POS can_pos[10];
   END_POS chair_pos[10];
   END_POS base_pos;
}_SIM_DATA;
extern _SIM_DATA _sim_webots;

typedef struct {
    char state_m;
    char need_trig, en_fast_cog_move;
    char area_too_small;
    char cog_stable;
    float cog_stable_timer;
    char cog_tar_reach;
    char ground_num;
    char pre_trig_id, trig_id, trig_id_last;
    char trig_record_cnt;
    char trig_id_his[4];
    char xy_control_mode;
    float stable_value;
    float min_st_value;//质心稳定阈值死区
    float stable_bind_width;//支撑三角形稳定边界
    float cog_reach_dead, cog_reach_deadnn;//质心控制死区
    float work_space_band;//工作空间边界
    float stable_band;
    float work_space[4];
    char out_ws_flag[4];
    float att_com_off[2];
    float support_area;
    float min_support_area;
    float support_area_check;
    float tar_spd_length;
    float tar_spd_yaw;
    float sw_h_off_walke;
    END_POS cord_n;//全局运动坐标系复位位置
    END_POS now_cog_nn;//目前支撑平面全局质心
    END_POS tar_zmp_nn;
    END_POS tar_zmp_n, tar_zmp_b;//质心参数(坐标系在质心)
    END_POS now_zmp_n, now_zmp_b;
    END_POS now_zmp_nn;
    END_POS now_cog_n, now_cog_b;
    END_POS leg_move_per_trig;
    END_POS leg_off_move;
    END_POS now_st_cog_n, now_st_cog_b;//支撑区域参数
    END_POS now_st_cog_n_reg, now_st_cog_b_reg;//支撑区域参数
    END_POS now_st_cogspd_n, now_st_cogspd_b;//支撑区域参数
    END_POS now_st_cog_n1, now_st_cog_b1;//支撑区域参数
    END_POS walk_cmd_trig;
    END_POS sw_cog_st_reg;
    END_POS sw_off;
    float imp_force_time_rate;
    float imp_alfa[5];
}_GAIT_WALK;
extern _GAIT_WALK walk_gait;

typedef struct
{ //phase
    char ref_leg;
    float timer[2];
    float t_ref;
    float t_d;
    float ti[4];
    float T_all;
    float T_sw;
    float T_st;

    float dS[4];
    float S_target[4];
    float S_now[4];
    float S_st[4];
    float S_sw[4];
    float S_sw_ref_reg, S_st_ref_reg;
    float sw_start[4], sw_end[4];//NEW
    int ground_flag[4][2];
    float tg;
    float tg_st;
    float w_gait_change;
    float t_gait_change;
    float min_st_rate;
    int gait_mode[2];
    float imp_force[4][5];
    //event
    char state[4];
}GAIT_SHEC;

extern GAIT_SHEC gait;
extern GAIT_SHEC gait_etl;

typedef struct
{
    char connect,connect_motor[6];
    char ready[6];
    int loss_cnt;
    char reset_q,reset_err;
    char motor_en,motor_en_reg;
    char motor_mode,motor_mode_reg;
    char err_flag[2];
    float bat_v[2];
    float temp[2];
    float q_now[6],qd_now[6],qdd_now[6],q_bias[6];
    float q_set[6],qd_set[6],qdd_set[6];
    float f_now[6];
    float f_set[6];
    float t_now[6];
    float q_reset[6];
    float set_t[6],set_t_flt[6];
    float set_i[6],set_i_flt[6];
    float max_t[6];
    float max_i[6];
    float t_to_i[6];
    char can_bus_id;
}_MOTOR;
extern _MOTOR leg_motor[5];
extern _MOTOR arm_motor[2];
extern _MOTOR head_motor;
extern _MOTOR wheel_motor;

#endif
