#ifndef __LOCOMOITON_H__
#define __LOCOMOITON_H__
#include "base_struct.h"
#include "include.h"
#if RUN_PI&&0
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/parse.h>

extern YAML::Node config_robot;
extern YAML::Node config_gait;

extern float Www;
extern float Hw;
extern float L1w;
extern float L2w;
extern float L3w;
extern float Mw;
extern float I_2_Nm;
#endif

void* Thread_UDP_Webots1(void*);// as ��������Ϊ
void* Thread_UDP_Webots2(void*);// as ��������Ϊ
int ik_pino(END_POS end_pos,END_POS end_att,float q_out[6]);
int fk_pino(float q_in[6],END_POS* end_pos,END_POS* end_att);
//----------------------------------------------VMC controller---------------------
void body_traj_planner(float dt);
void body_servo_control(float dt);
void force_dis_n(void);
void force_dis_n_side(void);
void force_dis_n_side_LR(void);
void reset_servo_interge(void);

//---------------------------------------------hardware interface---------------------
void subscribe_imu_to_webot(robotTypeDef* rob,float dt);
void subscribe_webot_to_vmc(float dt);
void subscribe_webot_to_vmc1(float dt);
void subscribe_webot_to_vmc2(float dt);
void subscribe_webot_to_vmc3(float dt);
void subscribe_webot_to_vmc4(float dt);
void publish_vmc_to_webot(float dt);
void readAllMotorPos(robotTypeDef* rob,float dt);
void set_motor_q(int id);
void set_motor_t(int id);
void pos_control_pd(float dt);
#if RUN_WEBOTS
#define NO_RC 1
void updateJoy(void);
void recorder(float dt);
void record_thread(char sel, float dt);
int updateKey(void);
#endif
//---------------------------------------------Force IMP controller----------------------
void force_control_and_dis_stand(float dt);
void force_control_and_dis_pose(float dt);
void force_control_and_dis_climb(float dt);
void force_control_and_dis_trot(float dt);
void force_control_and_dis_trot_air(float dt);
void force_control_and_dis_trot_local(float dt);
void force_control_and_dis_bound(float dt);
void force_control_and_dis_rolling(float dt);
//---------------------------------------------state estimator-------------------------
typedef struct {
    END_POS acc_n_m;
    END_POS acc_n_f;
    END_POS acc_nn_f;

    END_POS odom_spd_n;
    END_POS odom_pos_n;

    END_POS odom_spd_nn;
    END_POS odom_pos_nn;

    END_POS odom_dspd_n;
    END_POS odom_dpos_n;

    END_POS odom_spd_n_r;
    END_POS odom_pos_n_r;
    float Kpv;
}_ODOM_1;
extern _ODOM_1 odom1;

typedef struct
{
    int state_gait;
    int auto_switch, auto_mode;
    float switch_timer[3];
}Gait_Mode;

extern Gait_Mode gait_ww;
extern char sit_down_flag;

void reset_robot_statement(void);
void touchdown_check(robotTypeDef* rob, float dt);
void estimate_GRF(float dt);
void state_estimator(float dt);
void estimate_ground_att(float dt);
void pay_load_estimation(float dt);
//---------------------------------------------hardware interface---------------------
void subscribe_imu_to_webot(robotTypeDef* rob, float dt);
void subscribe_webot_to_vmc(float dt);
void publish_vmc_to_webot(float dt);
void readAllMotorPos(robotTypeDef* rob, float dt);
void set_motor_q(int id);
void set_motor_q_sel(int id);
void set_motor_t(int id);
void set_motor_q_up(int id);

void set_motor_w(void);
void set_motor_arm(void);
void set_motor_head(void);
void set_motor_cap(void);
//---------------------------------------------leg planner--------------------------------
void cal_vir_leg_pos(VMC* in, END_POS *td_vir_n, END_POS *lf_vir_n);
float end_pos_y_correction(VMC* in, float ep_slip_n, float dt);
void trig_curve_global(VMC *in, float *x, float *y, float *z, float *vx, float *vy, float *vz, float dt);
void espd_to_qspd(VMC *in, END_POS spd_end, float* spd_q, float dt);
void reset_sw_torque_as_now(VMC *in, float torque_now[3]);
void trig_plan(char leg_sel_trig, float dt);
char trig_lift(char leg_sel_trig, float dt);
char trig_swing(char leg_sel_trig, float dt);
char trig_td(char leg_sel_trig, float dt);
void swing_spd_control(char leg_sel_trig, float dt);
void reset_tar_pos(char sel,char init_q);
void trig_plan_online(char leg_sel_trig, float dt);
extern float sw_t[4];
extern float st_t[4];
//---------------------------------------------locomotion SFM--------------------------
typedef struct {
    char sdk_connect;
    int sdk_loss_cnt;
    char sdk_mode;
    char gait_mode;
    char cmd_power;
    char trig_mode;
    float cmd_vx;
    float cmd_vy;
    float cmd_vz;
    float cmd_vyaw;
    float cmd_x, cmd_y, cmd_z;
    float cmd_pit;
    float cmd_rol;
    float cmd_yaw;
    int cmd_pwm[12];
    float cmd_angle[4][3];
    END_POS trig_end_cmd[4];
}_SDK;
extern _SDK sdk;

extern char stand_force_enable_flag[5];
void vmc_param_init(void);
char safe_check(float dt);
void locomotion_sfm(float dt);
char gait_switch(float dt);

//-------------------------------------Gati  APP-----------------------------
typedef struct
{
    int state;
    int jump_trig;
    float timer[3];
    float jump_x_dis, jump_x_side_off;
    float jump_yaw;
    float jump_w;
    float jump_height;
    float jump_height_off;
    float jump_height_off_forward;
    float jump_power;
    float jump_avg_F[3];
    float jump_avg_T[3];
    float st_height;
    float ch_height;
    float couch_height;
    float damp_height;
    float sw_z;
    float stand_off;
    float lf_time;
    float couch_time;
    float td_time;
    float fly_time, fly_time_real;
    float lf_spd[3];
    float td_spd[3];

    float kp[3];
    float kp_T[3];
}Gait_Pronk;

extern Gait_Pronk gait_pronk;


typedef struct
{
    int state;
    int jump_trig;
    int jump_phase;
    int jump_phase_other;
    int force_control_mode;
    int up_stair;
    float timer[3];
    float jump_spdx;
    float jump_yaw;
    float jump_w;
    float jump_height;
    float jump_height_off;
    float jump_power;
    float jump_avg_F[3];
    float jump_avg_T[3];
    float st_height;
    float ch_height;
    float couch_height;
    float damp_height;
    float lf_time;
    float couch_time;
    float td_time;
    float fly_time, fly_time_real;
    float lf_spd[3];
    float td_spd[3];

    float kp[3];
    float fp[3];
    float kp_T[3];
}Gait_Climb;

extern Gait_Climb gait_climb;


typedef struct
{
    int state_all;
    int first_bound;
    float sw_dh;
    float T_sw;
    float T_st_norm;
    float T_air;
    float k_time;
    float L;
    float vd;
    float exp_z;
    float c;
    float mess;
    float s_peak_Fz;
    float tauP;
}Gait_Bound_IMP_Param;



typedef struct
{
    int state;
    int phase;
    int phase_rl;
    int other_phase;
    int wait_trig;
    float wait_trig_time;
    char touch, ground;
    char id_sw[2];
    END_POS hip_n_exp;
    END_POS hip_n_now;
    END_POS hip_nb_now;
    END_POS dhip_n_exp;
    END_POS dhip_n_now;
    END_POS sw_off;
    float sw_time;
    float att_exp[3];
    float att_now_rate[3];
    float att_now[3];
    float pid_hip[3];
    float pid_hip_x[3];
    float pid_v[3];
    float pid_P[3];
    float pid_R[3];
    float k_jump;
    float T, T_tirg;
    float t;
    float t_start;
    float s, s_sw;
    float T_st_norm;
    float T_st_now, T_st_now_trig;
    float T_st_last;
    float T_sw;
    float T_air;
    float alfa_Fz;
    float alfa_tauP;
    float Fz_imp;
    float TauP_imp;
    float g_st;
    float Fx;
    float Fz;
    float v_inter;
    float time_fly_near;
    float time_fly_near_last;
    float time_phase_off;
}Gait_Bound_IMP;


typedef struct
{
    Gait_Bound_IMP_Param slip_param;
    Gait_Bound_IMP slip[2];
    Gait_Bound_IMP slip_leg[4];
    Gait_Bound_IMP slip_all;
    int state;
    int jump_trig;
    int jump_phase;
    int jump_phase_other;
    int force_control_mode;
    int up_stair;
    float timer[3];
    float jump_spdx;
    float jump_yaw;
    float jump_w;
    float jump_height;
    float jump_height_off;
    float jump_power;
    float jump_avg_F[3];
    float jump_avg_T[3];
    float st_height;
    float ch_height;
    float couch_height;
    float damp_height;
    float lf_time;
    float couch_time;
    float td_time;
    float fly_time, fly_time_real;
    float lf_spd[3];
    float td_spd[3];
    float side_off[2];
    float kp[3];
    float fp[3];
    float kp_T[3];
    float kp_Y[4];
    float yaw_T;
    float yaw_i_T;
    float yaw_control_step;
    float yaw_rate_reg;
}Gait_Double_SLIP;
extern Gait_Double_SLIP gait_bound;

void  Gait_ETL_Active(void);
void  Gait_ETL_Update_v1(float dt);

void Gait_Walk_Active(void);
void Gait_Walk_Update_v1(float dt);

void Gait_Rolling_Active(char way_sel);
void Gait_Rolling_Update_v1(float dt);

void  Gait_Stand_Active(void);
void  Gait_Stand_Update(float dt);

void  Gait_Trot_Active(void);
void  Gait_Trot_Update_v1(float dt);

void  Gait_Recovery_Active(void);
void  Gait_Recovery_Falling(float dt);
void  Gait_Recovery_Update(float dt);

void  Gait_Pronk_Active(void);
void  Gait_Pronk_Update(float dt);

void  Gait_Climb_Active(void);
void  Gait_Climb_Update(float dt);

void  Gait_Bound_Active(void);
void  Gait_Bound_Update(float dt);

void  Gait_Trot_Active(void);
void  Gait_Trot_Update_v1(float dt);

void  Gait_FTrot_Active(void);
void  Gait_FTrot_Update_v1(float dt);

void  Gait_Recovery_Active(void);
void  Gait_Recovery_Falling(float dt);
void  Gait_Recovery_Update(float dt);

char move_joint_to_pos(VMC * in, int joint, float tar_angle, float max_spd, float dt);
char move_joint_to_pos1(VMC * in, int joint, float tar_angle, float max_spd, float err_check, float dt);
char move_joint_to_pos_sel(VMC * in, int joint, float tar_angle, float max_spd, float err_check, char sel, float dt);
char move_joint_to_pos_sel1(robotTypeDef * in, int joint, float tar_angle, float max_spd, float err_check, char sel,float dt);
void move_joint_with_spd(VMC * in, int joint, float tar_spd, float dt);
char check_lisence(void);

void moco8_ml_lst(void);
void moco8_webots(void);
void moco8_ml(void);


//----------------------Vision mapper------------------
#define FMAP_NUM 3
#define MAP_W 80//�̵߳�ͼ
#define MAP_H 80

#define FMAP_W 40// �˹���ͼ ��������
#define FMAP_H 40

#define GMAP_W 100//�²��ͼ
#define GMAP_H 100
#define MAP_SIZE 0.02
#if 0//traj ͬ���޸�
#define CUBE_NUM 28//28
#define CUBE_W 0.25

#define WAY_POINT_NUM 5//5
#else
#define CUBE_NUM 50+1//28
#define CUBE_W 0.25

#define WAY_POINT_NUM 14//5
#endif
typedef struct
{
    END_POS map_origin_pos_b;
    float map[GMAP_H][GMAP_W];
    END_POS grid_pos[GMAP_H][GMAP_W];
}_HEIGHT_MAP;
extern _HEIGHT_MAP fake_map_stair_12cm, fake_map_trench_5cm, guass_map;

#define MAP_W_N 5600
#define MAP_H_N 5600
typedef struct
{
    char init_odom;

    END_POS robot_cog;
    END_POS robot_cog_off_real;
    END_POS robot_vel;
    END_POS leg_pos[4];
    END_POS leg_tar[4][20];
    float robot_att[3];
    float h_grid[MAP_H_N][MAP_W_N];
    END_POS h_grid_pos[MAP_H_N][MAP_W_N];
    //������ĵľֲ���ͼ
    float h_grid_local[MAP_H][MAP_W];
    float h_grid_local_good[MAP_H][MAP_W];
    float h_grid_local_edge[MAP_H][MAP_W];
    END_POS grid_pos_local[MAP_H][MAP_W];
    float stand_map_z_off;
    END_POS way_point[20];
    END_POS flag_n[20];//������α�־
}_GLOBAL_MAP;
extern _HEIGHT_MAP fake_map_stair_12cm, fake_map_trench_5cm,guass_map;
extern _GLOBAL_MAP global_map;
int fake_map_init(_HEIGHT_MAP* map_fake, float x_off_n, float y_off_n, float rotate_yaw, char map_sel);
int fake_map_link_init(void);
typedef struct
{
    END_POS leg_sw_tar[4];
    END_POS leg_sw_slip[4];
    int ground[4], touch[4];
    END_POS tar_grf[4];
    END_POS mu_limit[4];
    double force_rotate[4][4];
    double force_size[4];
    double mu_rotate[4][4];
    double mu_size[4];
    double mu_r[4];
    END_POS sw_tar_traj[4][1024];
    END_POS sw_tar_traj_webots[4][1024];
    int sw_traj_cnt[4];
    END_POS cog_pos;
    float cog_att[3];

    float grid_m[328][328];
    float grid_m_good[328][328];
    float grid_m_edge[328][328];
    float fgrid_m[FMAP_NUM][328][328];
    float grid_m_base_z;
    int en_draw_map;

    END_POS zmp_pos;
    END_POS tar_zmp_pos;
    END_POS support_center;
    //human
    float grid_wide;
}WEBTOS_DRAWING;


extern WEBTOS_DRAWING webots_draw;

typedef struct
{

    END_POS cog_real;
    END_POS leg_end_real[4];
    END_POS fake_map_cog[10];
    double fake_map_ori[10][4];

    END_POS cube_cog[CUBE_NUM];
    double cube_ori[CUBE_NUM][4];

    END_POS flag_pos[20];
    END_POS way_point_pos[20];
}WEBTOS_DRAWING_RX;
extern WEBTOS_DRAWING_RX webots_draw_rx;

void share_memory_drawing(void);
void move_robot_on_map_n(float dt);
int location_select_on_map(char sel, END_POS slip_n, char mode, END_POS *location);

#define FMAP_USE_SIMU 1
#define POS_USE_SIMU 1

//----------------------Hunman mode------------------
typedef struct
{
    int env_type;//��������
    float complus;//���Ӷ�
    float slape;//������᫶�
    float cute;//���θ��Ѷ�
    float slope[2]; //�¶�
    float mu;//������
    float deep;//���ݶ�
}_HU_ENV;
extern  _HU_ENV _hu_env;

typedef struct
{
    int mode_en;
    int gait_mode;
    int gait_switching;
    int way_point_id;
    int en_traj_follow;
    float cmd_att[3];
    float cmd_spd[3];
    float cmd_spd_limit[3];
    float cmd_height;
    float decide_range;//ע���� m
    float local_map_grid;//�ֲ���ͼ����
    float gait_duty;
    float gait_delay;
    float sw_z;

    int phase_mode;//0 lsm 1elt
    int balance_mode;//0 vmc 1 mpc
    int perdict_win;//Ԥ�ⴰ��

    int stair_slope_mode_flag;//0 normal   1 б��    2¥��
    int slip_vision_mode_flag;//0 �ڲ�ģ�� 1�ⲿģ�� 2�����л�
    int imp_ff_mode_flag;     //0 ��ģʽ   1���ģʽ

    int hu_mode;//��������ģ��
    int fake_map_guass;
    int guass_map_type;

    float stable_rate;//�ȶ���
    float power;//����ͳ��
    float cpu_used;//��������

    float control_rate_mid;//�������Ƶ��
    float control_rate_low;//ĩ�ҿ���Ƶ��

    float time_trig[30];//ʱ���
    float time_trig_reg[30];
    float dt_app[30];
}_HU_MODE;
extern  _HU_MODE _hu_model;

#define HU_TASK1 0 //�����л�
#define HU_TASK2 1 //�Ӿ�����
#define HU_TASK3 2 //̨��б��
#define HU_TASK4 3 //÷��׮

#define HU_TASK  HU_TASK4

#define ENV_1 0//����
#define ENV_2 1//�ݵ�
#define ENV_3 2//��ʯ
#define ENV_4 3//����ʯ
#define ENV_5 4//����
#define ENV_6 5//ɳ��
#define ENV_11 6//���� foot trap
#define ENV_12 7//���� foot trap+ fake map

#define ENV_13 8//���� б��
#define ENV_14 9//���� ̨��

#define G_JOG_V   0
#define G_TROT_V  1
#define G_CTROT_V 3
#define G_DWALK_V 5
#define G_PACE_V  6
#define G_STAND   99

#define HU_MODE1  1 //��������ģ��1 ������֪�ع滮
#define HU_MODE2  2 //��������ģ��1 Ϊ�߳�ͼ+������֪

int brain_planner(float dt);
int env_modeling(float dt);//������ģ
int behavior_change(float dt);//��Ϊ�ݻ�
int balance_change_big(float dt);//��������������ݻ�
int balance_change_small(float dt);//��������ϸ�����ݻ�
int leg_change_big(float dt);//֫������������ݻ�
int leg_change_small(float dt);//֫������ϸ�����ݻ�
int human_thread(float dt);

#endif
