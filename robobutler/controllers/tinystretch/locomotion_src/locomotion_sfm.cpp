#include "include.h"
#include "locomotion_header.h"
#include "gait_math.h"
#include "eso.h"

Gait_Mode gait_ww;
POS_FORCE_PARM pos_force_p;
VMC_ROBOT_PARM vmc_robot_p;
VMC vmc[4];
VMC vmc_virtual[4];
VMC_ALL vmc_all;
ESO att_rate_eso[3];
_SLIP slip_mode;
_OCU ocu;
_SDK sdk;
_MOTOR leg_motor[5];
_MOTOR arm_motor[2];
_MOTOR head_motor;
_MOTOR wheel_motor;

float MIN_Z = -0.1;
float MAX_Z = -0.19;
float MIN_Y = -0.1;
float MAX_Y = -0.1;
float MIN_X = -0.15;
float MAX_X = 0.15;
float MAX_SPD = 0;
float MAX_SPD_X = 0;
float MAX_SPD_Y = 0;
float MAX_SPD_RAD = 50;
char state_pass = 1;
char test_pos_st = 0;//测试标志位
float test_st_exp[6] = { 8*0,
                         10*0,
                         0.05*0,//X
                         0.12*0,//Z
                         0.03,
                         1 };//P R X Z SPD
char stand_force_enable_flag[5] = { 0 };

void vmc_param_init(void)
{
    int i, j;
    float scale_x = 1;
    float scale_z = 20;
    float pos_gain_scale_st = 1;
    float pos_gain_scale_sw = 1;

    robotwb.tf_b2h.x=0.124;
    robotwb.tf_b2h.y=-0.01;
    robotwb.tf_b2h.z=0.048;

    robotwb.base_height=0.15;

    robotwb.wheel_h=0.301;
#if RUN_WEBOTS
    robotwb.wheel_w=0.303*1.2;
#else
    robotwb.wheel_w=0.303;
#endif
    robotwb.wheel_r=0.0535;//m

    robotwb.base_z=0.1;//m

    robotwb.head_tf_h2head.x=0.066;
    robotwb.head_tf_h2head.y=0.068;
    robotwb.head_tf_h2head.z=0.089;
    robotwb.head_tf_h2head.zz=0.051;

    robotwb.cap_set=1;//1open 0~1

    robotwb.yaw_lock=0;

    robotwb.e_yaw_off=robotwb.IMU_now.yaw;

    robotwb.dq_2_dv_base=0.00035;//0.0008;//<--------------doghome
    robotwb.dv_2_dq_base=1000;
    robotwb.dtau_t_basetau=10;
    //{ 45,89,45,-89,45 };
    robotwb.safe_height=0.02;
    robotwb.safe_arm_q[0][0]=-70;
    robotwb.safe_arm_q[0][1]=160;
    robotwb.safe_arm_q[0][2]=0;
    robotwb.safe_arm_q[0][3]=0;
    robotwb.safe_arm_q[0][4]=0;


    robotwb.max_q_z=robotwb.max_epos_z=0.9+robotwb.base_height;
    robotwb.min_epos_z=0;
    robotwb.min_q_z=0.0;
    for(int i=0;i<4;i++)
    {
        robotwb.wheel_touch[i]=1;
    }
#if !RUN_WEBOTS
    butler_param();
#endif
}

void vmc_reset(void)
{

    robotwb.exp_att.yaw=robotwb.IMU_now.yaw;
    for(int i=0;i<4;i++)
    {
        robotwb.wheel_touch[i]=1;
    }
    //printf("vmc_reset\n");
}

void reset_tar_pos(char sel,char init_q) {//区分着地离地？<<----------------------------!!!!!!!!!!!!!!!!
#if 1
    for(int i=0;i<6;i++)
    {
        if(init_q)
          robotwb.arm_q_exp_flt[sel][i]=robotwb.arm_q_exp[sel][i]=robotwb.arm_q[sel][i]*57.3;//out is degree/input is rad
        else
          robotwb.arm_q_exp[sel][i]=robotwb.arm_q[sel][i]*57.3;
    }
    if(init_q)
        robotwb.arm_base_height_exp_flt=robotwb.arm_base_height_exp= robotwb.arm_base_height;
    else
        robotwb.arm_base_height_exp= robotwb.arm_base_height;
    robotwb.arm_att_n_exp=robotwb.arm_att_n;
    robotwb.arm_att_h_exp_flt.x=robotwb.arm_att_h_exp.x=0;
    robotwb.arm_att_h_exp_flt.y=robotwb.arm_att_h_exp.y=0;
    robotwb.arm_att_h_exp_flt.z=robotwb.arm_att_h_exp.z=0;

    robotwb.arm_epos_n_exp=robotwb.arm_epos_n;
    robotwb.arm_epos_h_exp_reg=robotwb.arm_epos_h_exp_flt=robotwb.arm_epos_h_exp=robotwb.arm_epos_h;
    robotwb.arm_epos_b_exp=robotwb.arm_epos_b;
    robotwb.arm_epos_ny_exp=robotwb.arm_epos_ny;

//    printf("reset hip:%f %f %f| %f %f %f\n",robotwb.arm_att_h_exp.x,robotwb.arm_att_h_exp.y,robotwb.arm_att_h_exp.z,
//           robotwb.arm_epos_h_exp.x,robotwb.arm_epos_h_exp.y,robotwb.arm_epos_h_exp.z);
#endif
}

char safe_check(float dt)
{
    char i = 0;
    static float timer_q_safe[4] = { 0 };

    if (fabs(robotwb.exp_att.pitch - robotwb.now_att.pitch) > SAFE_PITCH&&stand_force_enable_flag[4] == 1)
        return 1;
    if (fabs(robotwb.exp_att.roll - robotwb.now_att.roll) > SAFE_ROLL&&stand_force_enable_flag[4] == 1)
        return 1;

    for (i = 0; i < 4; i++) {//∠角度保护1
        if (fabs(To_180_degreesw(vmc[i].sita1 - vmc[i].sita2)) < 5 && stand_force_enable_flag[4] == 1)
            timer_q_safe[i] += dt;
        else
            timer_q_safe[i] = 0;

        if (timer_q_safe[i] > 2) {
            timer_q_safe[i] = 0;
            return 1;
        }
    }
    return 0;
}

//--------------------------------------------------主步态状态机------------------------------------------------
int motor_power_on=0;
void locomotion_sfm(float dt)
{
    int i, j;
    char robot_protect;
    static char temp_q_rst_flag = 0, temp_err_rst_flag = 0;
    char temp_flag[4] = { 0 };
    static int init = 0;
    static char key_ud_reg=0;
    static float timer[10] = { 0 };
    if (!init) {
        init = 1;
        vmc_reset();//
        reset_tar_pos(0,1);
        vmc_param_init();
        reset_robot_statement();
    }

    switch (robotwb.state_gait)
    {
    case 0:
       vmc_reset();//
       reset_tar_pos(0,1);
       robotwb.arm_att_b_exp.x=0;
       robotwb.arm_att_b_exp.y=0;
       robotwb.arm_att_b_exp.z=0;
       switch(temp_q_rst_flag)
       {
           case 0:
               motor_power_on=0;
               //-------------------标定角度 按键Y
               if(ocu.key_y)timer[1]+=dt;else timer[1]=0;

               if(timer[1]>1)
               {
                printf("Motor Zero Position delaying!!.....\n");
                temp_q_rst_flag++;
                robotwb.beep_state=BEEP_BLDC_ZERO_CAL;
                timer[1]=0;
               }
           break;
           case 1:
               timer[1]+=dt;
               if(timer[1]>4)
               {
                   printf("Motor Zero Position Save!!.....\n");
                   temp_q_rst_flag++;
                   for(i=0;i<2;i++)
                       arm_motor[i].reset_q=2;
                   robotwb.beep_state=BEEP_BLDC_GAIT_SWITCH;
                   timer[1]=0;
               }
           break;
           case 2://
               if(!ocu.key_y)timer[1]+=dt;

               if(timer[1]>0.5)
               {
                   temp_q_rst_flag=0;
                   for(i=0;i<2;i++)
                       arm_motor[i].reset_q=0;
                   timer[1]=0;
               }
           break;
       }//----

       //-------------------------驱动器状态检测与角度复位
       //printf("ocu.key_x=%d timer[0]=%f dt=%f\n",ocu.key_x,timer[0],dt);
       if(ocu.key_x||ocu.key_a||DIR_ARM_POWER)timer[0]+=dt;else timer[0]=0;
       if(timer[0]>1||RUN_WEBOTS)
       {
            vmc_reset();                    robotwb.arm_att_b_exp.x=0;
            robotwb.arm_att_b_exp.y=0;
            robotwb.arm_att_b_exp.z=0;
            printf("motor arm:%d %d %d %d %d %d|\n",
                     arm_motor[0].ready[0],
                     arm_motor[0].ready[1],
                     arm_motor[0].ready[2],
                     arm_motor[0].ready[3],
                     arm_motor[0].ready[4],
                     arm_motor[0].ready[5]);
#if ONLY_ARM
            if(  ( arm_motor[0].ready[0]==1
                &&arm_motor[0].ready[1]==1
                &&arm_motor[0].ready[2]==1
                &&arm_motor[0].ready[3]==1
                &&arm_motor[0].ready[4]==1
                &&arm_motor[0].ready[5]==1
                )||RUN_WEBOTS
                 ){//电机准备好
#else
            if(  ( arm_motor[0].ready[0]==1
                &&arm_motor[0].ready[1]==1
                &&arm_motor[0].ready[2]==1
                &&arm_motor[0].ready[3]==1
                &&arm_motor[0].ready[4]==1
                &&arm_motor[0].ready[5]==1
                &&wheel_motor.ready[0]==1
                &&wheel_motor.ready[1]==1
                &&wheel_motor.ready[2]==1
                &&wheel_motor.ready[3]==1)||RUN_WEBOTS
                 ){//电机准备好
#endif
               arm_motor[0].motor_en=1;//开始能
               wheel_motor.motor_en=1;//开始能
               head_motor.motor_en=1;
               motor_power_on=1;
               if(ocu.key_x||DIR_ARM_POWER){
                   leg_motor[0].q_bias[0]=leg_motor[0].q_now[0];
                   printf("Motor Initing and reset Z-bias.....\n");
                   robotwb.reset_z_mode=1;
               }else{
                   robotwb.reset_z_mode=0;
                   printf("Motor Initing.....\n");
               }

               robotwb.gait_mode=IDLE;
               robotwb.cmd_robot_state=0;
               robotwb.state_gait++;

               //robotwb.beep_state=BEEP_BLDC_ZERO_INIT;
               reset_tar_pos(0,0);
           }
           else{
               arm_motor[0].motor_en=head_motor.motor_en=wheel_motor.motor_en=0;
           }

            timer[0]=0;
            timer[1]=0;
       }
   break;
   case 1://------------
       timer[0]+=dt;
       if(timer[0]>0.01){
#if RUN_WEBOTS
            float gain_all=3;
            move_joint_to_pos_sel1(&robotwb,99,robotwb.safe_height,100*gain_all,0.01,0,dt);//height
            temp_flag[0]=1;
#else
            float gain_all=1;
            printf("robotwb.arm_base_tau=%f timer[0]=%f\n",robotwb.arm_base_tau,timer[0]);
            if(fabs(robotwb.arm_base_tau)>25||timer[0]>3)
                temp_flag[0]+=1;
            else if(robotwb.reset_z_mode==0)
                temp_flag[0]+=move_joint_to_pos_sel1(&robotwb,99,robotwb.safe_height,100*gain_all,0.03,0,dt);//height
#endif
            temp_flag[0]+=move_joint_to_pos_sel1(&robotwb,0,robotwb.safe_arm_q[0][0],250*gain_all,10,0,dt);//d
            temp_flag[0]+=move_joint_to_pos_sel1(&robotwb,1,robotwb.safe_arm_q[0][1],500*gain_all,10,0,dt);//x
            temp_flag[0]+=move_joint_to_pos_sel1(&robotwb,2,robotwb.safe_arm_q[0][2],250*gain_all,5,0,dt);//doghome
            temp_flag[0]+=move_joint_to_pos_sel1(&robotwb,3,robotwb.safe_arm_q[0][3],250*gain_all,5,0,dt);//doghome
            temp_flag[0]+=move_joint_to_pos_sel1(&robotwb,4,robotwb.safe_arm_q[0][4],250*gain_all,5,0,dt);//doghome
       }

       if(temp_flag[0]>=6){//可强制结束 Lock now angle
           printf("Motor Initing Done!!\n");
           robotwb.beep_state=0;
           if(robotwb.reset_z_mode==1){
                leg_motor[0].q_bias[0]=leg_motor[0].q_now[0];
                robotwb.reset_z_mode=0;
           }
           robotwb.arm_epos_b_exp.z=0.05;
#if 1
           robotwb.state_gait++;//->2
           robotwb.cmd_robot_state=1;//active locomotion contoller  doghome
           robotwb.arm_control_mode=ARM_M_H;
           reset_tar_pos(0,1);
           robotwb.reset_arm=1;
#endif

           vmc_reset();
        }
    break;
    case 2://enable ik

    break;
   }

   gait_switch(dt);//步态切换状态机

   switch(robotwb.gait_mode){
       case STAND_RC:
           Gait_Stand_Update(dt);
       break;
   }


   if (ocu.key_ud == -1 && key_ud_reg == 0) {//Disable Power
       vmc_reset();//
       reset_tar_pos(0,0);
       robotwb.gait_mode = IDLE;
       robotwb.robot_mode = M_SAFE;
       robotwb.beep_state=0;
       arm_motor[0].motor_en = 0;
       arm_motor[1].motor_en = 0;
       wheel_motor.motor_en = 0;
       head_motor.motor_en = 0;
       robotwb.grasp_action=0;
       robotwb.state_gait = 0;//reset main state
       robotwb.cap_set=1;//open
       printf("Init Reset Stop!!\n");
   }
    key_ud_reg=ocu.key_ud;
 }

float att_trig_reset_dead = 2.56;
float soft_param[2] = { 0.75,0.6 };//
float z_init_spd = 6.8;//站立初始化速度
float init_end_z = 0;
char gait_switch(float dt)//cushihua  moshi  主步态切换状态机
{
    char i;
    static float timer_webot_auto = 0;
    static char sdk_mode_reg = 0;
    static char stand_switch_trig = 0;
    static float t, t_rst[3], timer[5], t_fall, t_fall_self_right = 0;
    static char state, rst_state, soft_start_state, fall_state, t_um;
    static float sbus_mode_sw_cnt = 0;
    float cog_off, att_off;
    float end_dis[4];
    float att_use[3], err[2];
    static int rolling_time = 0;
    att_use[ROLr] = vmc_all.att_ctrl[ROLr];
    err[ROLr] = vmc_all.tar_att[ROLr] + vmc_all.tar_att_off[ROLr] - att_use[ROLr];

    //printf("%d %d\n",ocu.connect,ocu.mode);
    if ((((ocu.connect&&ocu.mode >= 1) || sdk.sdk_mode == 1)) || (RUN_WEBOTS)) {//

        switch (robotwb.cmd_robot_state)//moshi
        {
         case 1://

            Gait_Stand_Active();//
            robotwb.cmd_robot_state=2;
         break;
         case 2://Stand mode



         break;
        }

        if (((ocu.key_ud == -1 && ocu.key_ud_reg == 0) || (sdk_mode_reg == 1 && sdk.sdk_mode == 0)) && vmc_all.sita_test[4] == 0) {//Disable Power
            vmc_reset();//
            reset_tar_pos(0,0);
            robotwb.gait_mode = IDLE;
            robotwb.robot_mode = M_SAFE;
            robotwb.beep_state=0;
            arm_motor[0].motor_en = 0;
            arm_motor[1].motor_en = 0;
            wheel_motor.motor_en = 0;
            head_motor.motor_en = 0;
            robotwb.state_gait = 0;//reset main state
        }

        sdk_mode_reg = sdk.sdk_mode; ocu.key_ud_reg = ocu.key_ud; ocu.key_lr_reg = ocu.key_lr; ocu.key_x_reg = ocu.key_x; ocu.key_y_reg = ocu.key_y; ocu.key_a_reg = ocu.key_a;
        ocu.key_b_reg = ocu.key_b; ocu.key_ll_reg = ocu.key_ll; ocu.key_rr_reg = ocu.key_rr; ocu.key_st_reg = ocu.key_st; ocu.key_back_reg = ocu.key_back;
    }
    return vmc_all.leg_power;
}
