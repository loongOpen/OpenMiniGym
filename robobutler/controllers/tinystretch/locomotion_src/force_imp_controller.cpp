#include "include.h"
#include "locomotion_header.h"
#include "gait_math.h"

#define TST_FF 0

float q_i_max = 0.05;
float max_imp_spd[3] = { 1.8,1.8,1.8 };//m/s
float max_imp_spd_bound[3] = { 0.8,0.8,0.8 };//m/s
float max_imp_spd_air[3] = {2.0,2.0,2.0 };//m/s 空中的IMP位置移动

#if TSET_F_IMP&&TEST_FF_MODE
float pos_force_enable[3] = { 1,1,0 };//力控标志位  位置  反馈  前馈
#elif TSET_F_FF&&TEST_FF_MODE
float pos_force_enable[3] = { 1,0,1 };//力控标志位  位置  反馈  前馈
#else
float pos_force_enable[3] = { 1,1,1 };//力控标志位  位置  反馈  前馈
#endif

char ground_reg[4] = { 0 };
char touch_reg[4] = { 0 };
char ff_flag_reg[4] = { 0 };
float kp_imp_spd = 1;//0.25;
char ground_test[4] = { 1,1,1,1 };
float imp_spd_test[3] = { 0,0,0 };
int nan_err1[10] = { 0,0,0 };
void force_control_and_dis_pose(float dt)//底层==>位力混控制  站立步态！！
{
     float kp_gain[2]={0.01,0.01};
     float kp_gain_pos[3]={0.01,0.01,0.01};

    if(robotwb.state_gait==2){
        if( !robotwb.reset_arm){
            kp_gain_pos[0]=0.1;
            kp_gain_pos[1]=0.1;
            kp_gain_pos[2]=0.1;
            kp_gain[1]=0.1;
        }
        if(robotwb.grasp_mission>0){
            kp_gain_pos[0]=0.015;
            kp_gain_pos[1]=0.015;
            kp_gain_pos[2]=0.005;
            kp_gain[1]=0.0025;
        }

        ik_hip_protect(&robotwb.arm_epos_h_exp);
        //IK output
        if(robotwb.reset_arm){
            float dis=sqrt(pow(robotwb.arm_epos_h_exp.x-robotwb.arm_epos_h.x,2)+pow(robotwb.arm_epos_h_exp.y-robotwb.arm_epos_h.y,2));
            if(dis<0.02){
                robotwb.reset_arm=0;
                printf("Reset done!\n");
            }
        }

        robotwb.arm_epos_h_exp_flt.x+=LIMIT(robotwb.arm_epos_h_exp.x-robotwb.arm_epos_h_exp_flt.x,-0.1,0.1)*kp_gain_pos[0];
        robotwb.arm_epos_h_exp_flt.y+=LIMIT(robotwb.arm_epos_h_exp.y-robotwb.arm_epos_h_exp_flt.y,-0.1,0.1)*kp_gain_pos[1];
        robotwb.arm_epos_h_exp_flt.z+=LIMIT(robotwb.arm_epos_h_exp.z-robotwb.arm_epos_h_exp_flt.z,-0.1,0.1)*kp_gain_pos[2];
        robotwb.arm_att_h_exp_flt.x+=LIMIT(robotwb.arm_att_h_exp.x-robotwb.arm_att_h_exp_flt.x,-30.5,30.5)*kp_gain[1];
        robotwb.arm_att_h_exp_flt.y+=LIMIT(robotwb.arm_att_h_exp.y-robotwb.arm_att_h_exp_flt.y,-30.5,30.5)*kp_gain[1];
        robotwb.arm_att_h_exp_flt.z+=LIMIT(robotwb.arm_att_h_exp.z-robotwb.arm_att_h_exp_flt.z,-30.5,30.5)*kp_gain[1]/2;
        //printf("0%f %f %f | %f %f %f\n",robotwb.arm_att_h_exp.x,robotwb.arm_att_h_exp.y,robotwb.arm_att_h_exp.z,robotwb.arm_att_h_exp.x,robotwb.arm_att_h_exp.y,robotwb.arm_att_h_exp.z);
        //printf("1%f %f %f | %f %f %f\n",robotwb.arm_att_h_exp_flt.x,robotwb.arm_att_h_exp_flt.y,robotwb.arm_att_h_exp_flt.z,robotwb.arm_att_h_exp_flt.x,robotwb.arm_att_h_exp_flt.y,robotwb.arm_att_h_exp_flt.z);
        END_POS end_pos;
        end_pos.x=robotwb.arm_epos_h_exp_flt.x;
        end_pos.y=robotwb.arm_epos_h_exp_flt.y;
        end_pos.z=robotwb.base_z+robotwb.arm_epos_h_exp_flt.z;//base =0.1
        END_POS end_att;
        end_att.x=robotwb.arm_att_h_exp_flt.x/57.3*!DIR_EE_ROLL;
        end_att.y=robotwb.arm_att_h_exp_flt.y/57.3;
        end_att.z=robotwb.arm_att_h_exp_flt.z/57.3;
#if USE_PINO
        float q_out[6]={0};
        ik_pino(end_pos,end_att,q_out);
        printf("ik_pino:%.2f %.2f %.2f %.2f %.2f %.2f\n",q_out[0],q_out[1],q_out[2],q_out[3],q_out[4],q_out[5]);
#else
        float q_out[6]={0};
        ik_rbdl(end_pos,end_att,q_out);
        //printf("ik_rbdl:%.2f %.2f %.2f %.2f %.2f %.2f\n",q_out[0],q_out[1],q_out[2],q_out[3],q_out[4],q_out[5]);
#endif
        robotwb.arm_base_height_exp= q_out[0];
        robotwb.arm_q_exp[0][0] = q_out[1]*57.3;//
        robotwb.arm_q_exp[0][1] = q_out[2]*57.3;//
        robotwb.arm_q_exp[0][2] = q_out[3]*57.3;//
        robotwb.arm_q_exp[0][3] = q_out[4]*57.3;//
#if DIR_EE_ROLL
        robotwb.arm_q_exp[0][4] = robotwb.arm_att_b_exp.x;//
#else
        robotwb.arm_q_exp[0][4] = q_out[5]*57.3;//
#endif
    }
}

void force_control_and_dis_stand(float dt)//底层==>位力混控制  unused
{
    int i = 0;
    float q_err[5] = { 0 };
    Vect3 Vect3_zero;
    Vect3_zero.x = Vect3_zero.y = Vect3_zero.z = 0;
    Vect3 force_err_n = Vect3_zero;
    Vect3 tau_err_n = Vect3_zero;
    Vect3 force_imp_spd_n = Vect3_zero;
    Vect3 tau_imp_spd_n = Vect3_zero;
    Vect3 force_imp_spd_n_yaw = Vect3_zero;
    Vect3 force_imp_spd_h = Vect3_zero;
    Vect3 epos_h_next = Vect3_zero;
    float pos_taod[5] = { 0,0 };
    float fb_force_taod[5] = { 0,0 };
    float ff_force_taod[5] = { 0,0 };
    float dt_scale = 1;//0.005/(dt+0.00000001);<---------------------------------微分尺度系数

    for (i = 0; i < 2; i++) {//站立
        force_err_n.x = robotwb.Leg[i].tar_force_dis_n.x - robotwb.Leg[i].force_est_n_output.x;
        force_err_n.y = robotwb.Leg[i].tar_force_dis_n.y - robotwb.Leg[i].force_est_n_output.y;
        force_err_n.z = robotwb.Leg[i].tar_force_dis_n.z - robotwb.Leg[i].force_est_n_output.z;

#if 1//力误差限制 防止抖腿
        force_err_n.x = limitw(force_err_n.x, -robotwb.max_err_force.x, robotwb.max_err_force.x);
        force_err_n.y = limitw(force_err_n.y, -robotwb.max_err_force.y, robotwb.max_err_force.y);
        force_err_n.z = limitw(force_err_n.z, -robotwb.max_err_force.z, robotwb.max_err_force.z);
#endif
        robotwb.Leg[i].force_err_n = force_err_n;//record


        tau_err_n.y = -(robotwb.Leg[i].tar_torque_dis_n_mpc.y - robotwb.Leg[i].tau_est_h_output.y);
        tau_err_n.y = limitw(tau_err_n.y, -25, 25);

        tau_err_n.z = (robotwb.Leg[i].tar_torque_dis_n_mpc.z - robotwb.Leg[i].tau_est_h_output.z);
        tau_err_n.z = limitw(tau_err_n.z, -15, 15);
        //printf("fz=%f %f\n", robotwb.Leg[i].tar_force_dis_n.z, robotwb.Leg[i].force_est_n_output.z);
//************************************************************************************************************
//***********************************************阻抗控制*****************************************************
//************************************************************************************************************
#if TST_FF
        stand_force_enable_flag[4] = 0;
#endif
        if (ground_reg[i] != vmc[i].ground) {//着地复位阻抗基准WS<--------------------------------
            //if(ff_flag_reg[i]!=stand_force_enable_flag[i])//W 需修改为力控标准  groundflag主要用于状态估计
            reset_tar_pos(i,0);
            robotwb.Leg[i].tao_q_i[0] = robotwb.Leg[i].tao_q_i[1] = robotwb.Leg[i].tao_q_i[2] = 0;
        }
        // printf("%d %d %d %d %d\n",stand_force_enable_flag[4],vmc[0].ground,vmc[1].ground,vmc[2].ground,vmc[3].ground);
        if (//力导纳模式  离地时存在误差 由导纳把腿拉到地面
            stand_force_enable_flag[4]//全局力控符号
          //&&stand_force_enable_flag[i]//独立力控符号
            && USE_FPOS_CONTROL
            &&pos_force_p.en_force_control_cal
            ) {
            if (vmc[i].ground) {//着地力误差即加速度产生末端速度期望
                force_imp_spd_n.x = force_err_n.x*robotwb.Leg[i].f_pos_pid[Xr].kp - robotwb.Leg[i].dforce_est_n_output.x*robotwb.Leg[i].f_pos_pid[Xr].kd*dt_scale;
                force_imp_spd_n.y = force_err_n.y*robotwb.Leg[i].f_pos_pid[Yr].kp - robotwb.Leg[i].dforce_est_n_output.y*robotwb.Leg[i].f_pos_pid[Yr].kd*dt_scale;
                force_imp_spd_n.z = force_err_n.z*robotwb.Leg[i].f_pos_pid[Zr].kp - robotwb.Leg[i].dforce_est_n_output.z*robotwb.Leg[i].f_pos_pid[Zr].kd*dt_scale;

                force_imp_spd_n.x *= -1;
                force_imp_spd_n.y *= -1;
                force_imp_spd_n.z *= -1;

                tau_imp_spd_n.y = tau_err_n.y * 200 * 0.1;
                tau_imp_spd_n.z = tau_err_n.z * 200 * 0.1;
            }
            else {//收腿受着地速度影响
#if 1
                force_imp_spd_n.x = 0;
                force_imp_spd_n.y = 0;
                force_imp_spd_n.z = -vmc_all.param.param_vmc.ground_seek_spd;//往下速度为+因为后面 反向符号了已经
#elif 1
                force_imp_spd_n.x = 0;//非着地load力
                force_imp_spd_n.y = 0;
                force_imp_spd_n.z = force_err_n.z*robotwb.Leg[i].f_pos_pid[Zr].kp - robotwb.Leg[i].dforce_est_n_output.z*robotwb.Leg[i].f_pos_pid[Zr].kd*dt_scale;
#else
                force_imp_spd_n.x = force_err_n.x*robotwb.Leg[i].f_pos_pid[Xr].kp - robotwb.Leg[i].dforce_est_n_output.x*robotwb.Leg[i].f_pos_pid[Xr].kd*dt_scale;
                force_imp_spd_n.z = force_err_n.z*robotwb.Leg[i].f_pos_pid[Zr].kp - robotwb.Leg[i].dforce_est_n_output.z*robotwb.Leg[i].f_pos_pid[Zr].kd*dt_scale;
#endif
            }

            //补偿 机体旋转引起的末端速度
#if 1
            Vect3 Hip_b;
            Vect3 com;
            com.x = com.y = com.z = 0;
#if 1
            Hip_b.x = com.x - vmc[i].epos_b.x;
            Hip_b.y = com.y - vmc[i].epos_b.y;
            Hip_b.z = com.z - vmc[i].epos_b.z;
#else
            Hip_b.x = com.x - (vmc[leg_sel_trig].flag_fb*Hw / 2);
            Hip_b.y = com.y - (vmc[leg_sel_trig].flag_rl*Www / 2);
            Hip_b.z = com.z - 0;
#endif
            Vect3 w_b;
            w_b.x = my_deathzoom(vmc_all.att_rate_trig[ROLr], 0) * 0.0173 * 1 * 1;  //需要补偿地形角的角速度吗？
            //w_b.y = LIMIT(my_deathzoom(vmc_all.att_rate_trig[PITr], 5),-25,25) * 0.0173 * -1*1;
            w_b.y = (my_deathzoom(vmc_all.att_rate_trig[PITr], 0)) * 0.0173 * 1 * 1;
            //printf("%f %f\n",vmc_all.att_rate_trig[PITr],vmc_all.att_rate_trig[ROLr]);
            w_b.z = vmc_all.att_rate_trig[YAWr] * 0.0173 * 1;//加上上台阶好一些
            float w_cross_b[3][3] = { 0 };
            vect3_2_cross(w_b, w_cross_b);
            Vect3 hip_rotate_spd_b, hip_rotate_spd_n, hip_rotate_spd_g;

            matrx33_mult_vect3(w_cross_b, Hip_b, &hip_rotate_spd_b);

            converV_b_to_n(hip_rotate_spd_b.x, hip_rotate_spd_b.y, hip_rotate_spd_b.z,
                &hip_rotate_spd_n.x, &hip_rotate_spd_n.y, &hip_rotate_spd_n.z);//转换速度直接输出到IMP  控制频率不够目前？？

            converV_n_to_g(hip_rotate_spd_n, &hip_rotate_spd_g);//不加不行
            //hip_rotate_spd_g = hip_rotate_spd_n;

            //hip_rotate_spd_g.x=LIMIT(hip_rotate_spd_g.x,-1,1);
            //hip_rotate_spd_g.y=LIMIT(hip_rotate_spd_g.y,-1,1);
            //hip_rotate_spd_g.z=LIMIT(hip_rotate_spd_g.z,-1,1);

            force_imp_spd_n.x += hip_rotate_spd_g.x * -1;
            force_imp_spd_n.y += hip_rotate_spd_g.y * -1;
            force_imp_spd_n.z += hip_rotate_spd_g.z * -1;
#endif
            //					if(isnan(robotwb.Leg[i].dforce_est_n_output.x)||isnan(robotwb.Leg[i].dforce_est_n_output.z))
            //						nan_err1++;
            force_imp_spd_n.x = limitw(force_imp_spd_n.x, -max_imp_spd[Xr], max_imp_spd[Xr]);
            force_imp_spd_n.y = limitw(force_imp_spd_n.y, -max_imp_spd[Yr], max_imp_spd[Yr]);
            force_imp_spd_n.z = limitw(force_imp_spd_n.z, -max_imp_spd[Zr], max_imp_spd[Zr]);

            robotwb.Leg[i].force_imp_spd_n = force_imp_spd_n;
            if (isnan(force_imp_spd_n.x) || isnan(force_imp_spd_n.y) || isnan(force_imp_spd_n.z))
                nan_err1[0]++;
            //转换机体速度 到地形坐标系
#if EN_F_T_G
            converV_n_to_bw(force_imp_spd_n, &force_imp_spd_g);
            converV_n_to_gw(force_imp_spd_g, &force_imp_spd_h);
#else
            converV_n_to_bw(force_imp_spd_n, &force_imp_spd_h);//唯一的转换处阻抗模型 全局转机体有问题
#endif
//补偿 机体旋转引起的末端速度
            if (isnan(force_imp_spd_h.x) || isnan(force_imp_spd_h.y) || isnan(force_imp_spd_h.z))
                nan_err1[1]++;

            //机体速度逆解更新求取角度与角速度
            robotwb.Leg[i].force_imp_spd_h = force_imp_spd_h;//record

            vmc[i].param.tar_epos_h.x += force_imp_spd_h.x*dt;//阻抗唯一使用的位置
            vmc[i].param.tar_epos_h.y += force_imp_spd_h.y*dt;
            vmc[i].param.tar_epos_h.z += force_imp_spd_h.z*dt;

            epos_h_next.x = vmc[i].param.tar_epos_h.x = limitw(vmc[i].param.tar_epos_h.x, MIN_X, MAX_X);//位置限制幅度
            epos_h_next.y = vmc[i].param.tar_epos_h.y = limitw(vmc[i].param.tar_epos_h.y, MIN_Y, MAX_Y);//位置限制幅度
            epos_h_next.z = vmc[i].param.tar_epos_h.z = limitw(vmc[i].param.tar_epos_h.z, MAX_Z, MIN_Z);
            epos_h_next.z += Hw;
            //					if(isnan(epos_h_next.x)||isnan(epos_h_next.z))
            //						nan_err1++;
           // inv_KI_hip(i, epos_h_next, &vmc[i].tar_sita1, &vmc[i].tar_sita2, &vmc[i].tar_sita3);//拟运动学阻抗输出  已加速
            //printf("st1::exp=%f now=%f expq=%f nowq=%f tau_imp_spd_n.y=%f\n", robotwb.Leg[i].tar_torque_dis_n_mpc.y, robotwb.Leg[i].tau_est_h_output.y,
            //	vmc[i].tar_sita5, robotwb.Leg[i].sita[4], tau_imp_spd_n.y);
            vmc[i].tar_sita5 += tau_imp_spd_n.y*dt;
            vmc[i].tar_sita4 += tau_imp_spd_n.z*dt;
            //printf("st2::exp=%f now=%f expq=%f nowq=%f tau_imp_spd_n.y=%f\n", robotwb.Leg[i].tar_torque_dis_n_mpc.y, robotwb.Leg[i].tau_est_h_output.y,
            //	vmc[i].tar_sita5, robotwb.Leg[i].sita[4], tau_imp_spd_n.y);
//					//非力控清除摆动位置期望？？
//					vmc[i].tar_epos_h=vmc[i].tar_epos_h_reg=vmc[i].epos;//站立时的末端逆解和摆动逆解
            vmc[i].param.spd_dj[0] = vmc[i].param.spd_dj[1] = 0;
        }
        else if (gait_ww.state_gait > 2 && !stand_force_enable_flag[4])//<---------------------------------位置模式 空中模式
        {
#if 1
            if (EN_END_SPD_MODE) {//速度平滑模式模式
#if 0//仅速度误差产生IMP 按压导致着地抖动 能越来越低
                force_imp_spd_h.x = -limitw(vmc[i].tar_epos_h.x - vmc[i].tar_epos_h_reg.x, -0.05, 0.05) / dt;//复位末端期望位置  离地后如不是全离地则保证当前位置
                force_imp_spd_h.z = -limitw(vmc[i].tar_epos_h.z - vmc[i].tar_epos_h_reg.z, -0.05, 0.05) / dt;

                force_imp_spd_h.x = limitw(force_imp_spd_h.x, -max_imp_spd_air[Xr], max_imp_spd_air[Xr]);
                force_imp_spd_h.z = limitw(force_imp_spd_h.z, -max_imp_spd_air[Zr], max_imp_spd_air[Zr]);
#elif 1//期望跟踪
                force_imp_spd_h.x = (vmc[i].param.tar_epos_h.x - vmc[i].tar_epos_h.x) / dt * kp_imp_spd;//复位末端期望位置  离地后如不是全离地则保证当前位置
                force_imp_spd_h.y = (vmc[i].param.tar_epos_h.y - vmc[i].tar_epos_h.y) / dt * kp_imp_spd;
                force_imp_spd_h.z = (vmc[i].param.tar_epos_h.z - vmc[i].tar_epos_h.z) / dt * kp_imp_spd;

                force_imp_spd_h.x = limitw(force_imp_spd_h.x, -max_imp_spd_air[Xr], max_imp_spd_air[Xr]);
                force_imp_spd_h.y = limitw(force_imp_spd_h.y, -max_imp_spd_air[Yr], max_imp_spd_air[Yr]);
                force_imp_spd_h.z = limitw(force_imp_spd_h.z, -max_imp_spd_air[Zr], max_imp_spd_air[Zr]);
#else//期望跟踪使用真实反馈
                force_imp_spd_h.x = limitw(vmc[i].param.tar_epos_h.x - vmc[i].epos.x, -0.05, 0.05) / dt * kp_imp_spd;//复位末端期望位置  离地后如不是全离地则保证当前位置
                force_imp_spd_h.y = 0;
                force_imp_spd_h.z = limitw(vmc[i].param.tar_epos_h.z - vmc[i].epos.z, -0.05, 0.05) / dt * kp_imp_spd;

                force_imp_spd_h.x = limitw(force_imp_spd_h.x, -max_imp_spd_air[Xr], max_imp_spd_air[Xr]);
                force_imp_spd_h.y = 0;
                force_imp_spd_h.z = limitw(force_imp_spd_h.z, -max_imp_spd_air[Zr], max_imp_spd_air[Zr]);
#endif
                //机体速度逆解更新求取角度与角速度
                if (vmc[i].is_touch == 1) {
                    force_imp_spd_h.x *= 0.1;
                    force_imp_spd_h.y *= 0.1;
                    force_imp_spd_h.z *= 0.1;
                }

                robotwb.Leg[i].force_imp_spd_h = force_imp_spd_h;//record
                END_POS force_imp_spd_ht;
                force_imp_spd_ht.x = force_imp_spd_h.x;
                force_imp_spd_ht.y = force_imp_spd_h.y;
                force_imp_spd_ht.z = force_imp_spd_h.z;
               // espd_to_qspd(&vmc[i], force_imp_spd_ht, vmc[i].param.spd_dj, dt);

                vmc[i].param.tar_epos_h.x -= force_imp_spd_h.x*dt;
                vmc[i].param.tar_epos_h.y -= force_imp_spd_h.y*dt;
                vmc[i].param.tar_epos_h.z -= force_imp_spd_h.z*dt;

                epos_h_next.x = vmc[i].param.tar_epos_h.x = limitw(vmc[i].param.tar_epos_h.x, MIN_X, MAX_X);//位置限制幅度
                epos_h_next.y = vmc[i].param.tar_epos_h.y = limitw(vmc[i].param.tar_epos_h.y, MIN_Y, MAX_Y);
                epos_h_next.z = vmc[i].param.tar_epos_h.z = limitw(vmc[i].param.tar_epos_h.z, MAX_Z, MIN_Z);
                //epos_h_next.z += Hw;
                //(i, epos_h_next, &vmc[i].tar_sita1, &vmc[i].tar_sita2, &vmc[i].tar_sita3);//拟运动学阻抗输出  已加速

                vmc[i].tar_foot_q_n = 0;
                vmc[i].tar_sita5 = vmc[i].tar_foot_q_n + 90 - vmc[i].sita_leg_knee_b + 10 + vmc_all.att[PITr];//足端  +向下压
                vmc[i].tar_sita4 = 0;
                //printf("[%d] tar_q=%f %f %f\n", i, vmc[i].tar_sita1, vmc[i].tar_sita2, vmc[i].tar_sita3);
                //printf("[%d] tar_now=%f %f %f\n", i, robotwb.Leg[i].sita[0], robotwb.Leg[i].sita[1], robotwb.Leg[i].sita[2]);
            }
#else//测试IMP
            force_imp_spd_n.x = imp_spd_test[Xr] + vmc_all.tar_spd.x / 2;
            force_imp_spd_n.z = imp_spd_test[Zr] + vmc_all.tar_spd.y / 2;//往下速度为+因为后面 反向符号了已经

            force_imp_spd_n.x = limitw(force_imp_spd_n.x, -max_imp_spd[Xr], max_imp_spd[Xr]);
            force_imp_spd_n.z = limitw(force_imp_spd_n.z, -max_imp_spd[Zr], max_imp_spd[Zr]);
            robotwb.Leg[i].force_imp_spd_n = force_imp_spd_n;
            //转换机体速度 到地形坐标系
#if EN_F_T_G
            converV_n_to_bw(force_imp_spd_n, &force_imp_spd_g);
            converV_n_to_gw(force_imp_spd_g, &force_imp_spd_h);
#else
            converV_n_to_bw(force_imp_spd_n, &force_imp_spd_h);//唯一的转换处阻抗模型
#endif
//补偿 机体旋转引起的末端速度


//机体速度逆解更新求取角度与角速度
            robotwb.Leg[i].force_imp_spd_h = force_imp_spd_h;//record

            vmc[i].param.tar_epos_h.x -= force_imp_spd_h.x*dt;
            vmc[i].param.tar_epos_h.z -= force_imp_spd_h.z*dt;
            epos_h_next.x = vmc[i].param.tar_epos_h.x = limitw(vmc[i].param.tar_epos_h.x, MIN_X, MAX_X);//位置限制幅度
            epos_h_next.z = vmc[i].param.tar_epos_h.z = limitw(vmc[i].param.tar_epos_h.z, MAX_Z, MIN_Z);
            inv_KI(i, epos_h_next, &vmc[i].tar_sita1, &vmc[i].tar_sita2, &vmc[i].tar_sita3);//拟运动学阻抗输出  已加速
#endif
            //printf("[%d]%f %f %f\n", i, vmc[i].tar_sita1, vmc[i].tar_sita2, vmc[i].tar_sita3);
        }
        else if (gait_ww.state_gait > 2 && 0) {//防止别处赋值造成阶跃WS
            vmc[i].tar_sita1 = vmc[i].sita1;
            vmc[i].tar_sita2 = vmc[i].sita2;
            vmc[i].tar_sita3 = vmc[i].sita3;
        }

        //----------------------------------------记录缓存
        vmc[i].tar_epos_h_reg = vmc[i].tar_epos_h;//微分规划足端产生期望速度
        ground_reg[i] = vmc[i].ground;
        ff_flag_reg[i] = stand_force_enable_flag[i];
        //printf("[%d]%f %f %f\n", i,vmc[i].tar_sita1, vmc[i].tar_sita2, vmc[i].tar_sita3);
        //************************************************************************************************************
        //**************************************************角度控制**************************************************
        //************************************************************************************************************
        if (gait_ww.state_gait > 2) {//非初始化下 限制位置期望范围 步态摆动
            vmc[i].tar_sita1 = limitw(vmc[i].tar_sita1, -45,165);
            vmc[i].tar_sita2 = limitw(vmc[i].tar_sita2, 0,165);
            vmc[i].tar_sita3 = limitw(vmc[i].tar_sita3, -robotwb.Leg[i].limit_sita[2], robotwb.Leg[i].limit_sita[2]);
            vmc[i].tar_sita4 = limitw(vmc[i].tar_sita4, -robotwb.Leg[i].limit_sita[3], robotwb.Leg[i].limit_sita[3]);
            vmc[i].tar_sita5 = limitw(vmc[i].tar_sita5, -robotwb.Leg[i].limit_sita[4], robotwb.Leg[i].limit_sita[4]);
        }
#if TST_FF
        vmc[i].tar_sita1 = 45;
        vmc[i].tar_sita2 = 90;
        vmc[i].tar_sita3 = 0;
#endif
        robotwb.Leg[i].tar_sita[0] = vmc[i].tar_sita1;//robotwb tar 位置闭环期望赋值处
        robotwb.Leg[i].tar_sita[1] = vmc[i].tar_sita2;
        robotwb.Leg[i].tar_sita[2] = vmc[i].tar_sita3;
        robotwb.Leg[i].tar_sita[3] = vmc[i].tar_sita4;
        robotwb.Leg[i].tar_sita[4] = vmc[i].tar_sita5;

        robotwb.Leg[i].err_sita[0] = To_180_degreesw(robotwb.Leg[i].tar_sita[0] - robotwb.Leg[i].sita[0]);
        robotwb.Leg[i].err_sita[1] = To_180_degreesw(robotwb.Leg[i].tar_sita[1] - robotwb.Leg[i].sita[1]);
        robotwb.Leg[i].err_sita[2] = To_180_degreesw(robotwb.Leg[i].tar_sita[2] - robotwb.Leg[i].sita[2]);
        robotwb.Leg[i].err_sita[3] = To_180_degreesw(robotwb.Leg[i].tar_sita[3] - robotwb.Leg[i].sita[3]);
        robotwb.Leg[i].err_sita[4] = To_180_degreesw(robotwb.Leg[i].tar_sita[4] - robotwb.Leg[i].sita[4]);//足端

        q_err[0] = limitw(To_180_degreesw(robotwb.Leg[i].tar_sita[0] - robotwb.Leg[i].sita[0]), -66, 66);
        q_err[1] = limitw(To_180_degreesw(robotwb.Leg[i].tar_sita[1] - robotwb.Leg[i].sita[1]), -66, 66);
        q_err[2] = limitw(To_180_degreesw(robotwb.Leg[i].tar_sita[2] - robotwb.Leg[i].sita[2]), -66, 66);
        q_err[3] = limitw(To_180_degreesw(robotwb.Leg[i].tar_sita[3] - robotwb.Leg[i].sita[3]), -25, 25);
        q_err[4] = limitw(To_180_degreesw(robotwb.Leg[i].tar_sita[4] - robotwb.Leg[i].sita[4]), -25, 25);
        //printf("tar=%f %f\n", robotwb.Leg[i].tar_sita[4], robotwb.Leg[i].sita[4]);
        //积分控制
        if (leg_motor[i].connect&&leg_motor[i].connect_motor[i] && leg_motor[i].ready[i] && leg_motor[i].motor_en) {
            robotwb.Leg[i].tao_q_i[0] += q_err[0] * robotwb.Leg[i].q_pid.ki*dt * 10 / 1000.;
            robotwb.Leg[i].tao_q_i[1] += q_err[1] * robotwb.Leg[i].q_pid.ki*dt * 10 / 1000.;
            robotwb.Leg[i].tao_q_i[2] += q_err[2] * robotwb.Leg[i].q_pid.ki*dt * 10 / 1000.;
            robotwb.Leg[i].tao_q_i[3] += q_err[3] * robotwb.Leg[i].q_pid.ki*dt * 10 / 1000.;
            robotwb.Leg[i].tao_q_i[4] += q_err[4] * robotwb.Leg[i].q_pid.ki*dt * 10 / 1000.;
        }
        else {
            robotwb.Leg[i].tao_q_i[0] = 0;
            robotwb.Leg[i].tao_q_i[1] = 0;
            robotwb.Leg[i].tao_q_i[2] = 0;
            robotwb.Leg[i].tao_q_i[3] = 0;
            robotwb.Leg[i].tao_q_i[4] = 0;
        }
        robotwb.Leg[i].tao_q_i[0] = limitw(robotwb.Leg[i].tao_q_i[0], -robotwb.Leg[i].limit_tao[0] * q_i_max, robotwb.Leg[i].limit_tao[0] * q_i_max);
        robotwb.Leg[i].tao_q_i[1] = limitw(robotwb.Leg[i].tao_q_i[1], -robotwb.Leg[i].limit_tao[1] * q_i_max, robotwb.Leg[i].limit_tao[1] * q_i_max);
        robotwb.Leg[i].tao_q_i[2] = limitw(robotwb.Leg[i].tao_q_i[2], -robotwb.Leg[i].limit_tao[2] * q_i_max, robotwb.Leg[i].limit_tao[2] * q_i_max);
        robotwb.Leg[i].tao_q_i[3] = limitw(robotwb.Leg[i].tao_q_i[3], -robotwb.Leg[i].limit_tao[3] * q_i_max, robotwb.Leg[i].limit_tao[3] * q_i_max);
        robotwb.Leg[i].tao_q_i[4] = limitw(robotwb.Leg[i].tao_q_i[4], -robotwb.Leg[i].limit_tao[4] * q_i_max, robotwb.Leg[i].limit_tao[4] * q_i_max);

#if !defined(CAN_ANL_MIT_MODE)&&!RUN_PI
        pos_taod[0] = q_err[0] * robotwb.Leg[i].q_pid.kp * 10 / 1000. -
            limitw(robotwb.Leg[i].sita_d[0], -3500, 3500)*robotwb.Leg[i].q_pid.kd*dt_scale * 10 / 1000. +
            robotwb.Leg[i].tao_q_i[0];

        pos_taod[1] = q_err[1] * robotwb.Leg[i].q_pid.kp * 10 / 1000. -
            limitw(robotwb.Leg[i].sita_d[1], -3500, 3500)*robotwb.Leg[i].q_pid.kd*dt_scale * 10 / 1000. +
            robotwb.Leg[i].tao_q_i[1];

        pos_taod[2] = q_err[2] * robotwb.Leg[i].q_pid.kp * 10 / 1000. -
        limitw(robotwb.Leg[i].sita_d[2], -3500, 3500)*robotwb.Leg[i].q_pid.kd*dt_scale * 10 / 1000. +
            robotwb.Leg[i].tao_q_i[2];

        pos_taod[3] = q_err[3] * robotwb.Leg[i].q_pid.kp * 10 / 1000. -
            limitw(robotwb.Leg[i].sita_d[3], -3500, 3500)*robotwb.Leg[i].q_pid.kd*dt_scale * 10 / 1000. +
            robotwb.Leg[i].tao_q_i[3];

        pos_taod[4] = q_err[4] * robotwb.Leg[i].q_pid.kp * 10 / 10000. -
            limitw(robotwb.Leg[i].sita_d[4], -3500, 3500)*robotwb.Leg[i].q_pid.kd*dt_scale * 10 / 10000. +
            robotwb.Leg[i].tao_q_i[4];
        //pos_taod[4] *= 0.07;

        pos_taod[0] += limitw(vmc[i].param.spd_dj[0] * robotwb.Leg[i].q_pid.vff* dt, -robotwb.Leg[i].limit_tao[0] * 0.2, robotwb.Leg[i].limit_tao[0] * 0.2);
        pos_taod[1] += limitw(vmc[i].param.spd_dj[1] * robotwb.Leg[i].q_pid.vff* dt, -robotwb.Leg[i].limit_tao[1] * 0.2, robotwb.Leg[i].limit_tao[1] * 0.2);
        pos_taod[2] += limitw(vmc[i].param.spd_dj[2] * robotwb.Leg[i].q_pid.vff* dt, -robotwb.Leg[i].limit_tao[2] * 0.2, robotwb.Leg[i].limit_tao[2] * 0.2);

        //pos_taod[4] += limitw(vmc[i].param.spd_dj[4] * robotwb.Leg[i].q_pid.vff* dt, -robotwb.Leg[i].limit_tao[4] * 0.2, robotwb.Leg[i].limit_tao[4] * 0.2);
        //printf("pos_taod[4]=%f\n", pos_taod[4]);

        robotwb.Leg[i].pos_taod[0] = pos_taod[0];//record
        robotwb.Leg[i].pos_taod[1] = pos_taod[1];//record
        robotwb.Leg[i].pos_taod[2] = pos_taod[2];//record
        robotwb.Leg[i].pos_taod[3] = pos_taod[3];//record
        robotwb.Leg[i].pos_taod[4] = pos_taod[4];//record
#else
        pos_taod[0] = robotwb.Leg[i].tao_q_i[0] * EN_Q_I_MIT_MODE;
        pos_taod[1] = robotwb.Leg[i].tao_q_i[1] * EN_Q_I_MIT_MODE;
#endif

        //************************************************************************************************************
        //**************************************************力反馈*************************************************
        //************************************************************************************************************

        if (stand_force_enable_flag[4] &&//all enable
            USE_FPOS_CONTROL&&
            pos_force_p.en_force_control_cal) {

            if (vmc[i].ground == 0) {//非着地下 力控屏蔽 仅靠导纳跟踪  妨碍Load力
                robotwb.Leg[i].tar_force_dis_n.x *= 0;
                robotwb.Leg[i].tar_force_dis_n.y *= 0;
                robotwb.Leg[i].tar_force_dis_n.z *= 0;

                robotwb.Leg[i].tar_torque_dis_n_mpc.y = robotwb.Leg[i].tar_torque_dis_n_mpc.z = 0;
            }

            //printf("leg=%d fx=%f fz=%f\n",i,robotwb.Leg[i].tar_force_dis_n.x ,robotwb.Leg[i].tar_force_dis_n.z);
#if 0  //转换地形坐标系
            force_n_to_bw(robotwb.Leg[i].tar_force_dis_n, &robotwb.Leg[i].tar_force_dis_g);
            converV_n_to_gw(robotwb.Leg[i].tar_force_dis_g, &robotwb.Leg[i].tar_force_h);
#else
            force_n_to_bw(robotwb.Leg[i].tar_force_dis_n, &robotwb.Leg[i].tar_force_h);
#endif

            force_to_tao(i, dt);//输出robotwb.Leg[i].tar_force_h前馈力计算      （1）
            ff_force_taod[0] = robotwb.Leg[i].taod[0] * vmc[i].ground;
            ff_force_taod[1] = robotwb.Leg[i].taod[1] * vmc[i].ground;
            ff_force_taod[2] = robotwb.Leg[i].taod[2] * vmc[i].ground;
            //-----
            Vect3 err_force_n = Vect3_zero, err_force_h = Vect3_zero;//反馈力计算									 		 （2）
            err_force_n.x = force_err_n.x*robotwb.Leg[i].f_pid[Xr].kp;
            err_force_n.y = force_err_n.y*robotwb.Leg[i].f_pid[Yr].kp;
            err_force_n.z = force_err_n.z*robotwb.Leg[i].f_pid[Zr].kp;

            force_n_to_bw(err_force_n, &err_force_h);
            force_to_tao_input(i, err_force_h, dt);

            fb_force_taod[0] = robotwb.Leg[i].taod[0] * vmc[i].ground;
            fb_force_taod[1] = robotwb.Leg[i].taod[1] * vmc[i].ground;
            fb_force_taod[2] = robotwb.Leg[i].taod[2] * vmc[i].ground;
            //

            ff_force_taod[4] = -robotwb.Leg[i].tar_torque_dis_n_mpc.y*R_W*FOOT_K;
            ff_force_taod[3] = robotwb.Leg[i].tar_torque_dis_n_mpc.z*R_W*FOOT_K;
            //printf("leg=%d tau0=%f tau1=%f tau2=%f ff_force_taod[4]=%f\n", i, ff_force_taod[0], ff_force_taod[1], ff_force_taod[2], ff_force_taod[4]);
        }
        else {
            fb_force_taod[0] = fb_force_taod[1] = fb_force_taod[2] = 0;
            ff_force_taod[0] = ff_force_taod[1] = ff_force_taod[2] = 0;
            ff_force_taod[4] = 0; ff_force_taod[3] = 0;
        }

        robotwb.Leg[i].fb_force_taod[0] = fb_force_taod[0];//record
        robotwb.Leg[i].fb_force_taod[1] = fb_force_taod[1];//record
        robotwb.Leg[i].fb_force_taod[2] = fb_force_taod[2];//record
        robotwb.Leg[i].ff_force_taod[0] = ff_force_taod[0];//record
        robotwb.Leg[i].ff_force_taod[1] = ff_force_taod[1];//record
        robotwb.Leg[i].ff_force_taod[2] = ff_force_taod[2];//record
//************************************************************************************************************
//*************************************************输出合成***************************************************
//************************************************************************************************************
        if (gait_ww.state_gait <= 2) {//初始化或摆动仅角度模式
            robotwb.Leg[i].taod[0] = pos_taod[0] * pos_force_enable[0];
            robotwb.Leg[i].taod[1] = pos_taod[1] * pos_force_enable[0];
            robotwb.Leg[i].taod[2] = pos_taod[2] * pos_force_enable[0];
            robotwb.Leg[i].taod[3] = pos_taod[3] * pos_force_enable[0];
            robotwb.Leg[i].taod[4] = pos_taod[4] * pos_force_enable[0];
        }
        else {
            robotwb.Leg[i].taod[0] = pos_taod[0] * pos_force_enable[0] +
                fb_force_taod[0] * pos_force_enable[1] * pos_force_p.en_force_control_out +
                ff_force_taod[0] * pos_force_enable[2] * pos_force_p.en_force_control_out;
            robotwb.Leg[i].taod[1] = pos_taod[1] * pos_force_enable[0] +
                fb_force_taod[1] * pos_force_enable[1] * pos_force_p.en_force_control_out +
                ff_force_taod[1] * pos_force_enable[2] * pos_force_p.en_force_control_out;
            robotwb.Leg[i].taod[2] = pos_taod[2] * pos_force_enable[0] +
                fb_force_taod[2] * pos_force_enable[1] * pos_force_p.en_force_control_out +
                ff_force_taod[2] * pos_force_enable[2] * pos_force_p.en_force_control_out;
            robotwb.Leg[i].taod[3] = pos_taod[3] * pos_force_enable[0] +
                fb_force_taod[3] * pos_force_enable[1] * pos_force_p.en_force_control_out +
                ff_force_taod[3] * pos_force_enable[2] * pos_force_p.en_force_control_out;
            robotwb.Leg[i].taod[4] = pos_taod[4] * pos_force_enable[0]*1 +
                fb_force_taod[4] * pos_force_enable[1] * pos_force_p.en_force_control_out*1 +
                ff_force_taod[4] * pos_force_enable[2] * pos_force_p.en_force_control_out;
        }
        set_motor_t(i);//CAN底层输出赋值

        END_POS end_pos;
        end_pos.x=0.16;
        end_pos.y=-0.12;
        end_pos.z=robotwb.base_z;//base =0.1
        END_POS end_att;
        end_att.x=0;
        end_att.y=0;//pit
        end_att.z=0;
#if 0
        float q_out[6];
        ik_pino(end_pos,end_att,q_out);
        //printf("ik_pino:%.2f %.2f %.2f %.2f %.2f %.2f\n",q_out[0],q_out[1],q_out[2],q_out[3],q_out[4],q_out[5]);
#else
        float q_out[6];
        ik_rbdl(end_pos,end_att,q_out);
       // printf("ik_rbdl:%.2f %.2f %.2f %.2f %.2f %.2f\n",q_out1[0],q_out1[1],q_out1[2],q_out1[3],q_out1[4],q_out1[5]);
#endif
        robotwb.arm_base_height_exp= q_out[0];

        robotwb.arm_q_exp[0][0] = q_out[1]*57.3;//
        robotwb.arm_q_exp[0][1] = q_out[2]*57.3;//
        robotwb.arm_q_exp[0][2] = q_out[3]*57.3;//
        robotwb.arm_q_exp[0][3] = q_out[4]*57.3;//
        robotwb.arm_q_exp[0][4] = q_out[5]*57.3;//

        robotwb.head_att_exp[0] = 0;
        robotwb.head_att_exp[1] = 0;

    }
}

void force_control_and_dis_climb(float dt)//底层==>位力混控制  站立步态！！
{
}

void force_control_and_dis_bound(float dt)//底层==>位力混控制  站立步态！！
{

}

void force_control_and_dis_rolling(float dt)//底层==>位力混控制  站立步态！！
{

}

void force_control_and_dis_trot(float dt)//底层==>位力混控制  TORT步态！！ DB
{

}

void force_control_and_dis_trot_air(float dt)//底层==>位力混控制  TORT步态！！ DB
{

}

void force_control_and_dis_trot_local(float dt)//底层==>位力混控制  TORT步态！！ DB
{

}
