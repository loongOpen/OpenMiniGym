//#include "cppTypes.h"
//#include "convexMPC_interface.h"
//#include "SolverMPC.h"
//#include "common_types.h"
//#include "convexMPC_interface.h"
//#include "RobotState.h"
//#include "mpc_locomotion.h"
//#include "base_struct.h"
//#include "gait_math.h"
//#include "locomotion_header.h"
//#define EN_GAIT_DIV 1
//#define TEST_MPC    0
//_HU_MODE _hu_model;
//ControlFSMData1 mpc_data;

//float mpc_win_rate = 0.66;
//int mpc_win_forward = 1;
//int iterationsBetweenMPC_default = 10;//MPC运行周期 2*15=30ms  *horizonLength(10)=0.3s->100Hz
//int iterationsBetweenMPC = 6;//MPC运行周期 2*15=30ms  *horizonLength(10)=0.3s->100Hz
//int iterationsBetweenMPC_d[4] = { 6 };//MPC运行周期 2*15=30ms  *horizonLength(10)=0.3s->100Hz
//int horizonLength = MPC_PREDIC_LEN;//预测窗口
//float cmpc_x_drag = 3;//拖拽参数
//float ki_mpc[2] = { 1,1 };//Roll Pit积分系数
//float dtMPC;
//int iterationCounter = 0;
//int iterationCounter_div[5] = { 0 };
//Vec5m<float> f_ff[2];//fx fy fz my mz
//Vec5m<float> f_ff_n[2];
//Vec5m<float> f_ff_n_list[2][MPC_PREDIC_LEN];
//Vec4m<float> swingTimes;
//Mat3m<float> Kp, Kd, Kp_stance, Kd_stance;
//bool firstRun = true;
//bool firstSwing[4];
//float swingTimeRemaining[4];
//float stand_traj[6];
//int current_gait = IDLE;//4 STAND
//int gaitNumber;

//Vec3m<float> world_position_desired;
//Vec3m<float> rpy_int;
//Vec3m<float> rpy_comp;
//Vec3m<float> pFoot[2];

//float x_comp_integral = 0;
//float trajAll[12 * 36];
//int _nMPC_segments;
//int f_list_cnt = 0;
//float f_list_cnt_timer = 0;
//int* _mpc_table;

//Array4i _offsets = Vec4m<int>(0, 0, 0, 0); // offset in mpc segments
//Array4i _durations = Vec4m<int>(100, 100, 100, 100); // duration of step in mpc segments
//Array4f _offsetsFloat = Vec4m<float>(0, 0, 0, 0); // offsets in phase (0 to 1)
//Array4f _durationsFloat = Vec4m<float>(100, 100, 100, 100); // durations in phase (0 to 1)

//int _iteration;
//int _nIterations = 0;//
//float _phase;

//void ConvexMPCLocomotion_updateMPCIfNeeded(int *mpcTable, bool omniMode, float dt);
//StateEstimate seResult;
//DesiredStateData stateCommand;

//void setIterations(int iterationsPerMPC, int currentIteration)
//{
//    _nIterations = horizonLength;//
//    _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
//    _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations)) / (float)(iterationsPerMPC * _nIterations);
//}

//enum class CoordinateAxis { X, Y, Z };
///*!
// * Convert radians to degrees 转化弧度到度数
// */

//float rad2deg(float rad) {

//    return rad * float(180) / float(3.1415926);
//}

///*!
// * Convert degrees to radians
// */

//float deg2rad(float deg) {

//    return deg * float(3.1415926) / float(180);
//}

//Mat3m<float> coordinateRotation(CoordinateAxis axis, float theta) {

//    float s = std::sin(theta);
//    float c = std::cos(theta);

//    Mat3m<float> R1;

//    if (axis == CoordinateAxis::X) {
//        R1 << 1, 0, 0, 0, c, s, 0, -s, c;
//    }
//    else if (axis == CoordinateAxis::Y) {
//        R1 << c, 0, -s, 0, 1, 0, s, 0, c;
//    }
//    else if (axis == CoordinateAxis::Z) {
//        R1 << c, s, 0, -s, c, 0, 0, 0, 1;
//    }

//    return R1;
//}

//Mat3m<float> rpyToRotMat(const Vec3m<float> v) {
//    Mat3m<float> m = coordinateRotation(CoordinateAxis::X, v[0]) *
//        coordinateRotation(CoordinateAxis::Y, v[1]) *
//        coordinateRotation(CoordinateAxis::Z, v[2]);
//    return m;
//}

//Quat<float> rotationMatrixToQuaternion(
//    Mat3m<float> r1) {
//    Quat<float> q;
//    Mat3m<float> r = r1.transpose();
//    float  tr = r.trace();
//    if (tr > 0.0) {
//        float  S = sqrt(tr + 1.0) * 2.0;
//        q(0) = 0.25 * S;
//        q(1) = (r(2, 1) - r(1, 2)) / S;
//        q(2) = (r(0, 2) - r(2, 0)) / S;
//        q(3) = (r(1, 0) - r(0, 1)) / S;
//    }
//    else if ((r(0, 0) > r(1, 1)) && (r(0, 0) > r(2, 2))) {
//        float  S = sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
//        q(0) = (r(2, 1) - r(1, 2)) / S;
//        q(1) = 0.25 * S;
//        q(2) = (r(0, 1) + r(1, 0)) / S;
//        q(3) = (r(0, 2) + r(2, 0)) / S;
//    }
//    else if (r(1, 1) > r(2, 2)) {
//        float  S = sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
//        q(0) = (r(0, 2) - r(2, 0)) / S;
//        q(1) = (r(0, 1) + r(1, 0)) / S;
//        q(2) = 0.25 * S;
//        q(3) = (r(1, 2) + r(2, 1)) / S;
//    }
//    else {
//        float  S = sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
//        q(0) = (r(1, 0) - r(0, 1)) / S;
//        q(1) = (r(0, 2) + r(2, 0)) / S;
//        q(2) = (r(1, 2) + r(2, 1)) / S;
//        q(3) = 0.25 * S;
//    }
//    return q;
//}

//Quat<float> rpyToQuat(const Vec3m<float> rpy) {

//    Mat3m<float> R1 = rpyToRotMat(rpy);
//    Quat<float> q = rotationMatrixToQuaternion(R1);

//    return q.normalized();
//}

//int* mpc_gait()
//{
//    for (int i = 0; i < _nIterations; i++)
//    {
//        int iter = (i + _iteration + 1) % _nIterations;
//        Array4i progress = iter - _offsets;
//        for (int j = 0; j < 2; j++)
//        {
//            if (progress[j] < 0) progress[j] += _nIterations;

//            if (vmc_all.gait_mode == STAND_RC || vmc_all.gait_mode == TROT|| vmc_all.gait_mode == WALK) {

//                if (seResult.contactEstimate[j] > 0.5) {
//                    _mpc_table[i * 2 + j] = 1;//着地
//                }
//                else {
//                    _mpc_table[i * 2 + j] = 0;
//                }
//            }
//            else {

//                if (progress[j] < _durations[j])
//                    _mpc_table[i * 2 + j] = 1;//着地
//                else
//                    _mpc_table[i * 2 + j] = 0;
//            }
//        }
//    }

//    return _mpc_table;
//}

//int* mpc_gait_real()
//{
//    static float timer = 0;
//    int id_swap[4] = { 0,2,1,3 };
//    int win_id = 0;
//    int win_div = 0;//分段位置
//    int win_len_next = 0;
//    float st_time_all = vmc_all.stance_time + vmc_all.delay_time[2];
//    float sw_time_all = vmc_all.gait_time[1] - vmc_all.stance_time;

//    for (int i = 0; i < _nIterations; i++)
//    {
//        for (int j = 0; j < 2; j++)
//        {
//            vmc[id_swap[j]].st_phase = LIMIT(vmc[id_swap[j]].st_phase, 0, st_time_all);
//            vmc[id_swap[j]].sw_phase = LIMIT(vmc[id_swap[j]].sw_phase, 0, sw_time_all);

//            if (vmc_all.gait_mode == TROT || vmc_all.gait_mode == G_ETL || vmc_all.gait_mode == WALK) {
//                if (seResult.contactEstimate[j] > 0.5)//着地
//                {
//                    win_div = LIMIT((int)((st_time_all - vmc[id_swap[j]].st_phase - dtMPC * mpc_win_forward) / dtMPC
//                        + 0 * fmod(st_time_all - vmc[id_swap[j]].st_phase - dtMPC * mpc_win_forward, dtMPC)), 0, _nIterations);//计算剩下的时间
//                    win_len_next = (int)(sw_time_all / dtMPC);

//                    if (i <= win_div || i >= win_div + win_len_next)
//                        _mpc_table[i * 2 + j] = 1;
//                    else
//                        _mpc_table[i * 2 + j] = 0;

//                    if(vmc_all.gait_mode==WALK)_mpc_table[i * 2 + j] = 1;
//                }
//                else//摆动
//                {
//                    win_div = LIMIT((int)((sw_time_all - vmc[id_swap[j]].sw_phase - dtMPC * mpc_win_forward) / dtMPC
//                        + 0 * fmod(sw_time_all - vmc[id_swap[j]].sw_phase - dtMPC * mpc_win_forward, dtMPC)), 0, _nIterations);
//                    win_len_next = (int)(st_time_all / dtMPC);
//                    if (i <= win_div || i >= win_div + win_len_next)
//                        _mpc_table[i * 2 + j] = 0;
//                    else
//                        _mpc_table[i * 2 + j] = 1;
//                    if (vmc_all.gait_mode == WALK)_mpc_table[i * 2 + j] = 0;
//                }

//            }
//            else if (vmc_all.gait_mode == G_ETL) {
//                if (seResult.contactEstimate[j] > 0.5)//着地
//                {
//                    float time_lf = gait_etl.sw_start[id_swap[j]] - gait_etl.ti[id_swap[j]];

//                    win_div = LIMIT((int)((time_lf - dtMPC * mpc_win_forward) / dtMPC
//                        + fmod(time_lf - dtMPC * mpc_win_forward, dtMPC)), 0, _nIterations);//计算剩下的时间
//                    win_len_next = (int)(sw_time_all / dtMPC);

//                    if (i <= win_div || i > win_div + win_len_next)
//                        _mpc_table[i * 2 + j] = 1;
//                    else
//                        _mpc_table[i * 2 + j] = 0;
//                }
//                else//摆动
//                {
//                    float time_lf = gait_etl.sw_end[id_swap[j]] - gait_etl.ti[id_swap[j]];

//                    win_div = LIMIT((int)((time_lf - dtMPC * mpc_win_forward) / dtMPC
//                        + fmod(time_lf - dtMPC * mpc_win_forward, dtMPC)), 0, _nIterations);

//                    win_len_next = (int)(st_time_all / dtMPC);
//                    if (i <= win_div || i > win_div + win_len_next)
//                        _mpc_table[i * 2 + j] = 0;
//                    else
//                        _mpc_table[i * 2 + j] = 1;

//                }

//            }
//            else if (vmc_all.gait_mode == STAND_RC) {
//                timer += 0.05;
//                if (j == 0 && timer > 10)
//                    seResult.contactEstimate[j] = 0;

//                if (seResult.contactEstimate[j] > 0.5) {
//                    _mpc_table[i * 2 + j] = 1;//着地
//                }
//                else {
//                    _mpc_table[i * 2 + j] = 0;
//                }
//            }
//            else {//IDLE
//                _mpc_table[i * 2 + j] = 1;//着地
//            }
//#if TEST_MPC
//            _mpc_table[i * 2 + j] = 1;//着地
//#endif
//        }
//    }

//    return _mpc_table;
//}

//void convert_mpc_data_in(void)//转换我的数据到MPC MIT
//{
//    int id_swap[4] = { 0,2,1,3 };//Moco定义顺序
//    char G[4] = { 0 };
//    double F1c[3], F2c[3], F3c[3], F4c[3];


//    G[0] = robotwb.Leg[0].is_ground;
//    G[1] = robotwb.Leg[1].is_ground;
//    seResult.contactEstimate[0] = G[0];
//    seResult.contactEstimate[1] = G[1];


//    //世界系下的足端位置
//    pFoot[0][Xrw] = (robotwb.Leg[0].epos_n.x) + vmc_all.pos_n.x;
//    pFoot[0][Yrw] = (-robotwb.Leg[0].epos_n.y) - vmc_all.pos_n.y;
//    pFoot[0][Zrw] = (robotwb.Leg[0].epos_n.z) + vmc_all.pos_n.z;

//    pFoot[1][Xrw] = (robotwb.Leg[1].epos_n.x) + vmc_all.pos_n.x;
//    pFoot[1][Yrw] = (-robotwb.Leg[1].epos_n.y) - vmc_all.pos_n.y;
//    pFoot[1][Zrw] = (robotwb.Leg[1].epos_n.z) + vmc_all.pos_n.z;
//    //printf("%d %d %d %d\n", G[0], G[1], G[2], G[3]);

//    //Vec4<float> contactEstimate;//接触估计
//    //Vec3<float> position;//位置
//    //Vec3<float> vBody;//速度
//    //Quat<float> orientation;//四元数
//    //Vec3<float> omegaBody;//角速度
//    //RotMat<float> rBody;//旋转矩阵
//    //Vec3<float> rpy;//欧拉角

//    //Vec3<float> omegaWorld;//世界坐标角速度
//    //Vec3<float> vWorld;//速度
//    //Vec3<float> aBody, aWorld;//加速度，世界坐标下加速度
//    //float* p = seResult.position.data();
//    //float* v = seResult.vWorld.data();
//    //float* w = seResult.omegaWorld.data();
//    //float* q = seResult.orientation.data();

//#if TEST_MPC//mpc test
//    G[0] = 1;//右腿
//    G[1] = 1;
//    seResult.contactEstimate[0] = G[0];
//    seResult.contactEstimate[1] = G[1];

//    vmc_all.pos_n.x = 0;
//    vmc_all.pos_n.y = 0;
//    vmc_all.pos_n.z = 0.45;

//    robotwb.exp_spd_ng.x = 0;
//    robotwb.exp_spd_ng.y = 0;
//    vmc_all.tar_pos.z = 0.45;
//    //--
//    robotwb.Leg[0].epos_n.x = 0;//右腿 足端
//    robotwb.Leg[0].epos_n.y = 0.1;
//    robotwb.Leg[0].epos_n.z = -0.45;

//    robotwb.Leg[1].epos_n.x = 0;
//    robotwb.Leg[1].epos_n.y = -0.1;
//    robotwb.Leg[1].epos_n.z = -0.45;
//    //世界系下的足端位置
//    pFoot[0][Xrw] = (robotwb.Leg[0].epos_n.x) + vmc_all.pos_n.x;//右腿
//    pFoot[0][Yrw] = (-robotwb.Leg[0].epos_n.y) - vmc_all.pos_n.y;
//    pFoot[0][Zrw] = (robotwb.Leg[0].epos_n.z) + vmc_all.pos_n.z;
//    //printf("%f %f %f\n", pFoot[0][0], pFoot[0][1], pFoot[0][2]);
//    pFoot[1][Xrw] = (robotwb.Leg[1].epos_n.x) + vmc_all.pos_n.x;
//    pFoot[1][Yrw] = (-robotwb.Leg[1].epos_n.y) - vmc_all.pos_n.y;
//    pFoot[1][Zrw] = (robotwb.Leg[1].epos_n.z) + vmc_all.pos_n.z;

//    seResult.position[0] = (vmc_all.pos_n.x);
//    seResult.position[1] = -(vmc_all.pos_n.y);
//    seResult.position[2] = vmc_all.pos_n.z;
//    //printf("pos=%f %f %f\n", seResult.position[0], seResult.position[1], seResult.position[2]);
//    seResult.vWorld[0] = 0;
//    seResult.vWorld[1] = 0;
//    seResult.vWorld[2] = 0;//原理向上为-我的   改为-好像不行
//    //printf("spd=%f %f %f\n", seResult.vWorld[0], seResult.vWorld[1], seResult.vWorld[2]);
//    seResult.omegaWorld[0] = 0 / 57.3;
//    seResult.omegaWorld[1] = 0 / 57.3;
//    seResult.omegaWorld[2] = 0 / 57.3;
//    seResult.omegaBody[0] = seResult.omegaWorld[0];
//    seResult.omegaBody[1] = seResult.omegaWorld[1];
//    seResult.omegaBody[2] = seResult.omegaWorld[2];

//    seResult.rpy[0] = 0 / 57.3;
//    seResult.rpy[1] = -0 / 57.3;
//    float yaw_temp = 0;
//    if (vmc_all.att[YAWr] == 180)
//        yaw_temp = 179.9;
//    if (vmc_all.att[YAWr] == -180)
//        yaw_temp = -179.9;

//    seResult.rpy[2] = yaw_temp / 57.3 * 0;//我不考虑机头

//    seResult.orientation = rpyToQuat(seResult.rpy);

//    stateCommand.stateDes[3] = 0 / 57.3;
//    stateCommand.stateDes[4] = -0 / 57.3;

//    stand_traj[0] = 0;
//    stand_traj[1] = -0;

//    int en_slip = 0;
//    stateCommand.stateDes[6] = robotwb.exp_spd_ng.x + slip_mode.vel.x*en_slip;
//    stateCommand.stateDes[7] = robotwb.exp_spd_ng.y - -slip_mode.vel.y*en_slip;
//    stateCommand.stateDes[8] = slip_mode.vel.z*en_slip;

//    robotwb.exp_rate.yaw = 0;
//    stateCommand.stateDes[11] = robotwb.exp_rate.yaw / 57.3;

//    stateCommand.stateDes[5] += stateCommand.stateDes[11] * T_MPC;//Yaw

//#else
//    seResult.position[0] = (vmc_all.pos_n.x + robotwb.mess_off[Xr]);
//    seResult.position[1] = -(vmc_all.pos_n.y + robotwb.mess_off[Yr]);
//    seResult.position[2] = vmc_all.pos_n.z;
//    //printf("pos=%f %f %f\n", seResult.position[0], seResult.position[1], seResult.position[2]);
//    seResult.vWorld[0] = vmc_all.spd_n.x;
//    seResult.vWorld[1] = -vmc_all.spd_n.y;
//    seResult.vWorld[2] = -vmc_all.spd_n.z;//原理向上为-我的   改为-好像不行
//    //printf("spd=%f %f %f\n", seResult.vWorld[0], seResult.vWorld[1], seResult.vWorld[2]);
//    seResult.omegaWorld[0] = vmc_all.att_rate[ROLr] / 57.3;
//    seResult.omegaWorld[1] = -vmc_all.att_rate[PITr] / 57.3;
//    seResult.omegaWorld[2] = vmc_all.att_rate[YAWr] / 57.3;
//    seResult.omegaBody[0] = seResult.omegaWorld[0];
//    seResult.omegaBody[1] = seResult.omegaWorld[1];
//    seResult.omegaBody[2] = seResult.omegaWorld[2];

//    seResult.rpy[0] = vmc_all.att[ROLr] / 57.3;
//    seResult.rpy[1] = -vmc_all.att[PITr] / 57.3;
//    float yaw_temp = vmc_all.att[YAWr];
//    if (vmc_all.att[YAWr] == 180)
//        yaw_temp = 179.9;
//    if (vmc_all.att[YAWr] == -180)
//        yaw_temp = -179.9;

//    seResult.rpy[2] = yaw_temp / 57.3 * 0;//我不考虑机头

//    //printf("att=%f %f %f\n", seResult.rpy[0], seResult.rpy[1], seResult.rpy[2]);
//    //printf("rate=%f %f %f\n", seResult.omegaBody[0], seResult.omegaBody[1], seResult.omegaBody[2]);
//    seResult.orientation = rpyToQuat(seResult.rpy);

//    stateCommand.stateDes[3] = vmc_all.tar_att[ROLr] / 57.3;
//    stateCommand.stateDes[4] = -vmc_all.tar_att[PITr] / 57.3;

//    stand_traj[0] = robotwb.exp_pos_n.x;
//    stand_traj[1] = -robotwb.exp_pos_n.y;
//    if (vmc_all.gait_mode == WALK) {
//        robotwb.exp_spd_ng.x = vmc_all.tar_spd_walk[Xr];
//        robotwb.exp_spd_ng.y = -vmc_all.tar_spd_walk[Yr];
//    }
//    int en_slip = 0;
//    stateCommand.stateDes[6] = robotwb.exp_spd_ng.x;//+ slip_mode.vel.x*en_slip;
//    stateCommand.stateDes[7] = robotwb.exp_spd_ng.y;// - -slip_mode.vel.y*en_slip;
//    stateCommand.stateDes[8] = 0;//slip_mode.vel.z*en_slip;

//    stateCommand.stateDes[11] = robotwb.exp_rate.yaw / 57.3;

//    stateCommand.stateDes[5] += stateCommand.stateDes[11] * T_MPC;//Yaw
//#endif
//}


//void convert_mpc_data_out(float dt) {//转换MIT 到我的数据

//    //MIT     1         0
//    //        3         2
//    //MOCO    2         0
//    //        3         1
//#if 1//采用连续预测输出

//    for (int j = 0; j < gait_mpc_trot._nIterations; j++) {
//        robotwb.Leg[0].mpc_table[j] = gait_mpc_trot._mpc_table[0 * 2 + j];
//        robotwb.Leg[1].mpc_table[j] = gait_mpc_trot._mpc_table[1 * 2 + j];
//    }
//    robotwb.Leg[0].tar_force_dis_n_mpc.x = f_ff_n[0][Xrw];//右腿
//    robotwb.Leg[0].tar_force_dis_n_mpc.y = -f_ff_n[0][Yrw];//
//    robotwb.Leg[0].tar_force_dis_n_mpc.z = f_ff_n[0][Zrw];//
//    robotwb.Leg[0].tar_torque_dis_n_mpc.y = f_ff_n[0][3];// my
//    robotwb.Leg[0].tar_torque_dis_n_mpc.z = f_ff_n[0][4];// mz

//    robotwb.Leg[1].tar_force_dis_n_mpc.x = f_ff_n[1][Xrw];//
//    robotwb.Leg[1].tar_force_dis_n_mpc.y = -f_ff_n[1][Yrw];//
//    robotwb.Leg[1].tar_force_dis_n_mpc.z = f_ff_n[1][Zrw];//
//    robotwb.Leg[1].tar_torque_dis_n_mpc.y = f_ff_n[1][3];// my
//    robotwb.Leg[1].tar_torque_dis_n_mpc.z = f_ff_n[1][4];// mz

//    f_list_cnt_timer += dt;
//    if (f_list_cnt_timer > dtMPC && 0) {
//        f_list_cnt_timer = 0;
//        f_list_cnt++;
//        //printf("mpc_forward out %d\n",f_list_cnt);
//    }
//    f_list_cnt = LIMIT(f_list_cnt, 0, MPC_PREDIC_LEN);
//#endif
//#if 0
//    printf("\n");
//    for (int i = 0; i < 4; i++) {
//        printf("leg=%d qp=%f %f %f mpc=%f %f %f\n",
//            i,
//            robotwb.Leg[i].tar_force_dis_n_qp.x,
//            robotwb.Leg[i].tar_force_dis_n_qp.y,
//            robotwb.Leg[i].tar_force_dis_n_qp.z,
//            robotwb.Leg[i].tar_force_dis_n_mpc.x,
//            robotwb.Leg[i].tar_force_dis_n_mpc.y,
//            robotwb.Leg[i].tar_force_dis_n_mpc.z);
//    }
//#endif
//}

//void ConvexMPCLocomotion_run(float dt) {//for mpc
//    static int init = 0;
//    static int gaitNumber_reg = 0;
//    if (!init)
//    {
//#if TEST_MPC
//        firstRun = 1;
//        gaitNumber = STAND_RC;
//#endif
//        init = 1;
//        f_list_cnt = 0;//list力预测输出
//        _mpc_table = new int[horizonLength * 2];
//        MPC_gait_init(horizonLength);
//    }
//    bool omniMode = false;//万向模式
//    //dt = T_MPC;

//    gaitNumber = vmc_all.gait_mode;
//    //计算覆盖一个步态周期下需要的迭代次数
//    if (gaitNumber == STAND_RC || gaitNumber == IDLE)
//        iterationsBetweenMPC = iterationsBetweenMPC_default;
//    else if (vmc_all.gait_mode == G_ETL) {
//        mpc_win_rate = 1;
//        float temp = ((vmc_all.gait_time[1]) / horizonLength) *mpc_win_rate;
//        iterationsBetweenMPC = (int)(temp / dt) + fmod(temp, dt) * 0;
//    }
//    else {
//        float temp = ((vmc_all.gait_time[1] + vmc_all.delay_time[2]) / horizonLength) *mpc_win_rate;
//        iterationsBetweenMPC = (int)(temp / dt) + fmod(temp, dt) * 0;

//    }
//    iterationsBetweenMPC = iterationsBetweenMPC_default;

//    robotwb._nIterations = horizonLength;
//    robotwb.dtMPC = dtMPC = dt *  iterationsBetweenMPC;//MPC一个窗口对应的间隔时间 2*15=30ms  同时也是MPC刷新的频率
//    gait_mpc_trot.dtMPC = dtMPC;

//    convert_mpc_data_in();//获取我的数据

//    if (((gaitNumber == STAND_RC) && gaitNumber_reg != STAND_RC) || firstRun)
//    {
//        printf("MPC::Transition to standing %d %d\n", gaitNumber, gaitNumber_reg);
//        stand_traj[0] = seResult.position[0];
//        stand_traj[1] = seResult.position[1];
//        stand_traj[2] = vmc_all.tar_pos.z;
//        stand_traj[3] = 0;
//        stand_traj[4] = 0;
//        stand_traj[5] = seResult.rpy[2];
//        world_position_desired[0] = stand_traj[0];
//        world_position_desired[1] = stand_traj[1];
//    }

//    Vec3m<float> v_des_robot(stateCommand.stateDes[6], stateCommand.stateDes[7], 0);
//    Vec3m<float> v_des_world = v_des_robot;
//    Vec3m<float> v_robot = seResult.vWorld;

//    //Integral-esque pitche and roll compensation
//    rpy_int[1] += T_MPC * (stateCommand.stateDes[4] - seResult.rpy[1]) / 0.2;//PITr
//    rpy_int[0] += T_MPC * (stateCommand.stateDes[3] - seResult.rpy[0]) / 0.2;

//    //height_interger += T_MPC * (robotwb.exp_pos_n.z - vmc_all.pos_n.z)*0.15;
//    //height_interger = LIMIT(height_interger, -0.015, 0.015);
//    //Integral-esque pitche and roll compensation
//    //pitche??roll ?????????????????
//    rpy_int[0] = fminf(fmaxf(rpy_int[0], -.15), .15);//?????
//    rpy_int[1] = fminf(fmaxf(rpy_int[1], -.15), .15);
//    rpy_comp[1] = 0.2 * rpy_int[1];
//    rpy_comp[0] = 0.2 * rpy_int[0] * (gaitNumber != PRONK);  //turn off for pronking

//    //非站定下目标位置 通过累加目标速度完成
//    if (vmc_all.gait_mode != STAND_RC) {//步态模式移动
//        world_position_desired += T_MPC * Vec3m<float>(v_des_world[0], v_des_world[1], 0);
//    }

//    // some first time initialization
//    if (firstRun&&seResult.position[2] != 0)//初始化期望上电
//    {
//        world_position_desired[0] = seResult.position[0];
//        world_position_desired[1] = seResult.position[1];
//        world_position_desired[2] = seResult.rpy[2];//初始化航向
//        firstRun = false;
//        printf("firstRun MPC\n");
//    }

//    // calc gait
//    setIterations(iterationsBetweenMPC, iterationCounter);// ?
//    if (vmc_all.gait_mode == G_ETL) {
//        setIterationsMPC(&gait_mpc_trot, iterationsBetweenMPC, iterationCounter);
//    }

//    iterationCounter++;//MPC控制频率计数用

//    int* mpcTable = mpc_gait();//MPC 预测着地状态 从当期计算开始
//    if (vmc_all.gait_mode == G_ETL) {
//        getMpcTable_Gait(&gait_mpc_trot);//获取MIT 的MPCtable
//    }
//    ConvexMPCLocomotion_updateMPCIfNeeded(mpcTable, omniMode, dt);//求解MPC问题

//    convert_mpc_data_out(dt);

//    gaitNumber_reg = gaitNumber;
//}

//void GaitLocomotion_run(float dt) {//for QP gait
//    static int init = 0;
//    static int gaitNumber_reg = 0;
//    static int _hu_model_gait_mode_reg;
//    static int st_sw = 0;
//    static float st_sw_timer = 0;
//    if (!init)
//    {
//        init = 1;
//        f_list_cnt = 0;//list力预测输出
//        _mpc_table = new int[horizonLength * 2];
//        MPC_gait_init(horizonLength);
//    }
//    bool omniMode = false;//万向模式
//    dt = T_MPC;

//    int ground_num = 0;
//    for (int i = 0; i < 2; i++) {
//        if (robotwb.Leg[i].is_ground)
//        {
//            ground_num++;
//        }
//    }

//    if (st_sw==0&&_hu_model.gait_mode != _hu_model_gait_mode_reg) {//行为切换
//        st_sw_timer = 0;
//        st_sw = 1;
//        printf("Gait::switching!\n");
//        _hu_model.gait_switching = 1;

//        Gait_switch_init(&gait_mpc_trot, iterationsBetweenMPC, iterationCounter_div[0],  dtMPC, 0);
//        Gait_switch_init(&gait_mpc_trot, iterationsBetweenMPC, iterationCounter_div[1],  dtMPC, 1);
//        iterationCounter_div[0] = 0;
//        iterationCounter_div[1] = 0;
//        iterationCounter_div[2] = 0;
//        iterationCounter_div[3] = 0;
//        iterationCounter_div[4] = 0;
//    }
//    else if (st_sw == 1) {
//        if ((iterationCounter_div[4] % iterationsBetweenMPC) == 0) {
//            printf("Gait update once!\n");
//            _hu_model.gait_switching = 0;
//            MPC_gait_init_fast(horizonLength);

//            //iterationCounter = 0;

//            //iterationCounter_div[0] = 0;
//            //iterationCounter_div[1] = 0;
//            //iterationCounter_div[2] = 0;
//            //iterationCounter_div[3] = 0;
//            st_sw = 0;
//        }
//    }

//    MPC_gait_update(horizonLength);

//    _hu_model_gait_mode_reg = _hu_model.gait_mode;

//    if (gaitNumber != vmc_all.gait_mode&&vmc_all.gait_mode == G_ETL) {
//        printf("MPC::Gait Reset ELT\n");
//        iterationCounter = 0;

//        iterationCounter_div[0] = 0;
//        iterationCounter_div[1] = 0;
//        iterationCounter_div[2] = 0;
//        iterationCounter_div[3] = 0;
//        iterationCounter_div[4] = 0;

//        setIterationsMPC(&gait_mpc_trot, iterationsBetweenMPC, iterationCounter);
//        convert_mpc_data_out(dt);

//        vmc_all.gait_alfa = gait_mpc_trot._durationsFloat[0];
//        vmc_all.stance_time = vmc_all.gait_time[1] * vmc_all.gait_alfa;

//        setMpcTable_Gait_sel(0, 1);
//        setMpcTable_Gait_sel(1, 1);

//        stateCommand.stateDes[5] = seResult.rpy[2];
//        //printf("vmc_all.gait_alfa =%f %f %f\n", vmc_all.gait_alfa, vmc_all.stance_time, vmc_all.gait_time[1]);
//    }

//    gaitNumber = vmc_all.gait_mode;
//    //计算覆盖一个步态周期下需要的迭代次数
//    if (gaitNumber == STAND_RC || gaitNumber == IDLE)
//        iterationsBetweenMPC = iterationsBetweenMPC_default;
//    else if (vmc_all.gait_mode == G_ETL) {
//        mpc_win_rate = 1;
//        float temp = ((vmc_all.gait_time[1]) / horizonLength) *mpc_win_rate;
//        iterationsBetweenMPC = (int)(temp / dt) + fmod(temp, dt) * 0;

//        temp = ((sw_t[0]) / horizonLength) *mpc_win_rate;
//        iterationsBetweenMPC_d[0] = (int)(temp / dt) + fmod(temp, dt) * 0;
//        temp = ((sw_t[1]) / horizonLength) *mpc_win_rate;
//        iterationsBetweenMPC_d[1] = (int)(temp / dt) + fmod(temp, dt) * 0;

//        iterationsBetweenMPC_d[0] = iterationsBetweenMPC_d[1] = iterationsBetweenMPC_d[2] = iterationsBetweenMPC_d[3] = iterationsBetweenMPC;
//    }
//    else if(1){
//        float temp = ((vmc_all.gait_time[1] + vmc_all.delay_time[2]) / horizonLength) *mpc_win_rate;
//        iterationsBetweenMPC = (int)(temp / dt) + fmod(temp, dt) * 0;

//    }
//    robotwb._nIterations = horizonLength;
//    robotwb.dtMPC = dtMPC = dt * iterationsBetweenMPC;//MPC一个窗口对应的间隔时间 2*15=30ms  同时也是MPC刷新的频率
//    gait_mpc_trot.dtMPC = dtMPC;

//    if (vmc_all.gait_mode == G_ETL) {
//        //setIterationsMPC(&gait_mpc_trot, iterationsBetweenMPC, iterationCounter);
//        setIterations_d(&gait_mpc_trot, iterationsBetweenMPC_d[0], iterationCounter_div[0], 0);
//        setIterations_d(&gait_mpc_trot, iterationsBetweenMPC_d[1], iterationCounter_div[1], 1);
//    }

//    iterationCounter++;//MPC控制频率计数用

//    iterationCounter_div[0]++;//MPC控制频率计数用
//    iterationCounter_div[1]++;//MPC控制频率计数用
//    iterationCounter_div[2]++;//MPC控制频率计数用
//    iterationCounter_div[3]++;//MPC控制频率计数用
//    iterationCounter_div[4]++;//整体计数

//    convert_mpc_data_out(dt);
//    gaitNumber_reg = gaitNumber;
//}

//void ConvexMPCLocomotion_updateMPCIfNeeded(int *mpcTable, bool omniMode, float dt) {

//    static float G_Reg[2] = { 0 };
//    static int gait_mode_reg = 0;
//    int mpc_update_force = 0;
//#if MPC_USE_REAL_TD
//    for (int i = 0; i < 2; i++) {
//        if (G_Reg[i] == 0 && seResult.contactEstimate[i] > 0.5&&vmc_all.gait_mode != STAND_RC) {//触地瞬间需要马上计算
//            mpc_update_force = 1;
//        }
//        if (G_Reg[i] == 1 && seResult.contactEstimate[i]==0&&vmc_all.gait_mode != STAND_RC) {//触地瞬间需要马上计算
//            mpc_update_force = 1;
//        }
//    }
//#endif
//    for (int i = 0; i < 2; i++) {
//        G_Reg[i] = seResult.contactEstimate[i];
//    }

//    if ((iterationCounter % iterationsBetweenMPC) == 0
//        //|| (mpc_update_force&&vmc_all.gait_mode != G_ETL)//对普通tort适应 着地后快速计算 用了反而不好
//        || vmc_all.force_update_mpc == 1
//        || mpc_update_force
//        )//控制周期
//    {
//        vmc_all.force_update_mpc = 0;

//        int* mpcTable_real = mpc_gait_real();//tort步态对table的处理
//        float* p = seResult.position.data();
//        float* v = seResult.vWorld.data();
//        float* w = seResult.omegaWorld.data();
//        float* q = seResult.orientation.data();
//        float r[6];//x1 x2 y1 y2 z1 z2
//        Vec3m<float> position_temp = seResult.position;//位置
//        for (int i = 0; i < 6; i++) {
//            r[i] = pFoot[i % 2][i / 2] - position_temp[i / 2];//x1 x2 y1 y2 z1 z2
//        }

//        float Q[12] = {
//            0.25, 0.25, 10,
//            2, 2, 50,
//            0, 0, 0.3,
//            0.2, 0.2, 0.1 };

//        float Q_stand[12] = {
//            20, 20, 15, //R P Y
//            35, 35, 100, //X Y Z
//            0.5, 0.5, 0.5, //DR DP DY
//            1, 1, 1 //VX VY VZ
//        };

//        float Q_trot[12] = {
//            300, 300, 120, //R P Y
//            155, 155, 1200, //X Y Z
//            0.1, 0.1, 10, //DR DP DY
//            0.5, 0.5, 1 //VX VY VZ
//        };
//        float yaw = seResult.rpy[2];
//        float* weights = Q;
//        float alpha = 1e-4; // make setting eventually 越小力大
//        float alpha_end = 5e-4;
//        Vec3m<float> v_des_robot(stateCommand.stateDes[6], stateCommand.stateDes[7], 0);
//        Vec3m<float> v_des_world = v_des_robot;

//        if (vmc_all.gait_mode == STAND_RC||TEST_MPC)//站立
//        {
//             vmc_all.tar_pos.z = 0.486+Hw;
//            for (int i = 0; i < 12; i++)
//                Q[i] = Q_stand[i];

//            float trajInitial[12] = {(float)stateCommand.stateDes[3]+ (float)rpy_comp[0] * ki_mpc[0],//roll
//                                     (float)stateCommand.stateDes[4]+ (float)rpy_comp[1] * ki_mpc[1] ,//- vmc_all.ground_att_cmd[PITr] / 57.3*!TEST_MPC*0 /*-hw_i->state_estimator->se_ground_pitch*/, //pitch
//                                     (float)stand_traj[5]/*+(float)stateCommand->data.stateDes[11]*/,//yaw

//                                     (float)stand_traj[0]/*+(float)fsm->main_control_settings.p_des[0]*/,//X
//                                     (float)stand_traj[1]/*+(float)fsm->main_control_settings.p_des[1]*/,//Y
//                                     (float)vmc_all.tar_pos.z,//(float)0.26/*fsm->main_control_settings.p_des[2]*/,
//                                     0,0,0,0,0,0 };
//            if (fabs(To_180_degreesw(seResult.rpy[2] * 57.3 - stand_traj[5] * 57.3)) > 45) {
//                stand_traj[5] = seResult.rpy[2];
//            }

//            //printf("%f %f\n", stand_traj[5]*57.3, seResult.rpy[2] * 57.3);
//            printf("tar_x: %f %f\n", stand_traj[0], vmc_all.pos_n.x);
//            printf("tar_z: %f %f\n", vmc_all.tar_pos.z, vmc_all.pos_n.z);
//            printf("attp: %f %f\n",( stateCommand.stateDes[4]+ (float)rpy_comp[1] * ki_mpc[1]) *57.3, seResult.rpy[1] * 57.3);
//            printf("attr: %f %f\n", stateCommand.stateDes[3] * 57.3, seResult.rpy[0] * 57.3);
//            printf("spd: %f %f\n", seResult.vWorld[0], seResult.vWorld[2]);
//#if TEST_MPC&&0
//            printf("exp_J= ");
//            for (int j = 0; j < 12; j++) {
//                printf("%.2f ", trajInitial[j]);
//            }
//            printf("\n");
//#endif
//            //变成mpc问题需要的格式
//            for (int i = 0; i < horizonLength; i++) {
//                for (int j = 0; j < 12; j++) {
//                    trajAll[12 * i + j] = trajInitial[j];
//                }
//            }
//        }
//        else if (vmc_all.gait_mode == TROT || vmc_all.gait_mode == G_ETL || vmc_all.gait_mode == WALK)//正常步态
//        {
//            vmc_all.tar_pos.z = 0.486+Hw;
//            if (gait_mode_reg != vmc_all.gait_mode) {
//                printf("MPC::ELT gait reset!\n");
//                world_position_desired[0] = position_temp[0];
//                world_position_desired[1] = position_temp[1];
//                stateCommand.stateDes[5] = seResult.rpy[2];
//            }

//            for (int i = 0; i < 12; i++)
//                Q[i] = Q_trot[i];
//#if 0//不控制位置
//            world_position_desired[0] = position_temp[0];
//            world_position_desired[1] = position_temp[1];
//#endif
//            //printf("%f %f\n", vmc_all.tar_pos.z, vmc_all.pos_n.z);
//            const float max_pos_error = .1;//轨迹跟踪误差
//            float xStart = world_position_desired[0];
//            float yStart = world_position_desired[1];

////在误差范围内，更新目标值
//            if (xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
//            if (p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

//            if (yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
//            if (p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

//            world_position_desired[0] = xStart;
//            world_position_desired[1] = yStart;

////机体初始参考轨迹
//            if (fabs(To_180_degreesw(seResult.rpy[2] * 57.3 - stateCommand.stateDes[5] * 57.3)) > 45) {//180度会歧义
//                printf("MPC::reset yaw over!!!\n");
//                stateCommand.stateDes[5] = seResult.rpy[2];
//            }
//            float trajInitial[12] = {
//                (float)rpy_comp[0] * ki_mpc[0],  // 0 roll
//                (float)rpy_comp[1] * ki_mpc[1],//- vmc_all.ground_att_cmd[PITr] / 57.3 * 1/*-hw_i->state_estimator->se_ground_pitch*/,    // 1 pitch
//                (float)stateCommand.stateDes[5],    // 2 yaw
//                xStart,                                   // 3 posx
//                yStart,                                   // 4 posy
//                (float)vmc_all.tar_pos.z,//(float)0.26,      // 5		                    posz
//                0,                                        // 6
//                0,                                        // 7
//                (float)stateCommand.stateDes[11],  // 8//转向速度
//                v_des_world[0],                           // 9
//                v_des_world[1],                           // 10
//                0 };                                       // 11

//            printf("spdx: %f %f\n", v_des_world[0], seResult.vWorld[0]);
//            printf("spdy: %f %f\n", v_des_world[1], seResult.vWorld[1]);
//            printf("tar_z: %f %f\n", vmc_all.tar_pos.z, vmc_all.pos_n.z);
//            printf("tar_yaw=%f %f\n", stateCommand.stateDes[11], stateCommand.stateDes[5]);
//            //变成mpc问题需要的格式
//            //轨迹为当前时刻向后预测一个步态周期的轨迹 按匀速运动计算
//            for (int i = 0; i < horizonLength; i++)
//            {
//                for (int j = 0; j < 12; j++)
//                    trajAll[12 * i + j] = trajInitial[j];
//#if 1
//                if (i == 0) // start at current position  TODO consider not doing this
//                {
//                    //trajAll[3] = hw_i->state_estimator->se_pBody[0];
//                    //trajAll[4] = hw_i->state_estimator->se_pBody[1];
//                    //trajAll[2] = (float)stateCommand.stateDes[5];//
//                    trajAll[2] = seResult.rpy[2];
//                }
//                else
//                {
//                    trajAll[12 * i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
//                    trajAll[12 * i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
//                    trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * stateCommand.stateDes[11];

//                }
//#endif
//            }
//        }

//        Vec3m<float> pxy_act(p[0], p[1], 0);
//        Vec3m<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);

//        float pz_err = p[2] - vmc_all.pos_n.z;
//        Vec3m<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);
//        //printf("robotwb.max_torque_foot.y=%f %f\n", robotwb.max_torque_foot.y, robotwb.max_torque_foot.z);
//        setup_problem(dtMPC, horizonLength, vmc_robot_p.ground_mu, robotwb.max_force.z, pos_force_p.load_fz, robotwb.max_torque_foot.y, robotwb.max_torque_foot.z);//设置MPC参数

//        //update_x_drag(x_comp_integral);//z轴方向加速度受x轴方向速度的影响程度

//        if (vxy[0] > 0.3 || vxy[0] < -0.3) {
//            x_comp_integral += cmpc_x_drag * pz_err * dtMPC / vxy[0];
//        }
//        static float timer = 0;
//        timer += 0.05;
//        int G_flag[2] = { robotwb.Leg[0].is_ground ,robotwb.Leg[1].is_ground };
//        //if (timer > 5) {
//        //	G_flag[0] = 0;
//        //	printf("ss\n");
//        //}
//        //G_flag[0] = 0;
//        //G_flag[1] = 1;
//#if MPC_USE_REAL_TD
//        if (vmc_all.gait_mode == G_ETL) {
//            update_problem_data_floats(p, v, q, w, r, yaw, weights, trajAll, alpha, alpha_end, G_flag,gait_mpc_trot._mpc_table);
//        }
//        else//trot下采用开环talble
//            update_problem_data_floats(p, v, q, w, r, yaw, weights, trajAll, alpha, alpha_end, G_flag,mpcTable_real);
//#else
//        update_problem_data_floats(p, v, q, w, r, yaw, weights, trajAll, alpha, mpcTable);
//#endif

//        for (int leg = 0; leg < 2; leg++)
//        {
//            for (int axis = 0; axis < 5; axis++) {//fx fy fz my mz   Rleg  Lleg
//                if(fabs(get_solution(leg * 5 + axis)) <500)
//                    f_ff_n[leg][axis] = get_solution(leg * 5 + axis);//世界下的力
//            }
//        }
//#if TEST_MPC||1
//        printf("R_leg[%d]:fx=%.2f %.2f %.2f | my=%.2f %.2f\n", robotwb.Leg[0].is_ground,f_ff_n[0][0], f_ff_n[0][1], f_ff_n[0][2], f_ff_n[0][3], f_ff_n[0][4]);
//        printf("L_leg[%d]:fx=%.2f %.2f %.2f | my=%.2f %.2f\n", robotwb.Leg[1].is_ground,f_ff_n[1][0], f_ff_n[1][1], f_ff_n[1][2], f_ff_n[1][3], f_ff_n[1][4]);
//#endif

//        for (int len = 0; len < MPC_PREDIC_LEN; len++) {
//            for (int leg = 0; leg < 2; leg++)
//            {
//                for (int axis = 0; axis < 5; axis++) {
//                    f_ff_n_list[leg][len][axis] = get_solution(len * 10 + (leg * 5 + axis));
//                }
//            }
//        }
//        f_list_cnt_timer = 0;
//        f_list_cnt = 0;
//        gait_mode_reg = vmc_all.gait_mode;
//    }
//}
