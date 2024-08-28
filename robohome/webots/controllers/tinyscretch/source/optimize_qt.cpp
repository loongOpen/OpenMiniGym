#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <iostream>
#include "locomotion_header.h"
#include "gait_math.h"
#if !RUN_WEBOTS
#include "qp_fdis.h"
#define TIME_STEP 0.002
#else
#include "optimaize.h"
#define TIME_STEP 0.003
#endif

//Webots初始化故障  VS优化关闭正常
using namespace Eigen;
using namespace std;
using namespace qpOASES;

#define FIX_NWSR 0
#define USE_LAST_FORCE_SMOOTH 0  //使能上次力平滑
#define FY_DISABLE 0					 //屏蔽侧向力
/* Fixed-Size qpOASES data */

static const int NUM_VARIABLES_QP = 12;//4*3
static const int NUM_CONSTRAINTS_QP = 20;//等式约束矩阵
static const int NUM_CONTACT_POINTS = 4;
static const int NUM_VARIABLES_PER_FOOT = 3;//Fx y z
static const int NUM_CONSTRAINTS_PER_FOOT = 5;//ux uy uz min max

// static const double PI_CONST = 3.1415;
static const double NEGATIVE_NUMBER = -1000000.0;
static const double POSITIVE_NUMBER = 1000000.0;

QProblem QProblemObj_qpOASES(NUM_VARIABLES_QP, NUM_CONSTRAINTS_QP);

int_t nWSR_qpOASES = 100;//求解迭代次数
int_t nWSR_fixed = 100;

real_t cpu_time;
real_t cpu_time_fixed;

int_t qp_exit_flag;

int nWSR_initial;
double cpu_time_initial;

double xOpt_local[12];
double qp_not_init;

Bounds guessedBounds;
Constraints guessedConstraints;

real_t H_qpOASES[NUM_VARIABLES_QP * NUM_VARIABLES_QP];
real_t A_qpOASES[NUM_CONSTRAINTS_QP * NUM_VARIABLES_QP];
real_t g_qpOASES[NUM_VARIABLES_QP];
real_t lb_qpOASES[NUM_VARIABLES_QP];
real_t ub_qpOASES[NUM_VARIABLES_QP];
real_t lbA_qpOASES[NUM_CONSTRAINTS_QP];
real_t ubA_qpOASES[NUM_CONSTRAINTS_QP];
real_t xOpt_qpOASES[NUM_VARIABLES_QP];
real_t yOpt_qpOASES[NUM_VARIABLES_QP + NUM_CONSTRAINTS_QP];

real_t xOpt_initialGuess[NUM_VARIABLES_QP];

/* Eigen Variables that Match qpOASES variables */
Eigen::MatrixXd H_eigen;
Eigen::MatrixXd A_eigen;
Eigen::MatrixXd g_eigen;
Eigen::VectorXd xOpt_eigen;
Eigen::VectorXd yOpt_eigen;

/* Robot control variables used to construct QP matrices, see (5) and (6) of
 * [R1] */
Eigen::MatrixXd A_control;
Eigen::MatrixXd S_control;
Eigen::MatrixXd W_control;
Eigen::MatrixXd C_control;
Eigen::VectorXd b_control;
Eigen::VectorXd b_control_Opt;

Eigen::VectorXd C_times_f_control;

Eigen::VectorXd contact_state;

Eigen::VectorXd direction_normal_flatGround;
Eigen::VectorXd direction_tangential_flatGround;

/* Temporary, Internal Matrices */
Eigen::MatrixXd omegaHat;
Eigen::MatrixXd tempSkewMatrix3;
Eigen::VectorXd tempVector3;

/* Interal QP management data */
bool QPFinished;

Eigen::VectorXd xOptPrev;
Eigen::VectorXd yOptPrev;
Eigen::MatrixXd R_yaw_act;
double alpha_control;
double use_hard_constraint_pitch;
/* Model and World parameters and force limits */
double mass;
double inertia;
Eigen::MatrixXd Ig;

double mu_friction;

Eigen::VectorXd minNormalForces_feet;
Eigen::VectorXd maxNormalForces_feet;

Eigen::MatrixXd p_feet;

void copy_Eigen_to_real_t(real_t* target,
    Eigen::MatrixXd& source, int nRows,
    int nCols) {
    int count = 0;

    // Strange Behavior: Eigen matrix matrix(count) is stored by columns (not
    // rows)
    for (int i = 0; i < nRows; i++) {
        for (int j = 0; j < nCols; j++) {
            target[count] = source(i, j);
            count++;
        }
    }
}

void copy_Eigen_to_double(double* target,
    Eigen::VectorXd& source,
    int length) {
    for (int i = 0; i < length; i++) {
        target[i] = source(i);
    }
}

void copy_Array_to_Eigen(Eigen::VectorXd& target,
    double* source, int len,
    int startIndex) {
    for (int i = 0; i < len; i++) {
        target(i) = source[i + startIndex];
    }
}

void copy_Array_to_Eigen(Eigen::MatrixXd& target,
    double* source, int len,
    int startIndex) {
    for (int i = 0; i < len; i++) {
        target(i) = source[i + startIndex];
    }
}

void copy_real_t_to_Eigen(Eigen::VectorXd& target,
    real_t* source, int len) {
    for (int i = 0; i < len; i++) {
        target(i) = source[i];
    }
}

void matrixLogRot(const Eigen::MatrixXd& Rm, Eigen::VectorXd& omega) {
    // theta = acos( (Trace(R) - 1)/2 )
    double theta;
    double tmp = (Rm(0, 0) + Rm(1, 1) + Rm(2, 2) - 1) / 2;
    if (tmp >= 1.) {
        theta = 0;
    }
    else if (tmp <= -1.) {
        theta = 3.1415926;
    }
    else {
        theta = acos(tmp);
    }

    // Matrix3F omegaHat = (R-R.transpose())/(2 * sin(theta));
    // crossExtract(omegaHat,omega);
    omega << Rm(2, 1) - Rm(1, 2), Rm(0, 2) - Rm(2, 0), Rm(1, 0) - Rm(0, 1);
    if (theta > 10e-5) {
        omega *= theta / (2 * sin(theta));
    }
    else {
        omega /= 2;
    }
}

void crossMatrix(Eigen::MatrixXd& Rm,
    const Eigen::VectorXd& omega) {
    Rm(0, 1) = -omega(2);
    Rm(0, 2) = omega(1);
    Rm(1, 0) = omega(2);
    Rm(1, 2) = -omega(0);
    Rm(2, 0) = -omega(1);
    Rm(2, 1) = omega(0);
}

void quaternion_to_rotationMatrix(Eigen::MatrixXd& Rm,
    Eigen::VectorXd& quat) {
    // wikipedia
    Rm(0, 0) = 1 - 2 * quat(2) * quat(2) - 2 * quat(3) * quat(3);
    Rm(0, 1) = 2 * quat(1) * quat(2) - 2 * quat(0) * quat(3);
    Rm(0, 2) = 2 * quat(1) * quat(3) + 2 * quat(0) * quat(2);
    Rm(1, 0) = 2 * quat(1) * quat(2) + 2 * quat(0) * quat(3);
    Rm(1, 1) = 1 - 2 * quat(1) * quat(1) - 2 * quat(3) * quat(3);
    Rm(1, 2) = 2 * quat(2) * quat(3) - 2 * quat(1) * quat(0);
    Rm(2, 0) = 2 * quat(1) * quat(3) - 2 * quat(2) * quat(0);
    Rm(2, 1) = 2 * quat(2) * quat(3) + 2 * quat(1) * quat(0);
    Rm(2, 2) = 1 - 2 * quat(1) * quat(1) - 2 * quat(2) * quat(2);
}

void rpyToR(Eigen::MatrixXd& Rm, double* rpy_in) {
    Eigen::Matrix3d Rz, Ry, Rx;

    Rz.setIdentity();
    Ry.setIdentity();
    Rx.setIdentity();

    Rz(0, 0) = cos(rpy_in[2]);
    Rz(0, 1) = -sin(rpy_in[2]);
    Rz(1, 0) = sin(rpy_in[2]);
    Rz(1, 1) = cos(rpy_in[2]);

    Ry(0, 0) = cos(rpy_in[1]);
    Ry(0, 2) = sin(rpy_in[1]);
    Ry(2, 0) = -sin(rpy_in[1]);
    Ry(2, 2) = cos(rpy_in[1]);

    Rx(1, 1) = cos(rpy_in[0]);
    Rx(1, 2) = -sin(rpy_in[0]);
    Rx(2, 1) = sin(rpy_in[0]);
    Rx(2, 2) = cos(rpy_in[0]);

    Rm = Rz * Ry * Rx;
}


int QP_Dis_Force(float min_f, float max_f)
{
    static int init = 0;
    static float yaw_control_mask = 0;
    int id_swap[4] = { 0,2,1,3 };//Moco定义顺序
    double FALL[3], TALL[3], Fp[2], PC[3], P1[3], P2[3], P3[3], P4[3];
    char G[4];
    double F1c[3], F2c[3], F3c[3], F4c[3];
    if (!init) {
        init = 1;
        use_hard_constraint_pitch = FY_DISABLE;
        QProblemObj_qpOASES_INIT();
    }
    //direction_normal_flatGround << -sind(vmc_all.ground_att_est[PITr]), 0, cosd(vmc_all.ground_att_est[PITr]);// 1;not good at stair

    FALL[Xrw] = robotwb.exp_force.x;//前
    FALL[Yrw] = -robotwb.exp_force.y;
    FALL[Zrw] = robotwb.exp_force.z;//高度
    TALL[Xrw] = -robotwb.exp_torque.x;//ROL
    TALL[Yrw] = robotwb.exp_torque.y;//PIT
    TALL[Zrw] = robotwb.exp_torque.z * -1;

    G[0] = robotwb.Leg[id_swap[0]].is_ground;
    G[1] = robotwb.Leg[id_swap[1]].is_ground;
    G[2] = robotwb.Leg[id_swap[2]].is_ground;
    G[3] = robotwb.Leg[id_swap[3]].is_ground;
#if 0
    G[0] = G[1] = G[2] = G[3] = 1;
#endif
    PC[Xrw] = 0;
    PC[Yrw] = 0;
    PC[Zrw] = 0;

    P1[Xrw] = PC[Xrw] - (robotwb.Leg[id_swap[0]].epos_n.x);
    P1[Yrw] = PC[Yrw] - (-robotwb.Leg[id_swap[0]].epos_n.y);
    P1[Zrw] = PC[Zrw] - robotwb.Leg[id_swap[0]].epos_n.z;

    P2[Xrw] = PC[Xrw] - (robotwb.Leg[id_swap[1]].epos_n.x);
    P2[Yrw] = PC[Yrw] - (-robotwb.Leg[id_swap[1]].epos_n.y);
    P2[Zrw] = PC[Zrw] - robotwb.Leg[id_swap[1]].epos_n.z;

    P3[Xrw] = PC[Xrw] - (robotwb.Leg[id_swap[2]].epos_n.x);
    P3[Yrw] = PC[Yrw] - (-robotwb.Leg[id_swap[2]].epos_n.y);
    P3[Zrw] = PC[Zrw] - robotwb.Leg[id_swap[2]].epos_n.z;

    P4[Xrw] = PC[Xrw] - (robotwb.Leg[id_swap[3]].epos_n.x);
    P4[Yrw] = PC[Yrw] - (-robotwb.Leg[id_swap[3]].epos_n.y);
    P4[Zrw] = PC[Zrw] - robotwb.Leg[id_swap[3]].epos_n.z;

    mu_friction = 0.45;//摩擦叙述


    if (G[0] + G[1] + G[2] + G[3] >= 1) {
        //printf("5\n");
        set_b_control(FALL, TALL);//设置期望虚拟扭矩
        //printf("6\n");
        SetContactData(G, min_f, max_f);//设定着地腿  计算摩擦约束
        //printf("7\n");
        update_end_pos(P1, P2, P3, P4);
        //printf("8\n");
        update_A_control();//末端落足点
        //printf("9\n");
        // Compute QP Problem data
        calc_H_qpOASES();//计算二次项矩阵
        //printf("10\n");
        calc_A_qpOASES();//计算约束矩阵
        //printf("11\n");
        calc_g_qpOASES();//计算常数矩阵
        //printf("12\n");
        cpu_time = cpu_time_fixed;
        nWSR_qpOASES = nWSR_fixed;

        //print_QPData();

        solveQP_nonThreaded();//QP求解
        //printf("13\n");
    }
    else
    {
        for (int i = 0; i < 4; i++)
            robotwb.Leg[i].tar_force_dis_n_qp.x = robotwb.Leg[i].tar_force_dis_n_qp.y = robotwb.Leg[i].tar_force_dis_n_qp.z = 0;
    }
    return 0;
}


void  QProblemObj_qpOASES_INIT(void) {//初始化

    Options options;
    options.printLevel = PL_NONE;//禁止打印
    QProblemObj_qpOASES.setOptions(options);
    QProblemObj_qpOASES.setPrintLevel(PL_NONE);
    //printf("1\n");
    H_eigen.resize(NUM_VARIABLES_QP, NUM_VARIABLES_QP);
    A_eigen.resize(NUM_CONSTRAINTS_QP, NUM_VARIABLES_QP);
    g_eigen.resize(NUM_VARIABLES_QP, 1);
    xOpt_eigen.resize(NUM_VARIABLES_QP, 1);
    yOpt_eigen.resize(NUM_VARIABLES_QP + NUM_CONSTRAINTS_QP, 1);
    //printf("2\n");
    /* Initialize to all feet on the ground */
    contact_state.resize(4, 1);//着地状态
    contact_state << 0, 0, 0, 0;

    minNormalForces_feet.resize(4, 1);
    maxNormalForces_feet.resize(4, 1);
    R_yaw_act.resize(3, 3);
    R_yaw_act.setIdentity();
    p_feet.resize(3, 4);//全局位置   腿数量   3行4列

    /* Temporary, Internal Matrices */
    omegaHat.resize(3, 3);//行row 列col
    tempSkewMatrix3.resize(3, 3);
    tempVector3.resize(3, 1);

    tempSkewMatrix3.setZero();
    tempVector3.setZero();

    A_control.resize(6, 3 * NUM_CONTACT_POINTS);
    b_control.resize(6, 1);
    b_control_Opt.resize(6, 1);
    S_control.resize(6, 6);

    W_control.resize(NUM_VARIABLES_QP, NUM_VARIABLES_QP);
    C_control.resize(NUM_CONSTRAINTS_QP, 3 * NUM_CONTACT_POINTS);

    C_times_f_control.resize(NUM_CONSTRAINTS_QP, 1);

    C_control.setZero();
    xOptPrev.setZero(12);
    yOptPrev.setZero(NUM_VARIABLES_QP + NUM_CONSTRAINTS_QP);

    for (int i = 0; i < NUM_VARIABLES_QP; i++) {
        xOpt_qpOASES[i] = 0.0;
        xOpt_initialGuess[i] = 0.0;
    }

    float initialGuess = 100;
    xOpt_initialGuess[2] = initialGuess;
    xOpt_initialGuess[5] = initialGuess;
    xOpt_initialGuess[8] = initialGuess;
    xOpt_initialGuess[11] = initialGuess;

    for (int i = 0; i < NUM_VARIABLES_QP + NUM_CONSTRAINTS_QP; i++) {
        yOpt_qpOASES[i] = 0.0;
    }


    //设定QP权重
    S_control.setIdentity();//单位阵 越大权重越小
    S_control(0, 0) = 1;
    S_control(1, 1) = 0.8;
    S_control(2, 2) = 1;

    S_control(3, 3) = 12;//PIT  影响航向和X
    S_control(4, 4) = 8;//ROL	影响高度
    S_control(5, 5) = 20;

    alpha_control = 0.1;
    //printf("41\n");
    W_control.setIdentity();//单位阵
//	W_control *= 2;
#if 0
    float weight = 1.0e-05;
    for (int i = 0; i < 4; i++)
    {
        W_control(0 + i * 3, 0 + i * 3) = weight;// 1.0e-03;
        W_control(1 + i * 3, 1 + i * 3) = weight;// 1.0e-03;
        W_control(2 + i * 3, 2 + i * 3) = weight;//1.0e-03;
    }
#endif

    direction_normal_flatGround.resize(3, 1);
    direction_tangential_flatGround.resize(3, 1);
#if 1
    direction_normal_flatGround << 0, 0, 1;
#else
    direction_normal_flatGround << -sind(vmc_all.ground_att_est[PITr]), 0, cosd(vmc_all.ground_att_est[PITr]);// 1;
#endif
    direction_tangential_flatGround << 0.7071, 0.7071, 0;//摩擦圆锥角度

    mu_friction = 0.55;//摩擦叙述

    cpu_time = TIME_STEP;
    cpu_time_fixed = TIME_STEP;
    qp_exit_flag = -1.0;

    qp_not_init = 1.0;

    minNormalForces_feet << 3, 3, 3, 3;
    maxNormalForces_feet << 250, 250, 250, 250;
    //printf("46\n");
}

void update_end_pos(double P1[3], double P2[3], double P3[3], double P4[3])
{
    /*MIT 坐标系
机体
/\X
|
|
O---------->-Y
// %1FL   0FR
// %
// %3BL   2BR
*/
    p_feet(0, 0) = P1[Xrw];
    p_feet(1, 0) = P1[Yrw];
    p_feet(2, 0) = P1[Zrw];

    p_feet(0, 1) = P2[Xrw];
    p_feet(1, 1) = P2[Yrw];
    p_feet(2, 1) = P2[Zrw];

    p_feet(0, 2) = P3[Xrw];
    p_feet(1, 2) = P3[Yrw];
    p_feet(2, 2) = P3[Zrw];

    p_feet(0, 3) = P4[Xrw];
    p_feet(1, 3) = P4[Yrw];
    p_feet(2, 3) = P4[Zrw];

    //std::cout << "p_feet = " << p_feet << "\n";
}

void set_b_control(double Fexp[3], double Texp[3])//设置虚拟力 力矩
{
    // See RHS of Equation (5), [R1]
    b_control << Fexp[0], Fexp[1], Fexp[2], Texp[0], Texp[1], Texp[2];  //ROLL PITCH
    //std::cout << "b_control" << b_control << endl;
    //printf("%f %f %f\n", Fexp[1], -Fexp[0], Fexp[2]);
}

void SetContactData(char G[4], double min_f, double max_f) {//设定着地状态

    double contact_state_in[4] = { (double)G[0],(double)G[1] ,(double)G[2] ,(double)G[3] };
    double min_forces_in[4] = { min_f ,min_f ,min_f ,min_f };
    double max_forces_in[4] = { max_f ,max_f ,max_f ,max_f };
    // Unpack inputs
    copy_Array_to_Eigen(contact_state, contact_state_in, 4, 0);
    copy_Array_to_Eigen(minNormalForces_feet, min_forces_in, 4, 0);
    copy_Array_to_Eigen(maxNormalForces_feet, max_forces_in, 4, 0);

    calc_lb_ub_qpOASES();
    calc_lbA_ubA_qpOASES();
}

void update_A_control() {
    // Update the A matrix in the controller notation A*f = b
    for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
        //R_yaw_act(0, 1) = 0; //行row 列col
        //R_yaw_act(1, 1) = 0;
        //R_yaw_act(2, 1) = 0;

        A_control.block<3, 3>(0, 3 * i) << R_yaw_act.transpose();

        tempVector3 << contact_state(i) * p_feet.col(i);
        crossMatrix(tempSkewMatrix3, tempVector3);
#if FY_DISABLE&&0
        tempSkewMatrix3(0, 1) = 0;
        tempSkewMatrix3(1, 1) = 0;
        tempSkewMatrix3(2, 1) = 0;
#endif
        A_control.block<3, 3>(3, 3 * i) << R_yaw_act.transpose() * tempSkewMatrix3;
    }
    //printf("%d %d %d %d\n", contact_state(0), contact_state(1), contact_state(2), contact_state(3));
}

void calc_H_qpOASES() {//计算Hessib矩阵
    // Use the A matrix to compute the QP cost matrix H
    H_eigen = 2 * (A_control.transpose() * S_control * A_control +
        (alpha_control + 1e-3) * W_control);

    // Copy to real_t array (qpOASES data type)
    copy_Eigen_to_real_t(H_qpOASES, H_eigen, NUM_VARIABLES_QP, NUM_VARIABLES_QP);
}

void calc_A_qpOASES() {//计算不等式约束A摩擦圆锥
    Eigen::Vector3d t1x;
    t1x << 1, 0, 0;
    Eigen::Vector3d t2y;
    t2y << 0, 1, 0;

    for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
        C_control.block<1, 3>(5 * i + 0, 3 * i)
            << -mu_friction * direction_normal_flatGround.transpose() +
            t1x.transpose();
        C_control.block<1, 3>(5 * i + 1, 3 * i)
            << -mu_friction * direction_normal_flatGround.transpose() +
            t2y.transpose();
        C_control.block<1, 3>(5 * i + 2, 3 * i)
            << mu_friction * direction_normal_flatGround.transpose() +
            t2y.transpose();
        C_control.block<1, 3>(5 * i + 3, 3 * i)
            << mu_friction * direction_normal_flatGround.transpose() +
            t1x.transpose();
        C_control.block<1, 3>(5 * i + 4, 3 * i)
            << direction_normal_flatGround.transpose();
    }

    /*if (use_hard_constraint_pitch == 1) {//fy=0
        C_control.row(NUM_CONSTRAINTS_QP - 1) =
            A_control.row(5 - 1);  // add hard constraint on pitch control
    }

    else {*/
    C_control.row(NUM_CONSTRAINTS_QP - 1) = 0 * A_control.row(5 - 1);
    //}
    //C_control.row(NUM_CONSTRAINTS_QP - 1) =A_control.row(5 );  // add hard constraint on pitch control
    copy_Eigen_to_real_t(A_qpOASES, C_control, NUM_CONSTRAINTS_QP,
        NUM_VARIABLES_QP);
}

void calc_g_qpOASES() {//计算g期望数据矩阵
    g_eigen = -2 * A_control.transpose() * S_control * b_control;
#if USE_LAST_FORCE_SMOOTH
    //g_eigen += -2 * xOptPrev.transpose() * alpha_control;//力平滑
    g_eigen += -2 * xOptPrev * alpha_control;//力平滑
#endif
    // Copy to real_t array (qpOASES data type)
    copy_Eigen_to_real_t(g_qpOASES, g_eigen, NUM_VARIABLES_QP, 1);
}

void calc_lb_ub_qpOASES() {//设置各足力上下限矩阵
    for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
        for (int j = 0; j < NUM_VARIABLES_PER_FOOT; j++) {
            lb_qpOASES[NUM_VARIABLES_PER_FOOT * i + j] =
                contact_state(i) * NEGATIVE_NUMBER;
            ub_qpOASES[NUM_VARIABLES_PER_FOOT * i + j] =
                contact_state(i) * POSITIVE_NUMBER;
        }
    }

    // add constraint on f_y=0 for jumping
    if (FY_DISABLE && 1) {//use_hard_constraint_pitch == 1) {
        for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
            lb_qpOASES[NUM_VARIABLES_PER_FOOT * i + 1] = 0;
            ub_qpOASES[NUM_VARIABLES_PER_FOOT * i + 1] = 0;
        }
    }

    if (FY_DISABLE && 0) {//f_x=0 屏蔽前向力8自由度
        for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
            lb_qpOASES[NUM_VARIABLES_PER_FOOT * i + 0] = 0;
            ub_qpOASES[NUM_VARIABLES_PER_FOOT * i + 0] = 0;
        }
    }

}

void calc_lbA_ubA_qpOASES() {//计算上下限
    for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
        lbA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i] =
            contact_state(i) * NEGATIVE_NUMBER;
        lbA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 1] =
            contact_state(i) * NEGATIVE_NUMBER;
        lbA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 2] = 0;
        lbA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 3] = 0;
        lbA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 4] =
            contact_state(i) * minNormalForces_feet(i);

        ubA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i] = 0;
        ubA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 1] = 0;
        ubA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 2] =
            contact_state(i) * POSITIVE_NUMBER;
        ubA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 3] =
            contact_state(i) * POSITIVE_NUMBER;
        ubA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 4] =
            contact_state(i) * maxNormalForces_feet(i);
    }

    // add hard constraint on pitch control
    /*if (use_hard_constraint_pitch == 1) {
        lbA_qpOASES[NUM_CONSTRAINTS_QP - 1] = b_control(5 - 1);//b5为Pitch轴
        ubA_qpOASES[NUM_CONSTRAINTS_QP - 1] = b_control(5 - 1);
    }

    else if (use_hard_constraint_pitch > 1) {
        lbA_qpOASES[NUM_CONSTRAINTS_QP - 1] = POSITIVE_NUMBER;
        ubA_qpOASES[NUM_CONSTRAINTS_QP - 1] = NEGATIVE_NUMBER;
    }

    else {*/
    lbA_qpOASES[NUM_CONSTRAINTS_QP - 1] = NEGATIVE_NUMBER;
    ubA_qpOASES[NUM_CONSTRAINTS_QP - 1] = POSITIVE_NUMBER;
    //}
}


void solveQP_nonThreaded() {//(double* xOpt) {
    // &cpu_time
    if (qp_not_init == 1.0) {
        qp_exit_flag = QProblemObj_qpOASES.init(
            H_qpOASES, g_qpOASES, A_qpOASES, lb_qpOASES, ub_qpOASES, lbA_qpOASES,
            ubA_qpOASES, nWSR_qpOASES, &cpu_time, xOpt_initialGuess);
        qp_not_init = 0.0;

        nWSR_initial = nWSR_qpOASES;
        cpu_time_initial = cpu_time;
    }
    else {
#if FIX_NWSR
        nWSR_qpOASES = 100;
        cpu_time = TIME_STEP;
#endif
        qp_exit_flag = QProblemObj_qpOASES.init(
            H_qpOASES, g_qpOASES, A_qpOASES, lb_qpOASES, ub_qpOASES, lbA_qpOASES,
            ubA_qpOASES, nWSR_qpOASES, &cpu_time, xOpt_qpOASES, yOpt_qpOASES,
            &guessedBounds, &guessedConstraints);
    }

    QProblemObj_qpOASES.getPrimalSolution(xOpt_qpOASES);
    QProblemObj_qpOASES.getDualSolution(yOpt_qpOASES);

    QProblemObj_qpOASES.getBounds(guessedBounds);
    QProblemObj_qpOASES.getConstraints(guessedConstraints);

    // std::cout << "cpu_time_initial = " << cpu_time_initial << "\n";
    // std::cout << "qp exit flag = " << qp_exit_flag << "\n";
    // std::cout << "nWSR_initial = " << nWSR_initial << "\n";
    // std::cout << "max NWSR = " << RET_MAX_NWSR_REACHED << "\n"; // 64
    // std::cout << "qp failed = " << RET_INIT_FAILED << "\n"; // 33
    copy_real_t_to_Eigen(xOpt_eigen, xOpt_qpOASES, 12);
    // copy_real_t_to_Eigen(yOptPrev, yOpt_qpOASES,
    // NUM_VARIABLES_QP+NUM_CONSTRAINTS_QP);

    b_control_Opt = A_control * xOpt_eigen;//反计算当前优化的伺服结果

    //std::cout << "b_control_Exp = " << b_control << "\n";
    //std::cout << "b_control_Opt = " << b_control_Opt << "\n";
    static float timer1 = 0;
#if 0
    timer1 += 0.005;
    if (timer1 > 0.02 || 1) {
        timer1 = 0;
        printf("b_control_Exp= Fx=%f Fz=%f Trol=%f Tpit=%f %f\n", b_control[0], b_control[2], b_control[3], b_control[4], b_control[5]);
        printf("b_control_Opt= Fx=%f Fz=%f Trol=%f Tpit=%f %f\n", b_control_Opt[0], b_control_Opt[2], b_control_Opt[3], b_control_Opt[4], b_control_Opt[5]);
        printf("leg1 fx=%f fy=%f fz=%f\n", xOpt_eigen[0], xOpt_eigen[1], xOpt_eigen[2]);
        printf("leg2 fx=%f fy=%f fz=%f\n", xOpt_eigen[3], xOpt_eigen[4], xOpt_eigen[5]);
        printf("leg3 fx=%f fy=%f fz=%f\n", xOpt_eigen[6], xOpt_eigen[7], xOpt_eigen[8]);
        printf("leg4 fx=%f fy=%f fz=%f\n", xOpt_eigen[9], xOpt_eigen[10], xOpt_eigen[11]);
        printf("%f %f %f %f\n", contact_state(0), contact_state(1), contact_state(2), contact_state(3));
        //std::cout << "xOpt_qpOASES = " << xOpt_eigen << "\n";
    }
#endif
    double F1c[3], F2c[3], F3c[3], F4c[3];

    //MIT     1         0
    //        3         2
    //MOCO    2         0
    //        3         1
    int id_swap[4] = { 0,2,1,3 };
    F1c[Xrw] = xOpt_eigen(0);
    F1c[Yrw] = xOpt_eigen(1);
    F1c[Zrw] = xOpt_eigen(2);

    F2c[Xrw] = xOpt_eigen(3);
    F2c[Yrw] = xOpt_eigen(4);
    F2c[Zrw] = xOpt_eigen(5);

    F3c[Xrw] = xOpt_eigen(6);
    F3c[Yrw] = xOpt_eigen(7);
    F3c[Zrw] = xOpt_eigen(8);

    F4c[Xrw] = xOpt_eigen(9);
    F4c[Yrw] = xOpt_eigen(10);
    F4c[Zrw] = xOpt_eigen(11);

    robotwb.Leg[id_swap[0]].tar_force_dis_n_qp.x = F1c[Xrw];// *G[0];
    robotwb.Leg[id_swap[0]].tar_force_dis_n_qp.y = -F1c[Yrw];// * G[0];
    robotwb.Leg[id_swap[0]].tar_force_dis_n_qp.z = F1c[Zrw];// * G[0];

    robotwb.Leg[id_swap[1]].tar_force_dis_n_qp.x = F2c[Xrw];// * G[1];
    robotwb.Leg[id_swap[1]].tar_force_dis_n_qp.y = -F2c[Yrw];// * G[1];
    robotwb.Leg[id_swap[1]].tar_force_dis_n_qp.z = F2c[Zrw];//* G[1];

    robotwb.Leg[id_swap[2]].tar_force_dis_n_qp.x = F3c[Xrw];// * G[2];
    robotwb.Leg[id_swap[2]].tar_force_dis_n_qp.y = -F3c[Yrw];// * G[2];
    robotwb.Leg[id_swap[2]].tar_force_dis_n_qp.z = F3c[Zrw];// * G[2];

    robotwb.Leg[id_swap[3]].tar_force_dis_n_qp.x = F4c[Xrw];// * G[3];
    robotwb.Leg[id_swap[3]].tar_force_dis_n_qp.y = -F4c[Yrw];// * G[3];
    robotwb.Leg[id_swap[3]].tar_force_dis_n_qp.z = F4c[Zrw];//* G[3];

    xOptPrev = xOpt_eigen;//缓存当前优化结果

    QProblemObj_qpOASES.reset();
}

void print_real_t(real_t* matrix, int nRows, int nCols) {
    int count = 0;
    for (int i = 0; i < nRows; i++) {
        for (int j = 0; j < nCols; j++) {
            std::cout << matrix[count] << "\t";
            count++;
        }
        std::cout << "\n";
    }
}

void print_QPData() {
    std::cout << "\n\n";
    std::cout << "\n\nH = ";
    print_real_t(H_qpOASES, NUM_VARIABLES_QP, NUM_VARIABLES_QP);

    std::cout << "\n\nAc = " << A_control << endl;

    std::cout << "\n\nA = ";
    print_real_t(A_qpOASES, NUM_CONSTRAINTS_QP, NUM_VARIABLES_QP);
    std::cout << "\n\ng = ";
    print_real_t(g_qpOASES, NUM_VARIABLES_QP, 1);
    std::cout << "\n\nlb = ";
    print_real_t(lb_qpOASES, NUM_VARIABLES_QP, 1);
    std::cout << "\n\nub = ";
    print_real_t(ub_qpOASES, NUM_VARIABLES_QP, 1);
    std::cout << "\n\nlbA = ";
    print_real_t(lbA_qpOASES, NUM_CONSTRAINTS_QP, 1);
    std::cout << "\n\nubA = ";
    print_real_t(ubA_qpOASES, NUM_CONSTRAINTS_QP, 1);
}

