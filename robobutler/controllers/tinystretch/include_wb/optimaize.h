#pragma once

int QP_Dis_Force(float min_f, float max_f);
void calc_lb_ub_qpOASES();
void calc_lbA_ubA_qpOASES();
void  QProblemObj_qpOASES_INIT(void);
void update_A_control();
void calc_H_qpOASES();
void calc_A_qpOASES();
void calc_g_qpOASES();
void solveQP_nonThreaded();
void set_b_control(double Fexp[3], double Texp[3]);
void SetContactData(char G[4],double min_f,double max_f);
void update_end_pos(double P1[3], double P2[3], double P3[3], double P4[3]);
void print_QPData();

void ConvexMPCLocomotion_run(float dt);
void GaitLocomotion_run(float dt);
