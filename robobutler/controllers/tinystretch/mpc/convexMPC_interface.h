#ifndef _convexmpc_interface
#define _convexmpc_interface
#define K_MAX_GAIT_SEGMENTS 36

//#include "common_types.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

struct problem_setup
{
  float dt;
  float mu;
  float f_min;
  float f_max;
  float my_max;
  float mz_max;
  int horizon;
};

struct update_data_t
{
  float p[3];
  float v[3];
  float q[4];
  float w[3];
  float r[6];//修改了腿数量
  int Gflag[2];
  float yaw;
  float weights[12];
  float traj[12*K_MAX_GAIT_SEGMENTS];
  float alpha;
  float alpha_end;
  unsigned char gait[K_MAX_GAIT_SEGMENTS];
  float x_drag;////z轴方向加速度受x轴方向速度的影响程度
};

EXTERNC void setup_problem(double dt, int horizon, double mu, double f_max,double f_min, float my_max, float mz_max);
EXTERNC void update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait);
EXTERNC double get_solution(int index);
EXTERNC void update_problem_data_floats(float* p, float* v, float* q, float* w,
                                        float* r, float yaw, float* weights,
                                        float* state_trajectory, float alpha, float alpha_end, int Gflag[2], int* gait);
void update_x_drag(float x_drag);
#endif