#include "convexMPC_interface.h"
#include "common_types.h"
#include "SolverMPC.h"
#include <Eigen/Dense>
//#include <pthread.h>
#include <stdio.h>
#include <string.h>

problem_setup problem_configuration;
u8 gait_data[K_MAX_GAIT_SEGMENTS];
//pthread_mutex_t problem_cfg_mt;
//pthread_mutex_t update_mt;
update_data_t update;
//pthread_t solve_thread;

u8 first_run = 1;

void initialize_mpc()
{
  //printf("Initializing MPC!\n");
  /*if(pthread_mutex_init(&problem_cfg_mt,NULL)!=0)
    printf("[MPC ERROR] Failed to initialize problem configuration mutex.\n");

  if(pthread_mutex_init(&update_mt,NULL)!=0)
    printf("[MPC ERROR] Failed to initialize update data mutex.\n");*/

#ifdef K_DEBUG
  printf("[MPC] Debugging enabled.\n");
    printf("[MPC] Size of problem setup struct: %ld bytes.\n", sizeof(problem_setup));
    printf("      Size of problem update struct: %ld bytes.\n",sizeof(update_data_t));
    printf("      Size of MATLAB floating point type: %ld bytes.\n",sizeof(mfp));
    printf("      Size of flt: %ld bytes.\n",sizeof(flt));
#else
  //printf("[MPC] Debugging disabled.\n");
#endif
}

void setup_problem(double dt, int horizon, double mu, double f_max,double f_min,float my_max,float mz_max)
{
  //mu = 0.6;
  if(first_run)
  {
    first_run = false;
    initialize_mpc();
  }

  //pthread_mutex_lock(&problem_cfg_mt);

  problem_configuration.horizon = horizon;
  problem_configuration.f_max = f_max;
  problem_configuration.f_min = f_min;
  problem_configuration.my_max = my_max;
  problem_configuration.mz_max = mz_max;
  problem_configuration.mu = mu;
  problem_configuration.dt = dt;

  //pthread_mutex_unlock(&problem_cfg_mt);
  resize_qp_mats(horizon);
}

//inline to motivate gcc to unroll the loop in here.
inline void mfp_to_flt(flt* dst, mfp* src, s32 n_items)
{
  for(s32 i = 0; i < n_items; i++)
    *dst++ = *src++;
}

inline void mint_to_u8(u8* dst, mint* src, s32 n_items)
{
  for(s32 i = 0; i < n_items; i++)
    *dst++ = *src++;
}

int has_solved = 0;

//void *call_solve(void* ptr)
//{
//  solve_mpc(&update, &problem_configuration);
//}
//safely copies problem data and starts the solver
void update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait)
{
  mfp_to_flt(update.p,p,3);
  mfp_to_flt(update.v,v,3);
  mfp_to_flt(update.q,q,4);
  mfp_to_flt(update.w,w,3);
  mfp_to_flt(update.r,r,6);//修改了腿数量
  update.yaw = yaw;
  mfp_to_flt(update.weights,weights,12);
  //this is safe, the solver isn't running, and update_problem_data and setup_problem
  //are called from the same thread
  mfp_to_flt(update.traj,state_trajectory,12*problem_configuration.horizon);
  update.alpha = alpha;
  mint_to_u8(update.gait,gait,2*problem_configuration.horizon);

  solve_mpc(&update, &problem_configuration);
  has_solved = 1;
}

void update_problem_data_floats(float* p, float* v, float* q, float* w,
                                float* r, float yaw, float* weights,
                                float* state_trajectory, float alpha, float alpha_end,int Gflag[2],int* gait)
{
  update.alpha = alpha;
  update.alpha_end = alpha_end;
  update.yaw = yaw;
  update.Gflag[0] = Gflag[0];
  update.Gflag[1] = Gflag[1];
  //float p[3];
  //float v[3];
  //float q[4];
  //float w[3];
  //float r[6];//修改了腿数量
  //float yaw;
  //float weights[12];
  //float traj[12 * K_MAX_GAIT_SEGMENTS];
  //float alpha;
  //unsigned char gait[K_MAX_GAIT_SEGMENTS];
  //float x_drag;////z轴方向加速度受x轴方向速度的影响程度
  mint_to_u8(update.gait,gait,2*problem_configuration.horizon);
  memcpy((void*)update.p,(void*)p,sizeof(float)*3);
  memcpy((void*)update.v,(void*)v,sizeof(float)*3);
  memcpy((void*)update.q,(void*)q,sizeof(float)*4);
  memcpy((void*)update.w,(void*)w,sizeof(float)*3);
  memcpy((void*)update.r,(void*)r,sizeof(float)*6);//修改了腿数量
  memcpy((void*)update.weights,(void*)weights,sizeof(float)*12);//权重 RPY XYZ DRPY DXYZ
  memcpy((void*)update.traj,(void*)state_trajectory, sizeof(float) * 12 * problem_configuration.horizon);
#if 0
  printf("p=%.2f %.2f %.2f\n", update.p[0], update.p[1], update.p[2]);
  printf("v=%.2f %.2f %.2f\n", update.v[0], update.v[1], update.v[2]);
  printf("q=%.2f %.2f %.2f %.2f\n", update.q[0], update.q[1], update.q[2], update.q[3]);
  printf("w=%.2f %.2f %.2f\n", update.w[0], update.w[1], update.w[2]);
  printf("r=%.2f %.2f | %.2f %.2f  | %.2f %.2f \n", update.r[0], update.r[1], update.r[2], update.r[3], update.r[4], update.r[5]);
  for (int i = 0; i < problem_configuration.horizon; i++)
  {
	  printf("Traj[%d]: ",i);
	  for (int j = 0; j < 12; j++)
		  printf("%.2f ", update.traj[j+12*i]);
	  printf("\n");
  }
#endif
  solve_mpc(&update, &problem_configuration);
  has_solved = 1;
}

void update_x_drag(float x_drag) {
	update.x_drag = x_drag;
}

double get_solution(int index)
{
  if(!has_solved) return 0.f;
  fpt* qs = get_q_soln();
  return qs[index];
}
