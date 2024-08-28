#pragma once
//#include "cppTypes.h"
#include "common_types.h"
using Eigen::Array4f;
using Eigen::Array4i;
using Eigen::Matrix;
using Eigen::Quaternionf;
#define T_MPC 0.005
#define MPC_USE_REAL_TD 1 //采用真实着地

typedef struct {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Vec4m<float> contactEstimate;//接触估计
	Vec3m<float> position;//位置
	Vec3m<float> vBody;//速度
	Quat<float> orientation;//四元数
	Vec3m<float> omegaBody;//角速度
	RotMat<float> rBody;//旋转矩阵
	Vec3m<float> rpy;//欧拉角

	Vec3m<float> omegaWorld;//世界坐标角速度
	Vec3m<float> vWorld;//速度
	Vec3m<float> aBody, aWorld;//加速度，世界坐标下加速度
}StateEstimate;


typedef struct  {
	// Instantaneous desired state command 瞬时期望状态指令
	Vec12<float> stateDes;

	Vec12<float> pre_stateDes;

	// Desired future state trajectory (for up to 10 timestep MPC) 期望的未来状态轨迹(最多10个时间步长MPC)
	Eigen::Matrix<float, 12, 30> stateTrajDes;
}DesiredStateData;

typedef struct {
 
}ControlFSMData1;

extern  ControlFSMData1 mpc_data;
extern  StateEstimate seResult;
extern  DesiredStateData stateCommand;
typedef struct {
	int* _mpc_table;
	int* _mpc_table_o;
	//（按分段算）
	Array4i _offsets; // offset in mpc segments 相位差
	Array4i _durations; // duration of step in mpc segments 支撑持续时间
	//按百分比算
	Array4f _offsetsFloat; // offsets in phase (0 to 1)相位差
	Array4f _durationsFloat; // durations in phase (0 to 1)支撑持续时间

	int _stance;//支撑时间，按分段算
	int _swing;//摆动时间 分段算
	float _stance_t;//支撑时间，按分段算
	float _swing_t;//摆动时间 分段算
	int _iteration;//步态片段计数
	int _nIterations;//步态片段数
	float _phase;//当前相位
	float _duty_cycle;
	float dtMPC;

	//（按分段算）
	int _offsets_d[4]; // offset in mpc segments 相位差
	int _durations_d[4]; // duration of step in mpc segments 支撑持续时间
	//按百分比算
	float _offsetsFloat_d[4]; // offsets in phase (0 to 1)相位差
	float _durationsFloat_d[4]; // durations in phase (0 to 1)支撑持续时间

	int _stance_d[4];//支撑时间，按分段算
	int _swing_d[4];//摆动时间 分段算
	float _stance_t_d[4];//支撑时间，按分段算
	float _swing_t_d[4];//摆动时间 分段算
	int _iteration_d[4];//步态片段计数
	int _nIterations_d[4];//步态片段数
	float _phase_d[4];//当前相位

	//Array4i _periods;
	//Array4f _phase;

}GAIT_MPC;

extern GAIT_MPC gait_mpc_trot;

void MPC_gait_init(int horizonLength);
void MPC_gait_init_fast(int horizonLength);
void MPC_gait_update(int horizonLength);
Vec4m<float>  getSwingState(GAIT_MPC *gait_mpc);
Vec4m<float>  getContactState(GAIT_MPC *gait_mpc);
int*  getMpcTable_Gait(GAIT_MPC *gait_mpc);
void OffsetDurationGait(GAIT_MPC *gait_mpc, int nSegment, Vec4m<int> offsets, Vec4m<int> durations);
void OffsetDurationGait_Fast(GAIT_MPC *gait_mpc, int nSegment, Vec4m<int> offsets, Vec4m<int> durations);
void setIterationsMPC(GAIT_MPC *gait_mpc, int iterationsPerMPC, int currentIteration);
//div
void OffsetDurationGait_Fast_div(GAIT_MPC *gait_mpc, int nSegment, int offsets, int durations, char sel);
void  setIterations_d(GAIT_MPC *gait_mpc, int iterationsPerMPC, int currentIteration, char sel);
float getCurrentSwingTime_d(GAIT_MPC *gait_mpc, float dtMPC, int leg);
float getCurrentStanceTime_d(GAIT_MPC *gait_mpc, float dtMPC, int leg);
float  getContactState_d(GAIT_MPC *gait_mpc, char leg);
float  getSwingState_d(GAIT_MPC *gait_mpc, char leg);
float  Gait_switch_init(GAIT_MPC *gait_mpc, int iterationsPerMPC, int currentIteration, float dtMPC, char sel);