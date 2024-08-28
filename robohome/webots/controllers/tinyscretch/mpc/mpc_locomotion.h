#pragma once
//#include "cppTypes.h"
#include "common_types.h"
using Eigen::Array4f;
using Eigen::Array4i;
using Eigen::Matrix;
using Eigen::Quaternionf;
#define T_MPC 0.005
#define MPC_USE_REAL_TD 1 //������ʵ�ŵ�

typedef struct {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Vec4m<float> contactEstimate;//�Ӵ�����
	Vec3m<float> position;//λ��
	Vec3m<float> vBody;//�ٶ�
	Quat<float> orientation;//��Ԫ��
	Vec3m<float> omegaBody;//���ٶ�
	RotMat<float> rBody;//��ת����
	Vec3m<float> rpy;//ŷ����

	Vec3m<float> omegaWorld;//����������ٶ�
	Vec3m<float> vWorld;//�ٶ�
	Vec3m<float> aBody, aWorld;//���ٶȣ����������¼��ٶ�
}StateEstimate;


typedef struct  {
	// Instantaneous desired state command ˲ʱ����״ָ̬��
	Vec12<float> stateDes;

	Vec12<float> pre_stateDes;

	// Desired future state trajectory (for up to 10 timestep MPC) ������δ��״̬�켣(���10��ʱ�䲽��MPC)
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
	//�����ֶ��㣩
	Array4i _offsets; // offset in mpc segments ��λ��
	Array4i _durations; // duration of step in mpc segments ֧�ų���ʱ��
	//���ٷֱ���
	Array4f _offsetsFloat; // offsets in phase (0 to 1)��λ��
	Array4f _durationsFloat; // durations in phase (0 to 1)֧�ų���ʱ��

	int _stance;//֧��ʱ�䣬���ֶ���
	int _swing;//�ڶ�ʱ�� �ֶ���
	float _stance_t;//֧��ʱ�䣬���ֶ���
	float _swing_t;//�ڶ�ʱ�� �ֶ���
	int _iteration;//��̬Ƭ�μ���
	int _nIterations;//��̬Ƭ����
	float _phase;//��ǰ��λ
	float _duty_cycle;
	float dtMPC;

	//�����ֶ��㣩
	int _offsets_d[4]; // offset in mpc segments ��λ��
	int _durations_d[4]; // duration of step in mpc segments ֧�ų���ʱ��
	//���ٷֱ���
	float _offsetsFloat_d[4]; // offsets in phase (0 to 1)��λ��
	float _durationsFloat_d[4]; // durations in phase (0 to 1)֧�ų���ʱ��

	int _stance_d[4];//֧��ʱ�䣬���ֶ���
	int _swing_d[4];//�ڶ�ʱ�� �ֶ���
	float _stance_t_d[4];//֧��ʱ�䣬���ֶ���
	float _swing_t_d[4];//�ڶ�ʱ�� �ֶ���
	int _iteration_d[4];//��̬Ƭ�μ���
	int _nIterations_d[4];//��̬Ƭ����
	float _phase_d[4];//��ǰ��λ

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