/*! @file PositionVelocityEstimator.h
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#ifndef PROJECT_POSITIONVELOCITYESTIMATOR_H
#define PROJECT_POSITIONVELOCITYESTIMATOR_H
//#include "locomotion_header.h"
//#include "gait_math.h"
#include "cppTypes.h"
#include <Eigen/Dense>
/*!
 * Position and velocity estimator based on a Kalman Filter.
 * This is the algorithm used in Mini Cheetah and Cheetah 3.
 */
//template <typename T>
class LinearKFPositionVelocityEstimator {
 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //LinearKFPositionVelocityEstimator();
void run(float dt);
void setup(float dt);
int en_update;
int rst_kf;
int force_ground;
int use_kf_out;
 private:

  Eigen::Matrix<float, 18, 1> _xhat;//状态估计值 [p v p1 p2 p3 p4] 世界坐标下
  Eigen::Matrix<float, 12, 1> _ps;//储存状态p
  Eigen::Matrix<float, 12, 1> _vs;//储存状态v
  Eigen::Matrix<float, 18, 18> _A;//状态转移阵
  Eigen::Matrix<float, 18, 18> _Q0;//初始状态估计噪声
  Eigen::Matrix<float, 18, 18> _P;//初始不确定性
  Eigen::Matrix<float, 28, 28> _R0;//初始观测噪声
  Eigen::Matrix<float, 18, 3> _B;//输入阵
  Eigen::Matrix<float, 28, 18> _C;//观测阵
  Vec3m<float> position;
  Vec3m<float> vWorld;
};

/*!
 * "Cheater" position and velocity estimator which will return the correct position and
 * velocity when running in simulation.
 */
//template<typename T>
//class CheaterPositionVelocityEstimator {
//public:
//  virtual void run();
//  virtual void setup() {}
//};

#endif  // PROJECT_POSITIONVELOCITYESTIMATOR_H
