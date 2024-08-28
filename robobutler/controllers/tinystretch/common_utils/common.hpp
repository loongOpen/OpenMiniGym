#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <rbdl.h>

using std::cout;
using std::endl;
using std::vector;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::VectorXd;

typedef Eigen::Quaternion<double> eigenQuaternion;
typedef Matrix<double,2,1> Vec2;
typedef Matrix<double,3,1> Vec3;
typedef Matrix<double,4,1> Vec4;
typedef Matrix<double,3,3> Mat3;

typedef RigidBodyDynamics::Math::Vector3d rbdl_Vec3;

void rpyToQua(double yaw, double pitch, double roll,eigenQuaternion& to);
void quaToRpy(const eigenQuaternion & from, double & yaw, double & pitch, double & roll);
Mat3 RotX_Matrix(double roll_angle);
Mat3 RotY_Matrix(double pitch_angle);
Mat3 RotZ_Matrix(double yaw_angle);
Matrix<double,3,3> vecCross(Matrix<double,3,1> v);
void pseudoInverse(MatrixXd const & matrix,
                       double sigmaThreshold,
                       MatrixXd & invMatrix,
                       VectorXd * opt_sigmaOut);