#include "RobotModel.hpp"
#include "DynModel.hpp"
#include "KinModel.hpp"
#include <urdfreader.h>
#include <stdio.h>

#define THIS_COM "C:/Users/hp/Desktop/moco12/controllers/my_controller12/urdf/"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

RobotModel::RobotModel(){
  model_ = new Model();
  rbdl_check_api_version (RBDL_API_VERSION);

  if (!Addons::URDFReadFromFile (
              THIS_COM"aliengo.urdf", model_, true, false)) {
    std::cerr << "Error loading model aliengo.urdf" << std::endl;
    abort();
  }

  dyn_model_ = new DynModel(model_);
  kin_model_ = new KinModel(model_);

  printf("[Aliengo Model] Contructed\n");
}

RobotModel::~RobotModel(){
  delete dyn_model_;
  delete kin_model_;
  delete model_;
}

void RobotModel::PrintLinkList() {
    kin_model_->displaylinks();
}
unsigned int RobotModel:: FindLinkId(const char* _link_name) {
	return  kin_model_->find_body_id(_link_name);
	//return 1;	
}

void RobotModel::UpdateSystem(const VectorXd & q, 
        const VectorXd & qdot){
  VectorXd qddot = qdot; qddot.setZero();
  UpdateKinematicsCustom(*model_, &q, &qdot, &qddot);
  dyn_model_->UpdateDynamics(q, qdot);
  kin_model_->UpdateKinematics(q, qdot);
}

void RobotModel::getCentroidInertia(MatrixXd & Icent) const {
  kin_model_->getCentroidInertia(Icent);
}

void RobotModel::getCentroidJacobian(MatrixXd & Jcent) const {
  Jcent.setZero();
  kin_model_->getCentroidJacobian(Jcent);
}
void RobotModel::getCentroidMatrix(MatrixXd & Mcent) const{
  kin_model_ -> getCentroidMatrix(Mcent);
}
// according to "Improved computation of the humanoid centroidal dynamics and application for 
// whole body control" by Patrick Wensing
//when rpy not equal 0, result is a little fault!!!
void RobotModel::getAg_AgdotQdot(MatrixXd & Ag, MatrixXd & AgdotQdot){
  MatrixXd H, U1;
  VectorXd Cq;
  Mat3 body_R_base;
  Vec3 compos;

  U1.setZero(6, robot::num_qdot);
  U1.block(0,0,6,6) = MatrixXd::Identity(6,6);

  dyn_model_->getMassInertia(H);
  MatrixXd base_lin_mat = H.block(0,0,3,6);
  MatrixXd base_ang_mat = H.block(3,0,3,6);
  MatrixXd joint_lin_mat = H.block(0,6,3,12);
  MatrixXd joint_ang_mat = H.block(3,6,3,12);
  //moemntum: rpy xyz
  //vel: rpy xyz joint
  H.block(0,0,3,6) << base_ang_mat.rightCols(3), base_ang_mat.leftCols(3);
  H.block(3,0,3,6) << base_lin_mat.rightCols(3), base_lin_mat.leftCols(3);
  H.block(0,6,3,12) = joint_ang_mat;
  H.block(3,6,3,12) = joint_lin_mat;
  dyn_model_->getCoriolis(Cq);
  getCoMPosition(compos);
  RigidBodyDynamics::Math::SpatialTransform base_X_com(Mat3::Identity(), -compos);
  getWorld2BodyMatrix(body_R_base);
  RigidBodyDynamics::Math::SpatialTransform body_X_base(body_R_base, Vec3::Zero());
  Ag = base_X_com.toMatrixTranspose() * U1 * H;
  Ag.block(0,0,3,18) = (base_X_com.toMatrixTranspose() * body_X_base.toMatrixTranspose() * U1 * H).block(0,0,3,18);
  Ag.block(0,3,3,3) = Mat3::Zero();

  AgdotQdot = base_X_com.toMatrixTranspose() * U1 * Cq;

}

void RobotModel::getComState_dwl(const VectorXd & q, const VectorXd & qdot,
        double& total_mass, rbdl_Vec3& com_pos, rbdl_Vec3& com_vel,
        rbdl_Vec3& ang_momentum){
	RigidBodyDynamics::Utils::CalcCenterOfMass(*model_, q, qdot, NULL, total_mass, com_pos, &com_vel, NULL, &ang_momentum);
}

 //when rpy not equal 0, result is a little fault!!!
void RobotModel::getCentroidInertia_dwl(const VectorXd & q, MatrixXd & Icent){
  MatrixXd joint_inertia_mat;
  //crba's mass matrix is based on Body frame
  dyn_model_ -> getMassInertia(joint_inertia_mat);
  rbdl_Vec3 com_pos, comvel, angmomen;
  Eigen::VectorXd qd = Eigen::VectorXd::Zero(robot::num_qdot);
  double mass;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
  getComState_dwl(q, qd, mass, com_pos, comvel, angmomen);
  RigidBodyDynamics::Math::SpatialTransform base_X_com(Mat3::Identity(), -com_pos);
  Mat3 baseTobody;
  getWorld2BodyMatrix(baseTobody);
  RigidBodyDynamics::Math::SpatialTransform body_X_base(baseTobody, Vec3::Zero());
  //cout << "dwl_compos:\n" << com_pos.transpose() << "\n\n";
  //cout << vecCross(com_pos) << "\n\n" << base_X_com.toMatrix()<< "\n\n";
  MatrixXd base_inertia_mat = MatrixXd::Zero(6,6);
  base_inertia_mat = body_X_base.toMatrixTranspose() * joint_inertia_mat.block(0,0,6,6) * body_X_base.toMatrix();
  MatrixXd base_lin_mat = base_inertia_mat.block(0,0,3,6);
	MatrixXd base_ang_mat = base_inertia_mat.block(3,0,3,6);
  MatrixXd Ibase;
  Ibase.resize(6,6);
  Ibase.block(0,0,3,6) << base_ang_mat.rightCols(3), base_ang_mat.leftCols(3);
  Ibase.block(3,0,3,6) << base_lin_mat.rightCols(3), base_lin_mat.leftCols(3);

  Icent = base_X_com.toMatrixTranspose() * Ibase * base_X_com.toMatrix();

}

bool RobotModel::getInverseMassInertia(MatrixXd & Ainv) const {
  return dyn_model_->getInverseMassInertia(Ainv);
}

bool RobotModel::getMassInertia(MatrixXd & A) const {
  return dyn_model_->getMassInertia(A);
}

bool RobotModel::getGravity(VectorXd & grav) const {
  return dyn_model_->getGravity(grav);
}

bool RobotModel::getCoriolis(VectorXd & coriolis) const {
  return dyn_model_->getCoriolis(coriolis);
}

void RobotModel::getFullJacobian(int link_id, MatrixXd & J) const {
  //J = MatrixXd::Zero(6, mercury::num_qdot);
  kin_model_->getJacobian(link_id, J);
}

void RobotModel::getFullJDotQdot(int link_id, VectorXd & JDotQdot) const{
    kin_model_->getJDotQdot(link_id, JDotQdot);
}

void RobotModel::getPos(int link_id, Vec3 & pos) const {
    kin_model_->getPos(link_id, pos);
}
void RobotModel::getOri(int link_id, Vec3 & rpy) const {
    kin_model_->getOri(link_id, rpy);
}
void RobotModel::getLinearVel(int link_id, Vec3 & vel) const {
    kin_model_->getLinearVel(link_id, vel);
}
void RobotModel::getAngularVel(int link_id, Vec3 & ang_vel) const {
    kin_model_->getAngularVel(link_id, ang_vel);
}

void RobotModel::getCoMJacobian(MatrixXd & J) const {
    J = MatrixXd::Zero(3, model_->qdot_size);
    kin_model_->getCoMJacobian(J);
}

void RobotModel::getCoMPosition(Vec3 & com_pos) const {
  com_pos = kin_model_->com_pos_;
}

void RobotModel::getCoMVelocity(Vec3 & com_vel) const {
    kin_model_->getCoMVel(com_vel);
}
// void RobotModel::getCentroidVelocity(VectorXd & centroid_vel) const {
//   centroid_vel = kin_model_->centroid_vel_;
// }
void RobotModel::getWorld2BodyMatrix(Mat3 & _World2Body){
  kin_model_ -> getWorldToBodyMatrix(_World2Body);
}
