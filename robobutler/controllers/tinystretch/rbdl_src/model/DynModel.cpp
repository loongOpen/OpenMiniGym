//unuse
#include "DynModel.hpp"

using namespace RigidBodyDynamics::Math;

DynModel::DynModel(RigidBodyDynamics::Model* model){
    model_ = model;
}

DynModel::~DynModel(){
}

bool DynModel::getMassInertia(MatrixXd & a){
    a = A_;
    return true;
}

bool DynModel::getInverseMassInertia(MatrixXd & ainv){
    ainv = Ainv_;
    return true;
}

bool DynModel::getGravity(VectorXd &  grav){
    grav = grav_;
    return true;
}
bool DynModel::getCoriolis(VectorXd & coriolis){
    coriolis = coriolis_;
    return true;
}

void DynModel::UpdateDynamics(const VectorXd & q, 
        const VectorXd & qdot){
    // Mass Matrix
    A_ = MatrixXd::Zero(model_->qdot_size, model_->qdot_size);
    CompositeRigidBodyAlgorithm(*model_, q, A_, false);

    // Ainv_ = A_.inverse();
    pseudoInverse(A_, 1.e-10, Ainv_, 0);

    VectorXd ZeroQdot = VectorXd::Zero(model_->qdot_size);
    // Gravity
    VectorXd grav_tmp = VectorXd::Zero(model_->qdot_size);
    InverseDynamics(*model_, q, ZeroQdot, ZeroQdot, grav_tmp);
    grav_ = grav_tmp;
    // Coriolis
    VectorXd coriolis_tmp = VectorXd::Zero(model_->qdot_size);
    InverseDynamics(*model_, q, qdot, ZeroQdot, coriolis_tmp);

    coriolis_ = coriolis_tmp - grav_;
}
