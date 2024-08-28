#include "KinModel.hpp"

using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics;

//extern variables in RobotDefinition.hpp
const char* link_name[] =
{
    "trunk",
    "FR_hip","FR_thigh","FR_calf",
    "FL_hip","FL_thigh","FL_calf",
    "RR_hip","RR_thigh","RR_calf",
    "RL_hip","RL_thigh","RL_calf"
};

KinModel::KinModel( RigidBodyDynamics::Model* model):
    gravity_(9.81)
{
    model_ = model;
    Ig_ = MatrixXd::Zero(6,6);
    Jg_ = MatrixXd::Zero(6, model_->qdot_size);
}

KinModel::~KinModel(){
}

void KinModel::UpdateKinematics(const VectorXd & q, 
        const VectorXd & qdot){
    _UpdateCentroidFrame(q, qdot);
}
/********mainly reference to "Dynamic behaviors on the NAO robot with closed-loop 
         whole body operational space control" by Donghyun Kim**************************/
void KinModel::_UpdateCentroidFrame(const VectorXd & q,
        const VectorXd & qdot){
    double mass;
    Vec3 zero_vector;
    zero_vector.setZero();

    Vec3 com_pos;
    Vec3 cm;
    Vec3 link_pos;
    Vec3 p_g;

    getCoMPos(com_pos);
    //cout << "kim_compos:\n" << com_pos.transpose() << "\n\n";
    com_pos_ = com_pos;

    MatrixXd Xg_inv = MatrixXd::Zero(6, 6);
    MatrixXd link_X_com = MatrixXd::Zero(6, 6);
    Ig_.setZero();
    Ag = MatrixXd::Zero(6, model_->qdot_size);
    Ig_2 = MatrixXd::Zero(6,6);
    Ag_2 = MatrixXd::Zero(6, model_->qdot_size);
    //std::cout << "model_->qdot_size: " << model_->qdot_size << "\n\n";
    MatrixXd I = MatrixXd::Zero(6, 6);
    MatrixXd Jsp = MatrixXd::Zero(6, model_->qdot_size);

    int start_idx = _find_body_idx(robot_link::trunk);
    Matrix3d p;
    Matrix3d cmm;
    Matrix3d R;

    for (int i(start_idx); i<model_->mBodies.size(); ++i){
        R = CalcBodyWorldOrientation(*model_, q, i, false);

        link_pos = CalcBodyToBaseCoordinates ( *model_, q, i, zero_vector, false);

        Jsp.setZero();
        CalcBodySpatialJacobian( *model_, q, i, Jsp, false);

        mass = model_->mBodies[i].mMass;
        I.setZero();
        cm = model_->mBodies[i].mCenterOfMass;
        //cout << "m: " << mass << " mass center: " << cm.transpose() << "\n\n";
        cmm = vecCross(cm);
        I.setZero();
        I.block(0, 0, 3, 3) = model_->mBodies[i].mInertia + mass * cmm * cmm.transpose();
        I.block(0,3, 3,3) = mass * cmm;
        I.block(3,0, 3,3) = mass * cmm.transpose();
        I.block(3, 3, 3, 3) = mass * MatrixXd::Identity(3,3);

        link_X_com.block(0,0,3,3) = R;
        link_X_com.block(3,3,3,3) = R;
        link_X_com.block(3,0,3,3) = -R * vecCross(link_pos - com_pos);
        Ig_ = Ig_ + link_X_com.transpose() * I * link_X_com;
        //xyz rpy joint
        Ag = Ag + link_X_com.transpose() * I * Jsp;
        //kim's code, not intuitive
        // p_g = R * (com_pos - link_pos);
        // p = vecCross(p_g);
        // Xg_inv.block(0,0, 3,3) = R;
        // Xg_inv.block(3,3, 3,3) = R;
        // Xg_inv.block(3,0, 3,3) = p * R;
        // Ig_2 = Ig_2 + Xg_inv.transpose() * I * Xg_inv;
        // Ag_2 = Ag_2 + Xg_inv.transpose() * I * Jsp;
    }
    // bool Ig_equal, Ag_equal;
    // Ig_equal = Ig_.isApprox(Ig_2, 1e-12);
    // Ag_equal = Ag.isApprox(Ag_2, 1e-12);
    // cout << "is equal to kim\n";
    // cout << "Ig: " << Ig_equal << "  Ag: " << Ag_equal << endl;
    MatrixXd lin_mat = Ag.block(0,0,3,6);
    MatrixXd ang_mat = Ag.block(3,0,3,6);
    //moemntum: rpy xyz
    //vel: rpy xyz joint
    Ag.block(0,0,3,6) << lin_mat.rightCols(3), lin_mat.leftCols(3);
    Ag.block(3,0,3,6) << ang_mat.rightCols(3), ang_mat.leftCols(3);
    Jg_ = Ig_.inverse() * Ag;

    //centroid_vel_ = Jg_ * qdot;
}

void KinModel::getCoMJacobian(MatrixXd & Jcom) const {
    Vec3 zero_vector = Vec3::Zero();
    VectorXd q(robot::num_q);

    Jcom = MatrixXd::Zero(3, model_->qdot_size);
    MatrixNd J(3, model_->qdot_size);

    double mass;
    double tot_mass(0.0);
    int start_idx = _find_body_idx(robot_link::trunk);

    for (int i(start_idx); i< model_->mBodies.size() ; ++i){
        mass = model_->mBodies[i].mMass;
        // CoM Jacobian Update
        J.setZero();
        CalcPointJacobian(*model_, q, i, model_->mBodies[i].mCenterOfMass, J, false);
        Jcom +=  mass * J;
        tot_mass += mass;
    }
    Jcom /= tot_mass;
}

void KinModel::getCoMPos(Vec3 & CoM_pos)const {
    Vec3 zero_vector = Vec3::Zero();
    VectorXd q(robot::num_q);

    CoM_pos.setZero();
    Vec3 link_pos;

    int start_idx = _find_body_idx(robot_link::trunk);
    double mass;
    double tot_mass(0.0);
    for (int i(start_idx); i< model_->mBodies.size() ; ++i){
        mass = model_->mBodies[i].mMass;

        // CoM position Update
        link_pos = CalcBodyToBaseCoordinates ( *model_, q, i, 
                model_->mBodies[i].mCenterOfMass, false);
        CoM_pos += mass * link_pos;
        tot_mass += mass;
    }
    CoM_pos /= tot_mass;
}

void KinModel::getCoMVel(Vec3 & CoM_vel) const {
    VectorXd q, qdot;
    int start_idx = _find_body_idx(robot_link::trunk);
    CoM_vel.setZero();
    Vec3 link_vel;

    Vec3 zero_vector = Vec3::Zero();
    double mass;
    double tot_mass(0.0);
    for (int i(start_idx); i< model_->mBodies.size() ; ++i){
        mass = model_->mBodies[i].mMass;

        // CoM velocity Update
        link_vel = CalcPointVelocity ( *model_, q, qdot, i, model_->mBodies[i].mCenterOfMass, false);
        //cout << link_vel << endl;
        CoM_vel += mass * link_vel;
        tot_mass += mass;
    }
    CoM_vel /= tot_mass;
}

void KinModel::getPos(int link_id, Vec3 & pos){
    Vec3 zero;
    Matrix3d R;
    VectorXd q(robot::num_q);

    int bodyid = _find_body_idx(link_id);
    if(bodyid >=model_->fixed_body_discriminator){
        zero = model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass;
    }
    else{
        zero =  model_->mBodies[bodyid].mCenterOfMass;
    }

    pos = CalcBodyToBaseCoordinates(*model_, q, _find_body_idx(link_id), zero, false);
    // R = CalcBodyWorldOrientation( *model_, q, _find_body_idx(link_id), false);
    // pos = R * pos;

}

void KinModel::getOri(int link_id, Vec3 & rpy){
    Matrix3d R;
    eigenQuaternion ori;
    VectorXd q(robot::num_q);
    R = CalcBodyWorldOrientation( *model_, q, _find_body_idx(link_id), false);
    ori = R.transpose();

    if(ori.w() < 0.){
        ori.w() *= (-1.);
        ori.x() *= (-1.);
        ori.y() *= (-1.);
        ori.z() *= (-1.);
    }
    quaToRpy(ori, rpy[2], rpy[1], rpy[0]);
}

void KinModel::getLinearVel(int link_id, Vec3 & vel){
    Vec3 zero;
    VectorXd q, qdot;

    int bodyid = _find_body_idx(link_id);
    if(bodyid >=model_->fixed_body_discriminator){
        zero = model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass;
    }
    else{
        zero =  model_->mBodies[bodyid].mCenterOfMass;
    }

    vel = CalcPointVelocity ( *model_, q, qdot, _find_body_idx(link_id), zero, false);
}

void KinModel::getAngularVel(int link_id, Vec3 & ang_vel){
    unsigned int bodyid = _find_body_idx(link_id);
    VectorXd vel, q, qdot;

    if(bodyid >=model_->fixed_body_discriminator){
        vel = CalcPointVelocity6D(*model_, q, qdot, bodyid,
                model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass, false);
    }
    else{
        vel = CalcPointVelocity6D(*model_, q, qdot, bodyid,
                model_->mBodies[bodyid].mCenterOfMass, false);
    }
    ang_vel = vel.head(3);
}

void KinModel::getJacobian(int link_id, MatrixXd &J){
    VectorXd q(robot::num_q);
    J = MatrixXd::Zero(6, model_->qdot_size);

    unsigned int bodyid = _find_body_idx(link_id);
    Vec3 zero_vector = Vec3::Zero();
    
    if(bodyid >=model_->fixed_body_discriminator){
        CalcPointJacobian6D(*model_, q, bodyid,
                model_->mFixedBodies
                [bodyid - model_->fixed_body_discriminator].mCenterOfMass,
                J, false);
    }
    else{
        CalcPointJacobian6D(*model_, q, bodyid,
                model_->mBodies[bodyid].mCenterOfMass,
                J, false);
    }
    /*************this will transform the jacobian from world frame to body frame********/

    // Eigen::Matrix<double, 6, 6> World2Body6D;
    // Eigen::Matrix3d World2Body;
    // World2Body6D.setZero(); World2Body.setZero();
    // World2Body = CalcBodyWorldOrientation( *model_, q, _find_body_idx(link_id), false);
    // World2Body6D.block(0,0,3,3) = World2Body;
    // World2Body6D.block(3,3,3,3) = World2Body;
    
    // J = World2Body6D * J;
}

void KinModel::getJDotQdot(int link_id, VectorXd & JDotQdot){
    VectorXd q, qdot, qddot;

    unsigned int bodyid = _find_body_idx(link_id);
    if(bodyid >=model_->fixed_body_discriminator){
        JDotQdot = CalcPointAcceleration6D(*model_, q, qdot, qddot, bodyid,
                model_->mFixedBodies
                [bodyid - model_->fixed_body_discriminator].mCenterOfMass, false);
    }
    else{
        JDotQdot = CalcPointAcceleration6D(*model_, q, qdot, qddot, bodyid,
                model_->mBodies[bodyid].mCenterOfMass, false);
    }
    JDotQdot[5] -= gravity_;
}

void KinModel::getWorldToBodyMatrix(Mat3 & BodyMatrix){
    VectorXd q;
    int body_idx = _find_body_idx(robot_link::trunk);

    BodyMatrix = CalcBodyWorldOrientation(*model_, q, body_idx, false);
}

unsigned int KinModel::_find_body_idx(int id) const {
    switch(id){
        case robot_link::trunk:
            return model_->GetBodyId("trunk");
        case robot_link::FR_foot:
            return model_->GetBodyId("FR_foot");
        case robot_link::FL_foot:
            return model_->GetBodyId("FL_foot");
        case robot_link::RR_foot:
            return model_->GetBodyId("RR_foot");
        case robot_link::RL_foot:
            return model_->GetBodyId("RL_foot");
        default:
            std::cout << "unkonown id" << std::endl;
    }
    return 0;
}
unsigned int KinModel::find_body_id(const char* link_name) const{
    unsigned int body_id;
    body_id = model_->GetBodyId(link_name);

    return body_id;
}
//according to this function, to know the number of each joint
void KinModel::displaylinks() {
    int linklist_len;
    linklist_len = sizeof(link_name)/sizeof(link_name[0]);
    cout << "link_num: " << linklist_len << endl;
    vector<const char*> link_name_list(link_name, link_name + linklist_len);
    vector<int> body_id_list;
    int body_id;
    for (int i(0); i < link_name_list.size(); ++i) {
        body_id = model_->GetBodyId(link_name_list[i]);
        body_id_list.push_back(body_id);
    }
    for (int i(0); i < body_id_list.size(); ++i) {
        cout << link_name_list[i] << ": " << body_id_list[i] << endl;
    }
}
