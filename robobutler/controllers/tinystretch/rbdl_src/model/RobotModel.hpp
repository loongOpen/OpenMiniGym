#ifndef ROBOT_MODEL
#define ROBOT_MODEL

#include <rbdl.h>
#include "RobotDefinition.hpp"

class DynModel;
class KinModel;

class RobotModel{
public:
    RobotModel();
     ~RobotModel(void);

     bool getMassInertia(MatrixXd & A) const ;
     bool getInverseMassInertia(MatrixXd & Ainv) const; 
     bool getGravity(VectorXd & grav) const;
     bool getCoriolis(VectorXd & coriolis) const;
    //in world frame
     void getCentroidJacobian(MatrixXd & Jcent) const;
     void getCentroidInertia(MatrixXd & Icent) const;
     void getCentroidMatrix(MatrixXd & Mcent) const;
    // according to "Improved computation of the humanoid centroidal dynamics and application for 
    // whole body control" by Patrick Wensing
    //when rpy not equal 0, result is a little fault!!!
     void getAg_AgdotQdot(MatrixXd & Ag, MatrixXd & AgdotQdot);

    //test dwl's code, https://github1s.com/robot-locomotion/dwl/blob/HEAD/dwl/dwl/model/WholeBodyDynamics.cpp#L204
    void getComState_dwl(const VectorXd & q, const VectorXd & qdot,
        double& total_mass, rbdl_Vec3& com_pos, rbdl_Vec3& com_vel,
        rbdl_Vec3& ang_momentum);
    
    //when rpy not equal 0, result is a little fault!!!
    void getCentroidInertia_dwl(const VectorXd & q, MatrixXd & Icent);


     void getCoMPosition(Vec3 & com_pos) const;
     void getCoMVelocity(Vec3 & com_vel) const;
     void getPos(int link_id, Vec3 & pos) const;
     void getOri(int link_id, Vec3 & rpy) const;
     void getLinearVel(int link_id, Vec3 & lin_vel) const;
     void getAngularVel(int link_id, Vec3 & ang_vel) const;
     //void getCentroidVelocity(VectorXd & centroid_vel) const;
     void getCoMJacobian(MatrixXd & J) const;
     void getFullJacobian(int link_id, MatrixXd & J) const;
     void getFullJDotQdot(int link_id, VectorXd & JDotQdot) const;
    //matrix from world frame to body frame
     void getWorld2BodyMatrix(Mat3 & _World2Body);

     void UpdateSystem(const VectorXd & q, const VectorXd & qdot);
    //according to this function, to know the number of each joint
     void PrintLinkList();
     unsigned int FindLinkId(const char* link_name);
protected:
    DynModel* dyn_model_;
    KinModel* kin_model_;

    RigidBodyDynamics::Model* model_;
};

#endif
