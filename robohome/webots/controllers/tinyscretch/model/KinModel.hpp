#ifndef KIN_MODEL
#define KIN_MODEL

#include "RobotDefinition.hpp"
#include <rbdl.h>
#include <iostream>

class KinModel{
    public:
        KinModel(RigidBodyDynamics::Model* model);
        ~KinModel(void);
        //in world frame
        void getPos(int link_id, Vec3 & pos);
        void getOri(int link_id, Vec3 & rpy);
        void getLinearVel(int link_id, Vec3 & vel);
        void getAngularVel(int link_id, Vec3 & ang_vel);
        void getJacobian(int link_id, MatrixXd & J);
        void getJDotQdot(int link_id, VectorXd & JDotQdot);
        void getCoMJacobian  (MatrixXd & J) const;
        void getCoMPos  (Vec3 & com_pos) const;
        void getCoMVel (Vec3 & com_vel) const;
        void getCentroidInertia(MatrixXd & Icent){ Icent = Ig_; }
        void getCentroidJacobian(MatrixXd & Jcent){ Jcent = Jg_; }
        void getCentroidMatrix(MatrixXd & Mcent){ Mcent = Ag; }
        //void getCentroidVelocity(VectorXd & centroid_vel);
        //matrix from world frame to body frame
        void getWorldToBodyMatrix(Mat3 & BodyMatrix); 

        void UpdateKinematics(const VectorXd & q, const VectorXd & qdot);

        unsigned int _find_body_idx(int id) const;
        unsigned int find_body_id(const char* link_name) const;
        void displaylinks();

        Vec3 com_pos_;
        VectorXd centroid_vel_;
    protected:
        double gravity_;
        void _UpdateCentroidFrame(const VectorXd & q, const VectorXd & qdot);
        MatrixXd Ig_, Ig_2;
        MatrixXd Jg_;
        MatrixXd Ag, Ag_2;

        RigidBodyDynamics::Model* model_;
};

#endif
