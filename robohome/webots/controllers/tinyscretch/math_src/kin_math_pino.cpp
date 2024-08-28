#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "locomotion_header.h"
#include <iostream>
#define ARM_URDF "/home/tinymal/文档/tinystrech/protos/TinyStretch/urdf/TinyStretch_arm.urdf"
Eigen::Matrix<double,3,3> eul2Rot(double roll, double pitch, double yaw); // 欧拉角转旋转矩阵，ZYX顺序

// 欧拉角转旋转矩阵，ZYX顺序
Eigen::Matrix<double, 3, 3> eul2Rot(double roll, double pitch, double yaw) {
    Eigen::Matrix<double,3,3> Rx,Ry,Rz;
    Rz<<cos(yaw),-sin(yaw),0,
            sin(yaw),cos(yaw),0,
            0,0,1;
    Ry<<cos(pitch),0,sin(pitch),
            0,1,0,
            -sin(pitch),0,cos(pitch);
    Rx<<1,0,0,
            0,cos(roll),-sin(roll),
            0,sin(roll),cos(roll);
    return Rz*Ry*Rx;
}


int ik_pino(END_POS end_pos,END_POS end_att,float q_out[6])//input rad
{
    // 导入urdf模型，初始化数据容器
    const std::string urdf_robot_filename = std::string(ARM_URDF);
    static pinocchio::Model model_biped;
    static int init=0;
    if(!init){
        init=1;
        pinocchio::urdf::buildModel(urdf_robot_filename, model_biped);
    }
    pinocchio::Data data_biped(model_biped);

    auto l_ankle_Joint=model_biped.getJointId("arm4_joint");

    Eigen::Matrix3d Rdes;
    Rdes<<  1,0,0,
            0,1,0,
            0,0,1; // note for the current URDF, the foot-end coordinate does NOT align with the one of the baselink
    Rdes=eul2Rot(end_att.x,end_att.y,end_att.z);
    //std::cout << Rdes << std::endl;
    Eigen::Vector3d Pdes;
    Pdes<<end_pos.x,end_pos.y,end_pos.z;  // for left leg

    const pinocchio::SE3 oMdes(Rdes, Pdes);

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_biped.nv);
   // std::cout <<model_biped.nv<< std::endl;

    const double eps  = 1e-4;
    const int IT_MAX  = 100;
    const double DT   = 10e-1;
    const double damp = 1e-3;

    Eigen::MatrixXd J(6,model_biped.nv);
    J.setZero();

    bool success = false;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err;
    Eigen::VectorXd v(model_biped.nv);

    int itr_count{0};
    for (itr_count=0;itr_count<IT_MAX; itr_count++)
    {
        pinocchio::forwardKinematics(model_biped,data_biped,q);
        const pinocchio::SE3 iMd = data_biped.oMi[l_ankle_Joint].actInv(oMdes);
        err = pinocchio::log6(iMd).toVector();  // in joint frame
        if(err.norm() < eps)
        {
            success = true;
            break;
        }
        if (itr_count >= IT_MAX)
        {
            success = false;
            break;
        }
        pinocchio::computeJointJacobian(model_biped,data_biped,q,l_ankle_Joint,J);  // J in joint frame
        pinocchio::Data::Matrix6 Jlog;
        pinocchio::Jlog6(iMd.inverse(), Jlog);
        J = -Jlog * J;
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = - J.transpose() * JJt.ldlt().solve(err);
        q = pinocchio::integrate(model_biped,q,v*DT);
//        if(!(itr_count % 10))
//            std::cout << itr_count << ": error = " << err.transpose() << std::endl;
    }

    if(success)
    {
        //std::cout << "Convergence achieved!" << std::endl;
        //std::cout<< "itr count= "<<itr_count<<std::endl;
    }
    else
    {
        std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
    }

   // std::cout << "\nresult: " << q.transpose()*57.3 << std::endl;
   // std::cout << "\nfinal error: " << err.transpose() << std::endl;
    q_out[0]=q[0];
    q_out[1]=q[1];
    q_out[2]=q[2];
    q_out[3]=q[3];
    q_out[4]=q[4];
    q_out[5]=q[5];
    return 0;
}


int fk_pino(float q_in[6],END_POS* end_pos,END_POS* end_att)
{
    // 导入urdf模型，初始化数据容器

    const std::string urdf_robot_filename = std::string(ARM_URDF);
    static pinocchio::Model model_biped;
    static int init=0;
    if(!init){
        init=1;
        pinocchio::urdf::buildModel(urdf_robot_filename, model_biped);
    }
    pinocchio::Data data_biped(model_biped);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_biped.nv);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(model_biped.nv);
    auto l_ankle_Joint=model_biped.getJointId("arm4_joint");

    q[0]=q_in[0];
    q[1]=q_in[1];
    q[2]=q_in[2];
    q[3]=q_in[3];
    q[4]=q_in[4];
    q[5]=q_in[5];

    pinocchio::forwardKinematics(model_biped,data_biped,q);
    Eigen::MatrixXd J(6,model_biped.nv);
    J.setZero();
    pinocchio::computeJointJacobians(model_biped,data_biped,q); // 计算所有关节的雅科比矩阵
    pinocchio::getJointJacobian(model_biped,data_biped,l_ankle_Joint,pinocchio::LOCAL,J); // 提取指定关节的雅科比矩阵，在本体坐标系下
    //std::cout<<"epos="<<data_biped.oMi[l_ankle_Joint].translation().transpose()<<std::endl;rotation
    end_pos->x=data_biped.oMi[l_ankle_Joint].translation()(0);
    end_pos->y=data_biped.oMi[l_ankle_Joint].translation()(1);
    end_pos->z=data_biped.oMi[l_ankle_Joint].translation()(2);

    // 使用Eigen将旋转矩阵转换为欧拉角
    Eigen::Vector3d eulerAngle1 = data_biped.oMi[l_ankle_Joint].rotation().eulerAngles(2,1,0); // ZYX顺序，yaw,pitch,roll

   // std::cout <<data_biped.oMi[l_ankle_Joint].rotation() << std::endl;
    if(eulerAngle1[2]>EIGEN_PI/2)
        eulerAngle1[2]=-(EIGEN_PI -eulerAngle1[2]);
    if(eulerAngle1[1]<0)
        eulerAngle1[1]=-(EIGEN_PI +eulerAngle1[1]);
    if(eulerAngle1[0]>EIGEN_PI/2)
        eulerAngle1[0]=-(EIGEN_PI -eulerAngle1[0]);
    end_att->x=eulerAngle1[2]*57.3 ;
    end_att->y=eulerAngle1[1]*57.3 ;
    end_att->z=eulerAngle1[0]*57.3 ;
   // std::cout << "roll_1 pitch_1 yaw_1 = " << eulerAngle1[2]*57.3 << " " << eulerAngle1[1]*57.3 << " " << eulerAngle1[0]*57.3 << std::endl;
   // std::cout <<     data_biped.oMi[l_ankle_Joint].translation()(0)<<std::endl;
    //std::cout << "\nJ: " <<J << std::endl;
    return 0;
}
