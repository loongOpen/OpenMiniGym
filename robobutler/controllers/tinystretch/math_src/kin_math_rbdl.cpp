#include "locomotion_header.h"
#if RUN_WEBOTS||1
#include <Eigen/Dense>
#include "common_types.h"
#include "cppTypes.h"
#include "RobotModel.hpp"
#include "common.hpp"
#include "log_pinRefined.h"
RobotModel* _model_rbdl = new RobotModel();

struct modelStateData{
    VectorXd _q;
    VectorXd _qdot;
};

void rbdl_test(void) {
    RobotModel* _model = new RobotModel();
    _model->PrintLinkList();
    struct modelStateData _state = {
        VectorXd::Zero(robot::num_q),
        VectorXd::Zero(robot::num_qdot)
    };
    double mass;
    Vec3 RPY, linkPos, linkOri, linkVel, linkAngVel;
    MatrixXd cJ, cI, cI_dwl, cenInerJac, Ag_kim, Ag_wensing, AgdotQdot_wensing, H;
    VectorXd L, comvel_2, L_wensing, qdot, G;
    Vec3 compos, comvel, linmomen_dwl;
    rbdl_Vec3 angmomen_dwl, compos_dwl, comvel_dwl;
    Mat3 World2BodyInRbdl, Body2WorldMyself;
    RPY << 0, 0, 0;
    Mat3 R = RotZ_Matrix(RPY[2]) * RotY_Matrix(RPY[1]) * RotX_Matrix(RPY[0]);
    MatrixXd R_6D = MatrixXd::Zero(6,6);
    R_6D.block(0,0,3,3) = R;
    R_6D.block(3,3,3,3) = R;
    eigenQuaternion qua(R);
    _state._q << 0, 0, 0, qua.x(), qua.y(), qua.z(),//6 for vitual
        0.1, 0., 0,//dof num
        0,  0,  0,
        qua.w();
    //xyz rpy
    _state._qdot << 0, 0, 0, 0, 0, 0,//6 for vitual
        0, 0, 0.,
        0, 0, 0;//dof num
    _model -> UpdateSystem(_state._q, _state._qdot);

    _model -> getCoMPosition(compos);
    _model -> getCoMVelocity(comvel);
    _model -> getAg_AgdotQdot(Ag_wensing, AgdotQdot_wensing);
    _model -> getCentroidJacobian(cJ);
    _model -> getCentroidInertia(cI);
    _model -> getCentroidInertia_dwl(_state._q, cI_dwl);
    _model -> getCentroidMatrix(Ag_kim);
    _model -> getMassInertia(H);
    _model -> getGravity(G);

    _model -> getComState_dwl(_state._q, _state._qdot, mass, compos_dwl, comvel_dwl, angmomen_dwl);
    int link_end=_model ->FindLinkId("arm4_link");
    std::cout<<link_end<<std::endl;
    _model ->getPos(link_end,linkPos);
    std::cout<<"epos:"<<linkPos<<std::endl;
//    qdot.resize(NUM_DQ);
//    //vel: rpy xyz joint
//    qdot << _state._qdot.segment(3,3), _state._qdot.segment(0,3), _state._qdot.tail(12);
//    L = Ag_kim * qdot;
//    comvel_2 = cJ * qdot;
//    linmomen_dwl = mass * comvel_dwl;
//    L_wensing = Ag_wensing *  qdot;

//    cout << "compos\n";
//        cout << "kim\n" << compos.transpose() << endl;
//        cout << "dwl\n" << compos_dwl.transpose() << endl;
//    cout << "com linear vel\n";
//        cout << "kim\n" << comvel.transpose() << endl;
//        cout << "dwl\n" << comvel_dwl.transpose() << endl;
//    cout << "com vel(Jcm * qdot)\n" << comvel_2.transpose() << endl;
//    cout << "com ang momen\n";
//        cout << "kim\n" << L.head(3).transpose() << endl;
//        cout << "dwl\n" << angmomen_dwl.transpose() << endl;
//        cout << "wensing\n" << L_wensing.head(3).transpose() << endl;
//    cout << "com lin momen\n";
//        cout << "kim\n" << L.tail(3) .transpose() << endl;
//        cout << "dwl\n" << linmomen_dwl.transpose() << endl;
//        cout << "wensing\n" << L_wensing.tail(3).transpose() << endl;
//    cout << "centroidal inertia\n";
//        cout << "kim\n" << cI<< endl;
//        cout << "dwl\n" << cI_dwl << endl;
//    cout << "Ag\n";
//        cout << "kim\n" << Ag_kim.block(0,0,6,12) << endl;
//        cout << "wensing\n" << Ag_wensing.block(0,0,6,12) << endl;

    cout << "gravity\n";
        cout << G.transpose() << endl;
}


// 欧拉角转旋转矩阵，ZYX顺序
static Eigen::Matrix<double, 3, 3> eul2Rot(double roll, double pitch, double yaw) {
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


float limit_rbdl(float x,float min,float max)
{
  if(x>max)return max;
    if(x<min)return min;
    return x;
}

int ik_rbdl(END_POS end_pos,END_POS end_att,float q_out[6])//input rad
{
    // 导入urdf模型，初始化数据容器
    //RobotModel* _model = new RobotModel();
    struct modelStateData _state = {
        VectorXd::Zero(robot::num_q),
        VectorXd::Zero(robot::num_qdot)
    };
//    printf("%f %f %f | %f %f %f\n",end_pos.x,end_pos.y,end_pos.z,end_att.x,end_att.y,end_att.z);
    double mass;
    Vec3 RPY, linkPos, linkOri, linkVel, linkAngVel;
    MatrixXd cJ, cI, cI_dwl, cenInerJac, Ag_kim, Ag_wensing, AgdotQdot_wensing, H;
    VectorXd L, comvel_2, L_wensing, qdot, G;
    Vec3 compos, comvel, linmomen_dwl;
    rbdl_Vec3 angmomen_dwl, compos_dwl, comvel_dwl;
    Mat3 World2BodyInRbdl, Body2WorldMyself;
    RPY << 0, 0, 0;
    Mat3 R = RotZ_Matrix(RPY[2]) * RotY_Matrix(RPY[1]) * RotX_Matrix(RPY[0]);
    MatrixXd R_6D = MatrixXd::Zero(6,6);
    R_6D.block(0,0,3,3) = R;
    R_6D.block(3,3,3,3) = R;
    eigenQuaternion qua(R);

    //auto l_ankle_Joint=model_biped.getJointId("arm4_joint");
    int link_end=_model_rbdl ->FindLinkId("arm4_link");
    Eigen::Matrix3d Rdes;
    Rdes<<  1,0,0,
            0,1,0,
            0,0,1; // note for the current URDF, the foot-end coordinate does NOT align with the one of the baselink
    Rdes=eul2Rot(end_att.x,end_att.y,end_att.z);
    //std::cout << Rdes << std::endl;
    Eigen::Vector3d Pdes;
    Pdes<<end_pos.x,end_pos.y,end_pos.z;  // for left leg
   // const pinocchio::SE3 oMdes(Rdes, Pdes);

    Eigen::Isometry3d  oMdes=Eigen::Isometry3d::Identity();   // T_M3是一个4x4的矩阵
    oMdes.rotate (Rdes);
    oMdes.pretranslate (Pdes);
 //   std::cout<<"oMdes: "<<std::endl<<oMdes.matrix()<<std::endl;
//    Eigen::Matrix4d T_M4 =  Eigen::Matrix4d::Identity();
//    T_M4 = T_M3.matrix();
//    std::cout<<"T_M4: "<<std::endl<<T_M4<<std::endl;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(NUM_DOF);
    q[0]=robotwb.arm_base_height;
    q[1]=robotwb.arm_q[0][0];
    q[2]=robotwb.arm_q[0][1];
    q[3]=robotwb.arm_q[0][2];
    q[4]=robotwb.arm_q[0][3];
    q[5]=robotwb.arm_q[0][4];

    const double eps  = 1e-4;
    const int IT_MAX  = 1000;
    const double DT   = 8e-1;
    const double damp = 1e-1;

    Eigen::MatrixXd J(6,NUM_DOF);
    J.setZero();

    bool success = false;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err;
    Eigen::VectorXd v(NUM_DOF);
    v.setZero(NUM_DOF);

    int itr_count{0};
    for (itr_count=0;itr_count<IT_MAX; itr_count++)
    {

        _state._q << 0, 0, 0, qua.x(), qua.y(), qua.z(),//6 for vitual
            q[0], q[1], q[2],//dof num
            q[3], q[4], q[5],
            qua.w();

        //xyz rpy
        _state._qdot << 0, 0, 0, 0, 0, 0,//6 for vitual
            0, 0, 0.,
            0, 0, 0;//dof num
        _model_rbdl -> UpdateSystem(_state._q, _state._qdot);
        //pinocchio::forwardKinematics(model_biped,data_biped,q);
        _model_rbdl ->getPos(link_end,linkPos);
        _model_rbdl ->getOri(link_end,linkOri);
        Eigen::Matrix3d Rnow;
        Eigen::Vector3d Pnow;
        Pnow[0]=linkPos[0];Pnow[1]=linkPos[1];Pnow[2]=linkPos[2];
        Rnow=eul2Rot(linkOri[0],linkOri[1],linkOri[2]);
//        Eigen::Isometry3d  oMi=Eigen::Isometry3d::Identity();   // T_M3是一个4x4的矩阵
//        oMi.rotate (Rnow);
//        oMi.pretranslate (Pnow);

        Eigen::Matrix3d RCur=Rnow;//oMcur.rotation();
        Eigen::Vector3d pCur=Pnow;//oMcur.translation();
//        cout<<"Rdes:"<<Rdes<<endl;
//        cout<<"RCur:"<<RCur<<endl;

        Eigen::Matrix3d RErr=RCur.transpose()*Rdes;
        Eigen::Vector3d pErr=RCur.transpose()*(Pdes-pCur);

        err = pin_Refined::log6(RErr,pErr);  // in joint frame
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
        Eigen::MatrixXd Jall(6,NUM_DOF);
        Jall.setZero();

        _model_rbdl ->getFullJacobian(link_end,Jall);
        Eigen::Matrix<double,6,6> rot;
        rot.setZero();
        //        cout<<Jall<<endl;
        rot.block(0,0,3,3)=RCur.transpose();
        rot.block(3,3,3,3)=RCur.transpose();
        J.block(3,0,3,6)=Jall.block(0,6,3,6);
        J.block(0,0,3,6)=Jall.block(3,6,3,6);
        //J=rot*J;
//        if(itr_count==0){
//         //cout<<Jall<<endl;
//         cout<<"rbld:"<<std::endl<<J<<endl;
//        }
//        cout<<RErr<<endl;
//        cout<<pErr<<endl;
        Eigen::Matrix<double,6,6> Jlog;
        Jlog.setZero();
        pin_Refined::Jlog6(RErr.transpose(), -RErr.transpose()*pErr, Jlog);
        J = -Jlog * J;
        //cout<<J<<endl;
        Eigen::Matrix<double,6,6> JJt;
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = - J.transpose() * JJt.ldlt().solve(err);

        q += v*DT;

        q[1] = limit_rbdl(q[1], -89.9/57.3, 89.9/57.3);//大臂
        q[2] = limit_rbdl(q[2], 0,179.9/57.3);//小臂
        q[3] = limit_rbdl(q[3], -100/57.3,100/57.3);//航向
        q[4] = limit_rbdl(q[4], -89.9/57.3,89.9/57.3);//俯仰
        q[5] = limit_rbdl(q[5], -90/57.3,90/57.3);//横滚

        //cout<<"v:"<<v<<endl;
//        if(!(itr_count % 10))
//           std::cout << itr_count << ": error = " << err.transpose() << std::endl;
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

int fk_rbdl(float q_in[6],END_POS* end_pos,END_POS* end_att)
{
    // 导入urdf模型，初始化数据容器
    //RobotModel* _model = new RobotModel();
    struct modelStateData _state = {
        VectorXd::Zero(robot::num_q),
        VectorXd::Zero(robot::num_qdot)
    };
    double mass;
    Vec3 RPY, linkPos, linkOri, linkVel, linkAngVel;
    MatrixXd cJ, cI, cI_dwl, cenInerJac, Ag_kim, Ag_wensing, AgdotQdot_wensing, H;
    VectorXd L, comvel_2, L_wensing, qdot, G;
    Vec3 compos, comvel, linmomen_dwl;
    rbdl_Vec3 angmomen_dwl, compos_dwl, comvel_dwl;
    Mat3 World2BodyInRbdl, Body2WorldMyself;
    RPY << 0, 0, 0;
    Mat3 R = RotZ_Matrix(RPY[2]) * RotY_Matrix(RPY[1]) * RotX_Matrix(RPY[0]);
    MatrixXd R_6D = MatrixXd::Zero(6,6);
    R_6D.block(0,0,3,3) = R;
    R_6D.block(3,3,3,3) = R;
    eigenQuaternion qua(R);
    _state._q << 0, 0, 0, qua.x(), qua.y(), qua.z(),//6 for vitual
        q_in[0], q_in[1], q_in[2],//dof num
        q_in[3], q_in[4], q_in[5],
        qua.w();
    //xyz rpy
    _state._qdot << 0, 0, 0, 0, 0, 0,//6 for vitual
        0, 0, 0.,
        0, 0, 0;//dof num
    _model_rbdl -> UpdateSystem(_state._q, _state._qdot);
    int link_end=_model_rbdl ->FindLinkId("arm4_link");
    _model_rbdl ->getPos(link_end,linkPos);
    end_pos->x=linkPos[0];
    end_pos->y=linkPos[1];
    end_pos->z=linkPos[2];
    _model_rbdl ->getOri(link_end,linkOri);

    Eigen::MatrixXd J(6,NUM_DOF);
    J.setZero();
    _model_rbdl ->getFullJacobian(link_end,J);
    //std::cout<<J<<std::endl;
    // 使用Eigen将旋转矩阵转换为欧拉角
    Eigen::Vector3d eulerAngle1; // ZYX顺序，yaw,pitch,roll
    eulerAngle1[0]=linkOri[2];
    eulerAngle1[1]=linkOri[1];
    eulerAngle1[2]=linkOri[0];

    if(eulerAngle1[2]>EIGEN_PI/2)
        eulerAngle1[2]=-(EIGEN_PI -eulerAngle1[2]);
    if(eulerAngle1[1]<0)
        eulerAngle1[1]=-(EIGEN_PI +eulerAngle1[1]);
    if(eulerAngle1[0]>EIGEN_PI/2)
        eulerAngle1[0]=-(EIGEN_PI -eulerAngle1[0]);
    end_att->x=eulerAngle1[2]*57.3 ;
    end_att->y=eulerAngle1[1]*57.3 ;
    end_att->z=eulerAngle1[0]*57.3 ;

    return 0;
}
#else

void rbdl_test(void) {

}


int ik_rbdl(END_POS end_pos,END_POS end_att,float q_out[6])//input rad
{

    return 0;
}

int fk_rbdl(float q_in[6],END_POS* end_pos,END_POS* end_att)
{

    return 0;
}
#endif
