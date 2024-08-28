# include "common.hpp"

void rpyToQua(double yaw, double pitch, double roll,eigenQuaternion& to){
        eigenQuaternion Qyaw(cos(0.5*yaw), 0, 0, sin(0.5*yaw) );
        eigenQuaternion Qpitch(cos(0.5*pitch), 0, sin(0.5*pitch), 0) ;
        eigenQuaternion Qroll(cos(0.5*roll), sin(0.5*roll), 0, 0);
        to = Qyaw * Qpitch * Qroll;
}
void quaToRpy(const eigenQuaternion & from, double & yaw, double & pitch, double & roll){
        eigenQuaternion Qfrom = from;
        Qfrom.normalize();
        Mat3 mat = from.toRotationMatrix();
        VectorXd ea =  mat.eulerAngles(2, 1, 0);
        yaw = ea[0];
        pitch = ea[1];
        roll = ea[2];
}
Mat3 RotX_Matrix(double roll_angle){
  Mat3 res;

  res<<1, 0, 0,
          0, cos(roll_angle), -sin(roll_angle),
          0, sin(roll_angle), cos(roll_angle);

  return res;

}
Mat3 RotY_Matrix(double pitch_angle){
    Mat3 res;

    res<<cos(pitch_angle), 0, sin(pitch_angle),
        0, 1, 0,
        -sin(pitch_angle), 0, cos(pitch_angle);

    return res;

}

Mat3 RotZ_Matrix(double yaw_angle){
    Mat3 res;

    res<<cos(yaw_angle), -sin(yaw_angle), 0,
            sin(yaw_angle), cos(yaw_angle), 0,
            0, 0, 1;

    return res;

}
Matrix<double,3,3> vecCross(Matrix<double,3,1> v)
  {
    Matrix<double,3,3> R;
    R << 0,-v[2],v[1],
         v[2],0,-v[0],
         -v[1],v[0],0;
    return R;
}

void pseudoInverse(MatrixXd const & matrix,
                       double sigmaThreshold,
                       MatrixXd & invMatrix,
                       VectorXd * opt_sigmaOut)    {
        
        if ((1 == matrix.rows()) && (1 == matrix.cols())) {
            // workaround for Eigen2
            invMatrix.resize(1, 1);
            if (matrix.coeff(0, 0) > sigmaThreshold) {
                invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
            }
            else {
                invMatrix.coeffRef(0, 0) = 0.0;
            }
            if (opt_sigmaOut) {
                opt_sigmaOut->resize(1);
                opt_sigmaOut->coeffRef(0) = matrix.coeff(0, 0);
            }
            return;
        }
      
        Eigen::JacobiSVD<MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // not sure if we need to svd.sort()... probably not
        int const nrows(svd.singularValues().rows());
        MatrixXd invS;
        invS = MatrixXd::Zero(nrows, nrows);
        for (int ii(0); ii < nrows; ++ii) {
            if (svd.singularValues().coeff(ii) > sigmaThreshold) {
                invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
            }
            else{
                // invS.coeffRef(ii, ii) = 1.0/ sigmaThreshold;
                // printf("sigular value is too small: %f\n", svd.singularValues().coeff(ii));
            }
        }
        invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();
        if (opt_sigmaOut) {
            *opt_sigmaOut = svd.singularValues();
        }
    }