//
// Created by boxing on 24-2-21.
//

#ifndef INVKINE_TEST_LOG_PINREFINED_H
#define INVKINE_TEST_LOG_PINREFINED_H
#include "Eigen/Dense"
namespace pin_Refined
{

    Eigen::Vector3d log3(const Eigen::Matrix3d & R, double theta);
    void Jlog3(const double & theta, const Eigen::Vector3d & log, Eigen::Matrix3d & Jlog);

    Eigen::Matrix<double,6,1> log6(const Eigen::Matrix3d &Rot, const Eigen::Vector3d &Trans);
    void Jlog6(const Eigen::Matrix3d & R,  const Eigen::Vector3d & Trans, Eigen::Matrix<double,6,6> & Jlog);

    template<typename Scalar>
    struct TaylorSeriesExpansion {
        ///
        /// \brief Computes the expected tolerance of the argument of a Taylor series expansion for a certain degree
        ///        according to the machine precision of the given input Scalar.
        ///
        /// \tparam degree the degree of the Taylor series expansion.
        ///
        template<int degree>
        static Scalar precision() {
            static Scalar value = pow(std::numeric_limits<Scalar>::epsilon(), Scalar(1) / Scalar(degree + 1));
            return value;
        };
    };

    void addSkew(const Eigen::Vector3d & v, Eigen::Matrix3d & M);

}

#endif //INVKINE_TEST_LOG_PINREFINED_H
