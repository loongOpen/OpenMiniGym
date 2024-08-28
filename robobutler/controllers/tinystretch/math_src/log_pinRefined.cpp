//
// Created by boxing on 24-2-21.
//
#include "log_pinRefined.h"

namespace pin_Refined {
    Eigen::Vector3d log3(const Eigen::Matrix3d &R, double theta) {
        Eigen::Vector3d res_;
        res_.setZero();
        Eigen::Vector3d Vector3;

        double PI_value = 3.1415926;
        typedef double Scalar;

        Scalar tr = R.trace();
        if (tr >= Scalar(3)) {
            tr = Scalar(3); // clip value
            theta = Scalar(0); // acos((3-1)/2)
        } else if (tr <= Scalar(-1)) {
            tr = Scalar(-1); // clip value
            theta = PI_value; // acos((-1-1)/2)
        } else
            theta = acos((tr - Scalar(1)) / Scalar(2));
        assert(theta == theta && "theta contains some NaN"); // theta != NaN

        // From runs of hpp-constraints/tests/logarithm.cc: 1e-6 is too small.
        if (theta >= PI_value - 1e-2) {
            // 1e-2: A low value is not required since the computation is
            // using explicit formula. However, the precision of this method
            // is the square root of the precision with the antisymmetric
            // method (Nominal case).
            const Scalar cphi = -(tr - Scalar(1)) / Scalar(2);
            const Scalar beta = theta * theta / (Scalar(1) + cphi);
            const Eigen::Vector3d tmp((R.diagonal().array() + cphi) * beta);
            res_(0) = (R(2, 1) > R(1, 2) ? Scalar(1) : Scalar(-1)) * (tmp[0] > Scalar(0) ? sqrt(tmp[0]) : Scalar(0));
            res_(1) = (R(0, 2) > R(2, 0) ? Scalar(1) : Scalar(-1)) * (tmp[1] > Scalar(0) ? sqrt(tmp[1]) : Scalar(0));
            res_(2) = (R(1, 0) > R(0, 1) ? Scalar(1) : Scalar(-1)) * (tmp[2] > Scalar(0) ? sqrt(tmp[2]) : Scalar(0));
        } else {
            const Scalar t = ((theta > TaylorSeriesExpansion<Scalar>::template precision<3>())
                              ? theta / sin(theta)
                              : Scalar(1)) / Scalar(2);
            res_(0) = t * (R(2, 1) - R(1, 2));
            res_(1) = t * (R(0, 2) - R(2, 0));
            res_(2) = t * (R(1, 0) - R(0, 1));
        }
        return  res_;
    }

    /// \brief Generic evaluation of Jlog3 function
    void Jlog3(const double &theta, const Eigen::Vector3d &log, Eigen::Matrix3d &Jlog) {
        typedef double Scalar;

        Scalar alpha, diag_value;
        if (theta < TaylorSeriesExpansion<Scalar>::template precision<3>()) {
            alpha = Scalar(1) / Scalar(12) + theta * theta / Scalar(720);
            diag_value = Scalar(0.5) * (2 - theta * theta / Scalar(6));
        } else {
            // Jlog = alpha I
            Scalar ct, st;
            st = sin(theta);
            ct = cos(theta);
            const Scalar st_1mct = st / (Scalar(1) - ct);

            alpha = Scalar(1) / (theta * theta) - st_1mct / (Scalar(2) * theta);
            diag_value = Scalar(0.5) * (theta * st_1mct);
        }

        Jlog.noalias() = alpha * log * log.transpose();
        Jlog.diagonal().array() += diag_value;

        // Jlog += r_{\times}/2
        addSkew(Scalar(0.5) * log, Jlog);
    }

    /// \brief Generic evaluation of log6 function
    // return: [linear;angular]
    Eigen::Matrix<double,6,1> log6(const Eigen::Matrix3d &Rot, const Eigen::Vector3d &Trans)  {
        typedef double Scalar;
        Eigen::Matrix<double,6,1> res;
        res.setZero();

        Scalar t;
        Eigen::Vector3d w(log3(Rot, t)); // t in [0,¦Ð]
        const Scalar t2 = t * t;
        Scalar alpha, beta;
        Eigen::Vector3d lin, ang;

        if (t < TaylorSeriesExpansion<Scalar>::template precision<3>()) {
            alpha = Scalar(1) - t2 / Scalar(12) - t2 * t2 / Scalar(720);
            beta = Scalar(1) / Scalar(12) + t2 / Scalar(720);
        } else {
            Scalar st, ct;
            st=sin(t);
            ct=cos(t);
            alpha = t * st / (Scalar(2) * (Scalar(1) - ct));
            beta = Scalar(1) / t2 - st / (Scalar(2) * t * (Scalar(1) - ct));
        }

        res.block<3,1>(0,0).noalias() = alpha * Trans - Scalar(0.5) * w.cross(Trans) + (beta * w.dot(Trans)) * w;
        res.block<3,1>(3,0) = w;

        return res;
    }

    void Jlog6(const Eigen::Matrix3d &R, const Eigen::Vector3d &Trans, Eigen::Matrix<double, 6, 6> &Jlog) {
        typedef double Scalar;

        Scalar t;
        Eigen::Vector3d w(log3(R, t));

        // value is decomposed as following:
        // value = [ A, B;
        //           C, D ]
        Eigen::Matrix3d A = Jlog.topLeftCorner<3, 3>();
        Eigen::Matrix3d B = Jlog.topRightCorner<3, 3>();
        Eigen::Matrix3d C = Jlog.bottomLeftCorner<3, 3>();
        Eigen::Matrix3d D = Jlog.bottomRightCorner<3, 3>();

        Jlog3(t, w, A);
        D = A;

        const Scalar t2 = t * t;
        Scalar beta, beta_dot_over_theta;
        if (t < TaylorSeriesExpansion<Scalar>::template precision<3>()) {
            beta = Scalar(1) / Scalar(12) + t2 / Scalar(720);
            beta_dot_over_theta = Scalar(1) / Scalar(360);
        } else {
            const Scalar tinv = Scalar(1) / t,
                    t2inv = tinv * tinv;
            Scalar st, ct;
            st=sin(t);
            ct=cos(t);
            const Scalar inv_2_2ct = Scalar(1) / (Scalar(2) * (Scalar(1) - ct));

            beta = t2inv - st * tinv * inv_2_2ct;
            beta_dot_over_theta = -Scalar(2) * t2inv * t2inv +
                                  (Scalar(1) + st * tinv) * t2inv * inv_2_2ct;
        }

        Scalar wTp = w.dot(Trans);

        Eigen::Vector3d v3_tmp((beta_dot_over_theta * wTp) * w - (t2 * beta_dot_over_theta + Scalar(2) * beta) * Trans);
        // C can be treated as a temporary variable
        C.noalias() = v3_tmp * w.transpose();

        C.noalias() += beta * w * Trans.transpose();
        C.diagonal().array() += wTp * beta;
        addSkew(Scalar(.5)* Trans, C);
        B.noalias() = C * A;
        C. setZero();

        Jlog.topLeftCorner<3, 3>()=A;
        Jlog.topRightCorner<3, 3>()=B;
        Jlog.bottomLeftCorner<3, 3>()=C;
        Jlog.bottomRightCorner<3, 3>()=D;
    }

    void addSkew(const Eigen::Vector3d & v, Eigen::Matrix3d & M)
    {
        M(0,1) -= v[2];      M(0,2) += v[1];
        M(1,0) += v[2];      M(1,2) -= v[0];
        M(2,0) -= v[1];      M(2,1) += v[0];
    }
}