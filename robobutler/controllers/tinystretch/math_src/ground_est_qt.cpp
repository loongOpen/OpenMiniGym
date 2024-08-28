/*
 * File: ground_att_est_n.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 30-Aug-2020 14:04:06
 */

 /* Include Files */
#include "ground_att_est_n.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>
#include   <stdlib.h>
#include   <string.h>

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

void invg(const double x[9], double y[9])
{
    double b_x[9];
    int p1;
    int p2;
    int p3;
    double absx11;
    double absx21;
    double absx31;
    int itmp;
    memcpy(&b_x[0], &x[0], 9U * sizeof(double));
    p1 = 0;
    p2 = 3;
    p3 = 6;
    absx11 = fabs(x[0]);
    absx21 = fabs(x[1]);
    absx31 = fabs(x[2]);
    if ((absx21 > absx11) && (absx21 > absx31)) {
        p1 = 3;
        p2 = 0;
        b_x[0] = x[1];
        b_x[1] = x[0];
        b_x[3] = x[4];
        b_x[4] = x[3];
        b_x[6] = x[7];
        b_x[7] = x[6];
    }
    else {
        if (absx31 > absx11) {
            p1 = 6;
            p3 = 0;
            b_x[0] = x[2];
            b_x[2] = x[0];
            b_x[3] = x[5];
            b_x[5] = x[3];
            b_x[6] = x[8];
            b_x[8] = x[6];
        }
    }

    b_x[1] /= b_x[0];
    b_x[2] /= b_x[0];
    b_x[4] -= b_x[1] * b_x[3];
    b_x[5] -= b_x[2] * b_x[3];
    b_x[7] -= b_x[1] * b_x[6];
    b_x[8] -= b_x[2] * b_x[6];
    if (fabs(b_x[5]) > fabs(b_x[4])) {
        itmp = p2;
        p2 = p3;
        p3 = itmp;
        absx11 = b_x[1];
        b_x[1] = b_x[2];
        b_x[2] = absx11;
        absx11 = b_x[4];
        b_x[4] = b_x[5];
        b_x[5] = absx11;
        absx11 = b_x[7];
        b_x[7] = b_x[8];
        b_x[8] = absx11;
    }

    b_x[5] /= b_x[4];
    b_x[8] -= b_x[5] * b_x[7];
    absx11 = (b_x[5] * b_x[1] - b_x[2]) / b_x[8];
    absx21 = -(b_x[1] + b_x[7] * absx11) / b_x[4];
    y[p1] = ((1.0 - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
    y[p1 + 1] = absx21;
    y[p1 + 2] = absx11;
    absx11 = -b_x[5] / b_x[8];
    absx21 = (1.0 - b_x[7] * absx11) / b_x[4];
    y[p2] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
    y[p2 + 1] = absx21;
    y[p2 + 2] = absx11;
    absx11 = 1.0 / b_x[8];
    absx21 = -b_x[7] * absx11 / b_x[4];
    y[p3] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
    y[p3 + 1] = absx21;
    y[p3 + 2] = absx11;
}


/* Function Definitions */

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
    double y;
    int b_u0;
    int b_u1;
    if (rtIsNaN(u0) || rtIsNaN(u1)) {
        y = rtNaN;
    }
    else if (rtIsInf(u0) && rtIsInf(u1)) {
        if (u0 > 0.0) {
            b_u0 = 1;
        }
        else {
            b_u0 = -1;
        }

        if (u1 > 0.0) {
            b_u1 = 1;
        }
        else {
            b_u1 = -1;
        }

        y = atan2(b_u0, b_u1);
    }
    else if (u1 == 0.0) {
        if (u0 > 0.0) {
            y = RT_PI / 2.0;
        }
        else if (u0 < 0.0) {
            y = -(RT_PI / 2.0);
        }
        else {
            y = 0.0;
        }
    }
    else {
        y = atan2(u0, u1);
    }

    return y;
}

/*
 * Roll Pitch Yaw
 *      ��+���Ӽ�ͷ���￴��
 *       z   y ��+
 *       |  /
 *       | /
 *       |/
 *       o����������x ��+
 * 1FL   2FR
 * Arguments    : const char G[4]
 *                const double P1n[3]
 *                const double P2n[3]
 *                const double P3n[3]
 *                const double P4n[3]
 *                const double ATT_OFF[3]
 *                double ATT_G[3]
 *                double *flag
 * Return Type  : void
 */
void ground_att_est_n(const char G[4], const double P1n[3], const double P2n[3],
    const double P3n[3], const double P4n[3], const double
    ATT_OFF[3], double Norm[3], double Plan_Param[3], double
    ATT_G[3], double *flag)
{
    double Gnum;
    double P1_idx_0;
    double P2_idx_0;
    double P3_idx_0;
    double P1_idx_1;
    double P2_idx_1;
    double P3_idx_1;
    double P1_idx_2;
    double P2_idx_2;
    double P3_idx_2;
    double XCoeff;
    double YCoeff;
    double CCoeff;
    double A[12];
    double b_A[9];
    int i;
    double y_tmp[12];
    double b_y_tmp[9];
    double b_P1n[4];
    int i1;
    double c_y_tmp[9];
    int i2;
    double P1[3];
    double Cc[3];


    Gnum = ((G[0] + G[1]) + G[2]) + G[3];

    /* -------------------------------A1-线性回归拟合平面----------------------------------- */
    /* ax + by + c = z */
    P1_idx_0 = 0.0;
    P2_idx_0 = 0.0;
    P3_idx_0 = 0.0;
    P1_idx_1 = 0.0;
    P2_idx_1 = 0.0;
    P3_idx_1 = 0.0;
    P1_idx_2 = 0.0;
    P2_idx_2 = 0.0;
    P3_idx_2 = 0.0;
    XCoeff = 0.0;
    YCoeff = 0.0;
    CCoeff = 0.0;
    if (Gnum == 3.0) {
        if (G[0] == 0.0) {
            P1_idx_0 = P2n[0];
            P2_idx_0 = P3n[0];
            P3_idx_0 = P4n[0];
            P1_idx_1 = P2n[1];
            P2_idx_1 = P3n[1];
            P3_idx_1 = P4n[1];
            P1_idx_2 = P2n[2];
            P2_idx_2 = P3n[2];
            P3_idx_2 = P4n[2];
        }

        if (G[1] == 0.0) {
            P1_idx_0 = P1n[0];
            P2_idx_0 = P3n[0];
            P3_idx_0 = P4n[0];
            P1_idx_1 = P1n[1];
            P2_idx_1 = P3n[1];
            P3_idx_1 = P4n[1];
            P1_idx_2 = P1n[2];
            P2_idx_2 = P3n[2];
            P3_idx_2 = P4n[2];
        }

        if (G[2] == 0.0) {
            P1_idx_0 = P2n[0];
            P2_idx_0 = P1n[0];
            P3_idx_0 = P4n[0];
            P1_idx_1 = P2n[1];
            P2_idx_1 = P1n[1];
            P3_idx_1 = P4n[1];
            P1_idx_2 = P2n[2];
            P2_idx_2 = P1n[2];
            P3_idx_2 = P4n[2];
        }

        if (G[3] == 0.0) {
            P1_idx_0 = P2n[0];
            P2_idx_0 = P3n[0];
            P3_idx_0 = P1n[0];
            P1_idx_1 = P2n[1];
            P2_idx_1 = P3n[1];
            P3_idx_1 = P1n[1];
            P1_idx_2 = P2n[2];
            P2_idx_2 = P3n[2];
            P3_idx_2 = P1n[2];
        }

        b_A[6] = 1.0;
        b_A[7] = 1.0;
        b_A[0] = P1_idx_0;
        b_A[1] = P2_idx_0;
        b_A[2] = P3_idx_0;
        b_A[3] = P1_idx_1;
        b_A[4] = P2_idx_1;
        b_A[5] = P3_idx_1;
        b_A[8] = 1.0;
        for (i = 0; i < 3; i++) {
            b_y_tmp[3 * i] = b_A[i];
            b_y_tmp[3 * i + 1] = b_A[i + 3];
            b_y_tmp[3 * i + 2] = 1.0;
        }

        for (i = 0; i < 3; i++) {
            P2_idx_0 = b_y_tmp[i + 3];
            P3_idx_0 = b_y_tmp[i + 6];
            for (i1 = 0; i1 < 3; i1++) {
                c_y_tmp[i + 3 * i1] = (b_y_tmp[i] * b_A[3 * i1] + P2_idx_0 * b_A[3 * i1
                    + 1]) + P3_idx_0 * b_A[3 * i1 + 2];
            }
        }

        invg(c_y_tmp, b_A);
        P1[0] = P1_idx_2;
        P1[1] = P2_idx_2;
        P1[2] = P3_idx_2;
        for (i = 0; i < 3; i++) {
            P2_idx_0 = 0.0;
            P3_idx_0 = b_A[i + 3];
            P1_idx_0 = b_A[i + 6];
            for (i1 = 0; i1 < 3; i1++) {
                P2_idx_0 += ((b_A[i] * b_y_tmp[3 * i1] + P3_idx_0 * b_y_tmp[3 * i1 + 1])
                    + P1_idx_0 * b_y_tmp[3 * i1 + 2]) * P1[i1];
            }

            Cc[i] = P2_idx_0;
        }

        XCoeff = Cc[0];
        YCoeff = Cc[1];
        CCoeff = Cc[2];
    }
    else {
        if (Gnum == 4.0) {
            A[8] = 1.0;
            A[9] = 1.0;
            A[10] = 1.0;
            A[0] = P1n[0];
            A[1] = P2n[0];
            A[2] = P3n[0];
            A[3] = P4n[0];
            A[4] = P1n[1];
            A[5] = P2n[1];
            A[6] = P3n[1];
            A[7] = P4n[1];
            A[11] = 1.0;
            for (i = 0; i < 4; i++) {
                y_tmp[3 * i] = A[i];
                y_tmp[3 * i + 1] = A[i + 4];
                y_tmp[3 * i + 2] = A[i + 8];
            }

            for (i = 0; i < 3; i++) {
                P2_idx_0 = y_tmp[i + 3];
                P3_idx_0 = y_tmp[i + 6];
                P1_idx_0 = y_tmp[i + 9];
                for (i1 = 0; i1 < 3; i1++) {
                    i2 = i1 << 2;
                    b_y_tmp[i + 3 * i1] = ((y_tmp[i] * A[i2] + P2_idx_0 * A[i2 + 1]) +
                        P3_idx_0 * A[i2 + 2]) + P1_idx_0 * A[i2 + 3];
                }
            }

            invg(b_y_tmp, b_A);
            b_P1n[0] = P1n[2];
            b_P1n[1] = P2n[2];
            b_P1n[2] = P3n[2];
            b_P1n[3] = P4n[2];
            for (i = 0; i < 3; i++) {
                P2_idx_0 = 0.0;
                P3_idx_0 = b_A[i + 3];
                P1_idx_0 = b_A[i + 6];
                for (i1 = 0; i1 < 4; i1++) {
                    P2_idx_0 += ((b_A[i] * y_tmp[3 * i1] + P3_idx_0 * y_tmp[3 * i1 + 1]) +
                        P1_idx_0 * y_tmp[3 * i1 + 2]) * b_P1n[i1];
                }

                Cc[i] = P2_idx_0;
            }

            XCoeff = Cc[0];
            YCoeff = Cc[1];
            CCoeff = Cc[2];
        }
    }

    if (Gnum > 0.0) {
        *flag = 1.0;

        /* 地形平面法向量 */
        P1_idx_0 = sqrt((XCoeff * XCoeff + YCoeff * YCoeff) + 1.0);
        Norm[0] = -XCoeff / P1_idx_0;
        Norm[1] = -YCoeff / P1_idx_0;
        Norm[2] = 1.0 / P1_idx_0;
        Plan_Param[0] = XCoeff;
        Plan_Param[1] = YCoeff;
        Plan_Param[2] = CCoeff;

        /* 向量起bai点du */
        /* 向量终点zhi */
        /* 计算地形角度 */
        ATT_G[1] = rt_atan2d_snf(Norm[1], Norm[2]) * 57.3 + ATT_OFF[1];
        ATT_G[0] = rt_atan2d_snf(Norm[0], Norm[2]) * 57.3 + ATT_OFF[0];
        ATT_G[2] = ATT_OFF[2];
    }
    else {
        ATT_G[1] = 0.0;
        ATT_G[0] = 0.0;
        ATT_G[2] = 0.0;
        Norm[0] = 0.0;
        Plan_Param[0] = 0.0;
        Norm[1] = 0.0;
        Plan_Param[1] = 0.0;
        Norm[2] = 1.0;
        Plan_Param[2] = 0.0;
        *flag = 0.0;
    }
}

/*
 * File trailer for ground_att_est_n.c
 *
 * [EOF]
 */
