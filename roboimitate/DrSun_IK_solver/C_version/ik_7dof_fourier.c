/*
 * File: ik_7dof_fourier.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 07-Oct-2023 11:12:14
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "ik_7dof_fourier.h"
#include "norm.h"

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

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
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/*
 * theta = ik_7dof_fourier(R_x,R_y,R_z,p_x,p_y,p_z,bet)
 *  [q1,q2,q3,q4,q5,q6,q7] = ik_7dof_fourier(R_x,R_y,R_z,p_x,p_y,p_z,bet)
 * Arguments    : double R_x
 *                double R_y
 *                double R_z
 *                double p_x
 *                double p_y
 *                double p_z
 *                double bet
 *                double theta[7]
 * Return Type  : void
 */
void ik_7dof_fourier(double R_x, double R_y, double R_z, double p_x, double p_y,
                     double p_z, double bet, double theta[7])
{
  double p[3];
  double n[3];
  double y;
  double b_y;
  double p_e[3];
  int i;
  double b_p_e[9];
  double Rk0x[3];
  double n4[3];
  int i0;
  double cB;
  double sB;
  double b_Rk0x[9];
  double c_Rk0x[9];
  double b_n;
  static const signed char iv0[3] = { 0, 0, 1 };

  double x;
  double theta2;
  int i1;
  double cond_BE;
  static const double b[3] = { 249.72434803198504, 0.0, 0.0 };

  double theta1;
  double dv0[16];
  double dv1[16];
  double T02[4];
  static const signed char iv1[4] = { 0, 0, 1, 0 };

  static const signed char iv2[4] = { 0, 0, 0, 1 };

  double b_T02[16];
  double theta4;
  static const signed char b_b[4] = { 40, 0, 0, 1 };

  double theta3;
  double c_T02[16];
  static const double dv2[4] = { 0.0, 0.0, -1.0, -246.5 };

  double dv3[9];
  double theta6;
  double R47[9];
  double theta5;
  double theta7;

  /*  机械臂尺寸参数 */
  /*  R07 = [1,0,0; */
  /*         0,1,0; */
  /*         0,0,1]; */
  p[0] = p_x;
  p[1] = p_y;
  p[2] = p_z;
  n[0] = p_y * -0.0 - p_z * -0.0;
  n[1] = -p_z - p_x * -0.0;
  n[2] = p_x * -0.0 - (-p_y);

  /*  零位面法向量为n */
  y = norm(n);
  b_y = norm(p);
  for (i = 0; i < 3; i++) {
    p_e[i] = p[i] / b_y;
    n[i] /= y;
  }

  /*  归一化的p */
  b_p_e[0] = p_e[0] * p_e[0] * (1.0 - cos(bet)) + cos(bet);
  b_p_e[3] = p_e[0] * p_e[1] * (1.0 - cos(bet)) - p_e[2] * sin(bet);
  b_p_e[6] = p_e[0] * p_e[2] * (1.0 - cos(bet)) + p_e[1] * sin(bet);
  b_p_e[1] = p_e[0] * p_e[1] * (1.0 - cos(bet)) + p_e[2] * sin(bet);
  b_p_e[4] = p_e[1] * p_e[1] * (1.0 - cos(bet)) + cos(bet);
  b_p_e[7] = p_e[1] * p_e[2] * (1.0 - cos(bet)) - p_e[0] * sin(bet);
  b_p_e[2] = p_e[0] * p_e[2] * (1.0 - cos(bet)) - p_e[1] * sin(bet);
  b_p_e[5] = p_e[1] * p_e[2] * (1.0 - cos(bet)) + p_e[0] * sin(bet);
  b_p_e[8] = p_e[2] * p_e[2] * (1.0 - cos(bet)) + cos(bet);
  for (i = 0; i < 3; i++) {
    n4[i] = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      n4[i] += b_p_e[i + 3 * i0] * n[i0];
    }
  }

  Rk0x[0] = p_e[0];
  Rk0x[1] = p_e[1];
  Rk0x[2] = p_e[2];
  p_e[0] = n4[0];
  p_e[1] = n4[1];
  p_e[2] = n4[2];
  n[0] = n4[1] * Rk0x[2] - n4[2] * Rk0x[1];
  n[1] = n4[2] * Rk0x[0] - n4[0] * Rk0x[2];
  n[2] = n4[0] * Rk0x[1] - n4[1] * Rk0x[0];

  /* 得到k轴方向的y在0中的描述 */
  y = norm(n);

  /* 归一化，真正得到k轴的y在0中的描述 */
  b_y = norm(p);
  cB = ((62362.249999999993 + b_y * b_y) - 58322.25) / (499.44869606397009 *
    norm(p));
  sB = sqrt(1.0 - cB * cB);
  b_p_e[0] = cB;
  b_p_e[3] = -sB;
  b_p_e[6] = 0.0;
  b_p_e[1] = sB;
  b_p_e[4] = cB;
  b_p_e[7] = 0.0;
  for (i = 0; i < 3; i++) {
    b_n = n[i] / y;
    b_Rk0x[i] = Rk0x[i];
    b_Rk0x[3 + i] = b_n;
    b_Rk0x[6 + i] = p_e[i];
    b_p_e[2 + 3 * i] = iv0[i];
    n[i] = b_n;
  }

  for (i = 0; i < 3; i++) {
    n4[i] = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      c_Rk0x[i + 3 * i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        c_Rk0x[i + 3 * i0] += b_Rk0x[i + 3 * i1] * b_p_e[i1 + 3 * i0];
      }

      n4[i] += c_Rk0x[i + 3 * i0] * b[i0];
    }
  }

  /*  E点坐标在0系下的表示, xE=pE0(1), yE=pE0(2), zE=pE0(3) */
  /*  角fai */
  y = n4[2] / 249.72434803198504;
  x = acos(y);
  theta2 = acos(y) - 0.16086957252543341;

  /*  以上求得theta2  */
  cond_BE = 40.0 * cos(theta2) + 246.5 * sin(theta2);
  if (fabs(cond_BE) > 1.0E-6) {
    theta1 = rt_atan2d_snf(n4[1] / cond_BE, n4[0] / cond_BE);
  } else {
    theta1 = -0.16086957252543341;
  }

  /*  theta1 = pi - theta1; %不知道发生了什么，这里反一下就好了 */
  /*  以上求得theta1 */
  dv0[0] = cos(theta1);
  dv0[4] = -sin(theta1);
  dv0[8] = 0.0;
  dv0[12] = 0.0;
  dv0[1] = sin(theta1);
  dv0[5] = cos(theta1);
  dv0[9] = 0.0;
  dv0[13] = 0.0;
  dv1[0] = cos(theta2);
  dv1[4] = -sin(theta2);
  dv1[8] = 0.0;
  dv1[12] = 0.0;
  dv1[2] = -sin(theta2);
  dv1[6] = -cos(theta2);
  dv1[10] = 0.0;
  dv1[14] = 0.0;
  for (i = 0; i < 4; i++) {
    dv0[2 + (i << 2)] = iv1[i];
    dv0[3 + (i << 2)] = iv2[i];
    dv1[1 + (i << 2)] = iv1[i];
    dv1[3 + (i << 2)] = iv2[i];
  }

  for (i = 0; i < 4; i++) {
    T02[i] = 0.0;
    for (i0 = 0; i0 < 4; i0++) {
      b_T02[i + (i0 << 2)] = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        b_T02[i + (i0 << 2)] += dv0[i + (i1 << 2)] * dv1[i1 + (i0 << 2)];
      }

      T02[i] += b_T02[i + (i0 << 2)] * (double)b_b[i0];
    }
  }

  for (i = 0; i < 3; i++) {
    p_e[i] = p[i] - T02[i];
    p[i] -= n4[i];
  }

  b_y = norm(p_e);
  b_y *= b_y;
  if (119084.5 - b_y > 1.0E-6) {
    theta4 = 3.1415926535897931 - acos((119084.5 - b_y) / 119059.5);
  } else {
    theta4 = 0.0;
  }

  /*  以上求得theta4 */
  n[0] = p_e[1] * p[2] - p_e[2] * p[1];
  n[1] = p_e[2] * p[0] - p_e[0] * p[2];
  n[2] = p_e[0] * p[1] - p_e[1] * p[0];
  y = norm(n);
  for (i = 0; i < 3; i++) {
    n[i] /= y;
  }

  if (sin(theta2) == 0.0) {
    theta3 = rt_atan2d_snf(-n[0], n[1]) - theta1;
  } else if ((fabs(sin(x - 0.16086957252543341)) > 1.0E-6) && (fabs(cos(theta1))
              < 1.0E-6)) {
    theta3 = rt_atan2d_snf(n[2] / sin(x - 0.16086957252543341), -(n[0] + cos
      (theta1) * cos(theta2) * n[2] / sin(x - 0.16086957252543341)) / sin(theta1));
  } else {
    theta3 = rt_atan2d_snf(n[2] / sin(x - 0.16086957252543341), (n[1] + sin
      (theta1) * cos(theta2) * n[2] / sin(x - 0.16086957252543341)) / cos(theta1));
  }

  /*  以上求得theta3 */
  dv0[0] = cos(theta3);
  dv0[4] = -sin(theta3);
  dv0[8] = 0.0;
  dv0[12] = 40.0;
  dv0[2] = sin(theta3);
  dv0[6] = cos(theta3);
  dv0[10] = 0.0;
  dv0[14] = 0.0;
  for (i = 0; i < 4; i++) {
    dv0[1 + (i << 2)] = dv2[i];
    dv0[3 + (i << 2)] = iv2[i];
  }

  dv1[0] = cos(theta4);
  dv1[4] = -sin(theta4);
  dv1[8] = 0.0;
  dv1[12] = 0.0;
  dv1[2] = -sin(theta4);
  dv1[6] = -cos(theta4);
  dv1[10] = 0.0;
  dv1[14] = 0.0;
  for (i = 0; i < 4; i++) {
    for (i0 = 0; i0 < 4; i0++) {
      c_T02[i + (i0 << 2)] = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        c_T02[i + (i0 << 2)] += b_T02[i + (i1 << 2)] * dv0[i1 + (i0 << 2)];
      }
    }

    dv1[1 + (i << 2)] = iv1[i];
    dv1[3 + (i << 2)] = iv2[i];
  }

  for (i = 0; i < 4; i++) {
    for (i0 = 0; i0 < 4; i0++) {
      b_T02[i + (i0 << 2)] = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        b_T02[i + (i0 << 2)] += c_T02[i0 + (i1 << 2)] * dv1[i1 + (i << 2)];
      }
    }
  }

  dv3[0] = cos(R_x) * cos(R_y);
  dv3[3] = cos(R_x) * sin(R_y) * sin(R_z) - sin(R_x) * cos(R_z);
  dv3[6] = cos(R_x) * sin(R_y) * cos(R_z) + sin(R_x) * sin(R_z);
  dv3[1] = sin(R_x) * cos(R_y);
  dv3[4] = sin(R_x) * sin(R_y) * sin(R_z) + cos(R_x) * cos(R_z);
  dv3[7] = sin(R_x) * sin(R_y) * cos(R_z) - cos(R_x) * sin(R_z);
  dv3[2] = -sin(R_y);
  dv3[5] = cos(R_y) * sin(R_z);
  dv3[8] = cos(R_y) * cos(R_z);
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      R47[i + 3 * i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        R47[i + 3 * i0] += b_T02[i + (i1 << 2)] * dv3[i1 + 3 * i0];
      }
    }
  }

  theta6 = acos(-R47[7]);

  /*  以上求得theta6 */
  if (fabs(theta6) > 1.0E-6) {
    theta5 = rt_atan2d_snf(R47[8] / sin(theta6), R47[6] / sin(theta6));
    theta7 = rt_atan2d_snf(-R47[4] / sin(theta6), R47[1] / sin(theta6));
  } else {
    theta5 = 0.0;

    /*  真实机器人赋予上一时刻值 */
    theta7 = rt_atan2d_snf(-R47[3], R47[5]);
  }

  theta[0] = theta1;
  theta[1] = x - 0.16086957252543341;
  theta[2] = theta3;
  theta[3] = theta4;
  theta[4] = theta5;
  theta[5] = theta6;
  theta[6] = theta7;

  /*  [q1,q2,q3,q4,q5,q6,q7] = [theta1,theta2,theta3,theta4,theta5,theta6,theta7]; */
  /*  q1 = theta1; */
  /*  q2 = theta2; */
  /*  q3 = theta3; */
  /*  q4 = theta4; */
  /*  q5 = theta5; */
  /*  q6 = theta6; */
  /*  q7 = theta7; */
  /*  theta = theta*(180/pi); */
  /*  整个函数是end */
}

/*
 * File trailer for ik_7dof_fourier.c
 *
 * [EOF]
 */
