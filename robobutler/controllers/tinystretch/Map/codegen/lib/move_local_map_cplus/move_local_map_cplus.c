/*
 * File: move_local_map_cplus.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 16-Dec-2021 16:11:24
 */

/* Include Files */
#include "move_local_map_cplus.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Variable Definitions */
static boolean_T isInitialized_move_local_map_cplus = false;

/* Function Declarations */
static double b_mod(double x, double y);
static double rt_roundd_snf(double u);

/* Function Definitions */

/*
 * Arguments    : double x
 *                double y
 * Return Type  : double
 */
static double b_mod(double x, double y)
{
  double r;
  boolean_T rEQ0;
  double q;
  r = x;
  if (y == 0.0) {
    if (x == 0.0) {
      r = y;
    }
  } else if (rtIsNaN(x) || rtIsNaN(y) || rtIsInf(x)) {
    r = rtNaN;
  } else if (x == 0.0) {
    r = 0.0 / y;
  } else if (rtIsInf(y)) {
    if ((y < 0.0) != (x < 0.0)) {
      r = y;
    }
  } else {
    r = fmod(x, y);
    rEQ0 = (r == 0.0);
    if ((!rEQ0) && (y > floor(y))) {
      q = fabs(x / y);
      rEQ0 = !(fabs(q - floor(q + 0.5)) > 2.2204460492503131E-16 * q);
    }

    if (rEQ0) {
      r = y * 0.0;
    } else {
      if ((x < 0.0) != (y < 0.0)) {
        r += y;
      }
    }
  }

  return r;
}

/*
 * Arguments    : double u
 * Return Type  : double
 */
static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/*
 * Arguments    : const double cog_n[3]
 *                const double map_n[1000000]
 *                const double map_param[5]
 *                double map_local[1600]
 * Return Type  : void
 */
void move_local_map_cplus(const double cog_n[3], const double map_n[1000000],
  const double map_param[5], double map_local[1600])
{
  double MAP_SIZE;
  double pos_grid_n_idx_0_tmp;
  double d;
  int i;
  double pos_grid_n_idx_0;
  double d1;
  short b_i;
  short i1;
  int j;
  int i2;
  short i3;
  short i4;
  int i5;
  if (!isInitialized_move_local_map_cplus) {
    move_local_map_cplus_initialize();
  }

  /* MAP_W=map_param(2); */
  /* MAP_H=map_param(1); */
  MAP_SIZE = map_param[4];
  pos_grid_n_idx_0_tmp = 40.0 * MAP_SIZE / 2.0;
  d = map_param[4];
  for (i = 0; i < 40; i++) {
    /* 获取全局地图 */
    pos_grid_n_idx_0 = ((((double)i + 1.0) * MAP_SIZE - pos_grid_n_idx_0_tmp) +
                        cog_n[0]) + MAP_SIZE / 2.0;
    d1 = rt_roundd_snf(pos_grid_n_idx_0 / d);
    if (d1 < 32768.0) {
      if (d1 >= -32768.0) {
        b_i = (short)d1;
      } else {
        b_i = MIN_int16_T;
      }
    } else if (d1 >= 32768.0) {
      b_i = MAX_int16_T;
    } else {
      b_i = 0;
    }

    d1 = rt_roundd_snf(b_mod(pos_grid_n_idx_0, d));
    if (d1 < 32768.0) {
      if (d1 >= -32768.0) {
        i1 = (short)d1;
      } else {
        i1 = MIN_int16_T;
      }
    } else if (d1 >= 32768.0) {
      i1 = MAX_int16_T;
    } else {
      i1 = 0;
    }

    for (j = 0; j < 40; j++) {
      pos_grid_n_idx_0 = ((((double)j + 1.0) * MAP_SIZE - pos_grid_n_idx_0_tmp)
                          + cog_n[1]) + MAP_SIZE / 2.0;

      /* 查找一个坐标在全局地图上的位置 */
      /* Gx=limit(Gx,1,MAP_H_N); */
      /* Gy=limit(Gy,1,MAP_W_N); */
      i2 = b_i + i1;
      if (i2 > 32767) {
        i2 = 32767;
      } else {
        if (i2 < -32768) {
          i2 = -32768;
        }
      }

      d1 = rt_roundd_snf(pos_grid_n_idx_0 / d);
      if (d1 < 32768.0) {
        if (d1 >= -32768.0) {
          i3 = (short)d1;
        } else {
          i3 = MIN_int16_T;
        }
      } else if (d1 >= 32768.0) {
        i3 = MAX_int16_T;
      } else {
        i3 = 0;
      }

      d1 = rt_roundd_snf(b_mod(pos_grid_n_idx_0, d));
      if (d1 < 32768.0) {
        if (d1 >= -32768.0) {
          i4 = (short)d1;
        } else {
          i4 = MIN_int16_T;
        }
      } else if (d1 >= 32768.0) {
        i4 = MAX_int16_T;
      } else {
        i4 = 0;
      }

      i5 = i3 + i4;
      if (i5 > 32767) {
        i5 = 32767;
      } else {
        if (i5 < -32768) {
          i5 = -32768;
        }
      }

      map_local[i + 40 * j] = map_n[(i2 + 1000 * (i5 - 1)) - 1] + 0.001;
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void move_local_map_cplus_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_move_local_map_cplus = true;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void move_local_map_cplus_terminate(void)
{
  /* (no terminate code required) */
  isInitialized_move_local_map_cplus = false;
}

/*
 * File trailer for move_local_map_cplus.c
 *
 * [EOF]
 */
