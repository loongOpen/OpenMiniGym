/*
 * File: foot_good_foot_found.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 22-Dec-2021 17:35:42
 */

/* Include Files */
#include "foot_good_foot_found.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Variable Definitions */
static boolean_T isInitialized_foot_good_foot_found = false;

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
 * Arguments    : short leg_sel
 *                double spd_fb
 *                const double now_foot[3]
 *                const double slip_foot[3]
 *                const double map_local_foot[1600]
 *                const double map_param[5]
 *                const double check_param[3]
 *                const double robot_param[3]
 *                double gfoot[3]
 *                double *is_found
 * Return Type  : void
 */
void foot_good_foot_found(short leg_sel, double spd_fb, const double now_foot[3],
  const double slip_foot[3], const double map_local_foot[1600], const double
  map_param[5], const double check_param[3], const double robot_param[3], double
  gfoot[3], double *is_found)
{
  double d;
  short dis;
  int b_dis;
  short Hw_id_2;
  short r_L_id_2;
  short dis_min;
  short Gx_foot_now;
  int good_foot_id;
  short Gy_foot_now;
  short Gx_slip_now;
  short Gy_slip_now;
  int good_foot_cnt1;
  int i;
  int good_foot_id_f;
  int good_foot_cnt2;
  int good_foot_id_b;
  int good_foot_cnt3;
  if (!isInitialized_foot_good_foot_found) {
    foot_good_foot_found_initialize();
  }

  gfoot[0] = 0.0;
  gfoot[1] = 0.0;
  gfoot[2] = 0.0;

  /* map_param(1); */
  /* map_param(2); */
  if (leg_sel == 0) {
    d = rt_roundd_snf(robot_param[0] / 2.0 / map_param[4]);
    if (d < 32768.0) {
      if (d >= -32768.0) {
        dis = (short)d;
      } else {
        dis = MIN_int16_T;
      }
    } else if (d >= 32768.0) {
      dis = MAX_int16_T;
    } else {
      dis = 0;
    }

    b_dis = dis + 20;
    if (b_dis > 32767) {
      b_dis = 32767;
    }

    Hw_id_2 = (short)b_dis;
    d = rt_roundd_snf(robot_param[2] / map_param[4] / 2.0);
    if (d < 32768.0) {
      if (d >= -32768.0) {
        r_L_id_2 = (short)d;
      } else {
        r_L_id_2 = MIN_int16_T;
      }
    } else if (d >= 32768.0) {
      r_L_id_2 = MAX_int16_T;
    } else {
      r_L_id_2 = 0;
    }
  } else if (leg_sel == 1) {
    d = rt_roundd_snf(robot_param[0] / 2.0 / map_param[4]);
    if (d < 32768.0) {
      if (d >= -32768.0) {
        dis = (short)d;
      } else {
        dis = MIN_int16_T;
      }
    } else if (d >= 32768.0) {
      dis = MAX_int16_T;
    } else {
      dis = 0;
    }

    b_dis = -dis;
    if (b_dis > 32767) {
      b_dis = 32767;
    }

    b_dis += 20;
    if (b_dis > 32767) {
      b_dis = 32767;
    }

    Hw_id_2 = (short)b_dis;
    d = rt_roundd_snf(robot_param[2] / map_param[4] / 2.0);
    if (d < 32768.0) {
      if (d >= -32768.0) {
        r_L_id_2 = (short)d;
      } else {
        r_L_id_2 = MIN_int16_T;
      }
    } else if (d >= 32768.0) {
      r_L_id_2 = MAX_int16_T;
    } else {
      r_L_id_2 = 0;
    }
  } else if (leg_sel == 2) {
    d = rt_roundd_snf(robot_param[0] / 2.0 / map_param[4]);
    if (d < 32768.0) {
      if (d >= -32768.0) {
        dis = (short)d;
      } else {
        dis = MIN_int16_T;
      }
    } else if (d >= 32768.0) {
      dis = MAX_int16_T;
    } else {
      dis = 0;
    }

    b_dis = dis + 20;
    if (b_dis > 32767) {
      b_dis = 32767;
    }

    Hw_id_2 = (short)b_dis;
    d = rt_roundd_snf(robot_param[2] / map_param[4] / 2.0);
    if (d < 32768.0) {
      if (d >= -32768.0) {
        r_L_id_2 = (short)d;
      } else {
        r_L_id_2 = MIN_int16_T;
      }
    } else if (d >= 32768.0) {
      r_L_id_2 = MAX_int16_T;
    } else {
      r_L_id_2 = 0;
    }
  } else {
    /* if leg_sel==3 */
    d = rt_roundd_snf(robot_param[0] / 2.0 / map_param[4]);
    if (d < 32768.0) {
      if (d >= -32768.0) {
        dis = (short)d;
      } else {
        dis = MIN_int16_T;
      }
    } else if (d >= 32768.0) {
      dis = MAX_int16_T;
    } else {
      dis = 0;
    }

    b_dis = -dis;
    if (b_dis > 32767) {
      b_dis = 32767;
    }

    b_dis += 20;
    if (b_dis > 32767) {
      b_dis = 32767;
    }

    Hw_id_2 = (short)b_dis;
    d = rt_roundd_snf(robot_param[2] / map_param[4] / 2.0);
    if (d < 32768.0) {
      if (d >= -32768.0) {
        r_L_id_2 = (short)d;
      } else {
        r_L_id_2 = MIN_int16_T;
      }
    } else if (d >= 32768.0) {
      r_L_id_2 = MAX_int16_T;
    } else {
      r_L_id_2 = 0;
    }
  }

  *is_found = 0.0;

  /* 绘制矢量方向线 */
  /* 查找一个坐标在全局地图上的位置 */
  /* map_param(1); */
  /* map_param(2); */
  d = rt_roundd_snf(now_foot[0] / map_param[4]);
  if (d < 32768.0) {
    if (d >= -32768.0) {
      dis = (short)d;
    } else {
      dis = MIN_int16_T;
    }
  } else if (d >= 32768.0) {
    dis = MAX_int16_T;
  } else {
    dis = 0;
  }

  d = rt_roundd_snf(b_mod(now_foot[0], map_param[4]));
  if (d < 32768.0) {
    if (d >= -32768.0) {
      dis_min = (short)d;
    } else {
      dis_min = MIN_int16_T;
    }
  } else if (d >= 32768.0) {
    dis_min = MAX_int16_T;
  } else {
    dis_min = 0;
  }

  b_dis = dis + dis_min;
  if (b_dis > 32767) {
    b_dis = 32767;
  } else {
    if (b_dis < -32768) {
      b_dis = -32768;
    }
  }

  b_dis += 20;
  if (b_dis > 32767) {
    b_dis = 32767;
  }

  Gx_foot_now = (short)b_dis;
  d = rt_roundd_snf(now_foot[1] / map_param[4]);
  if (d < 32768.0) {
    if (d >= -32768.0) {
      dis = (short)d;
    } else {
      dis = MIN_int16_T;
    }
  } else if (d >= 32768.0) {
    dis = MAX_int16_T;
  } else {
    dis = 0;
  }

  d = rt_roundd_snf(b_mod(now_foot[1], map_param[4]));
  if (d < 32768.0) {
    if (d >= -32768.0) {
      dis_min = (short)d;
    } else {
      dis_min = MIN_int16_T;
    }
  } else if (d >= 32768.0) {
    dis_min = MAX_int16_T;
  } else {
    dis_min = 0;
  }

  good_foot_id = dis + dis_min;
  if (good_foot_id > 32767) {
    good_foot_id = 32767;
  } else {
    if (good_foot_id < -32768) {
      good_foot_id = -32768;
    }
  }

  good_foot_id += 20;
  if (good_foot_id > 32767) {
    good_foot_id = 32767;
  }

  Gy_foot_now = (short)good_foot_id;
  if ((short)b_dis < 1) {
    Gx_foot_now = 1;
  }

  if ((short)b_dis > 40) {
    Gx_foot_now = 40;
  }

  if ((short)good_foot_id < 1) {
    Gy_foot_now = 1;
  }

  if ((short)good_foot_id > 40) {
    Gy_foot_now = 40;
  }

  /* 查找一个坐标在全局地图上的位置 */
  /* map_param(1); */
  /* map_param(2); */
  d = rt_roundd_snf(slip_foot[0] / map_param[4]);
  if (d < 32768.0) {
    if (d >= -32768.0) {
      dis = (short)d;
    } else {
      dis = MIN_int16_T;
    }
  } else if (d >= 32768.0) {
    dis = MAX_int16_T;
  } else {
    dis = 0;
  }

  d = rt_roundd_snf(b_mod(slip_foot[0], map_param[4]));
  if (d < 32768.0) {
    if (d >= -32768.0) {
      dis_min = (short)d;
    } else {
      dis_min = MIN_int16_T;
    }
  } else if (d >= 32768.0) {
    dis_min = MAX_int16_T;
  } else {
    dis_min = 0;
  }

  b_dis = dis + dis_min;
  if (b_dis > 32767) {
    b_dis = 32767;
  } else {
    if (b_dis < -32768) {
      b_dis = -32768;
    }
  }

  b_dis += 20;
  if (b_dis > 32767) {
    b_dis = 32767;
  }

  Gx_slip_now = (short)b_dis;
  d = rt_roundd_snf(slip_foot[1] / map_param[4]);
  if (d < 32768.0) {
    if (d >= -32768.0) {
      dis = (short)d;
    } else {
      dis = MIN_int16_T;
    }
  } else if (d >= 32768.0) {
    dis = MAX_int16_T;
  } else {
    dis = 0;
  }

  d = rt_roundd_snf(b_mod(slip_foot[1], map_param[4]));
  if (d < 32768.0) {
    if (d >= -32768.0) {
      dis_min = (short)d;
    } else {
      dis_min = MIN_int16_T;
    }
  } else if (d >= 32768.0) {
    dis_min = MAX_int16_T;
  } else {
    dis_min = 0;
  }

  good_foot_id = dis + dis_min;
  if (good_foot_id > 32767) {
    good_foot_id = 32767;
  } else {
    if (good_foot_id < -32768) {
      good_foot_id = -32768;
    }
  }

  good_foot_id += 20;
  if (good_foot_id > 32767) {
    good_foot_id = 32767;
  }

  Gy_slip_now = (short)good_foot_id;
  if ((short)b_dis < 1) {
    Gx_slip_now = 1;
  }

  if ((short)b_dis > 40) {
    Gx_slip_now = 40;
  }

  if ((short)good_foot_id < 1) {
    Gy_slip_now = 1;
  }

  if ((short)good_foot_id > 40) {
    Gy_slip_now = 40;
  }

  good_foot_id = 0;
  good_foot_cnt1 = -1;
  for (i = 0; i < 40; i++) {
    /* 查可落足X方向ID */
    d = map_local_foot[i + 40 * (Gy_foot_now - 1)];
    if ((d >= check_param[1]) && (d < check_param[2])) {
      good_foot_cnt1++;
      good_foot_id = i + 1;
    }
  }

  good_foot_id_f = 0;
  good_foot_cnt2 = 0;
  good_foot_id_b = 0;
  good_foot_cnt3 = -1;
  for (i = 0; i <= good_foot_cnt1; i++) {
    /* 查当前点速度前方ID */
    if (spd_fb >= 0.0) {
      if (good_foot_id >= Gx_foot_now) {
        b_dis = Hw_id_2 + r_L_id_2;
        if (b_dis > 32767) {
          b_dis = 32767;
        } else {
          if (b_dis < -32768) {
            b_dis = -32768;
          }
        }

        if (good_foot_id <= b_dis) {
          good_foot_cnt2++;
          good_foot_id_f = good_foot_id;
        }
      }

      if (good_foot_id <= Gx_foot_now) {
        b_dis = Hw_id_2 - r_L_id_2;
        if (b_dis > 32767) {
          b_dis = 32767;
        } else {
          if (b_dis < -32768) {
            b_dis = -32768;
          }
        }

        if (good_foot_id >= b_dis) {
          /* 查当前点后方ID */
          good_foot_cnt3++;
          good_foot_id_b = good_foot_id;
        }
      }
    } else {
      if (good_foot_id >= Gx_foot_now) {
        b_dis = Hw_id_2 + r_L_id_2;
        if (b_dis > 32767) {
          b_dis = 32767;
        } else {
          if (b_dis < -32768) {
            b_dis = -32768;
          }
        }

        if (good_foot_id <= b_dis) {
          good_foot_cnt3++;
          good_foot_id_b = good_foot_id;
        }
      }

      if (good_foot_id <= Gx_foot_now) {
        b_dis = Hw_id_2 - r_L_id_2;
        if (b_dis > 32767) {
          b_dis = 32767;
        } else {
          if (b_dis < -32768) {
            b_dis = -32768;
          }
        }

        if (good_foot_id >= b_dis) {
          /* 查当前点后方ID */
          good_foot_cnt2++;
          good_foot_id_f = good_foot_id;
        }
      }
    }
  }

  dis_min = 99;
  good_foot_id = 99;
  if (good_foot_cnt2 != 0) {
    /* 在移动方向前方找离SLIP最近的点 */
    dis = (short)(good_foot_id_f - Gx_slip_now);
    if (dis < 0) {
      b_dis = -dis;
    } else {
      b_dis = dis;
    }

    if (b_dis <= 99) {
      /* 如果SLIP位置不在危险区域仍然采用 */
      b_dis = (Gx_slip_now + 40 * (Gy_slip_now - 1)) - 1;
      if ((map_local_foot[b_dis] > check_param[1]) && (map_local_foot[b_dis] <
           check_param[2])) {
        gfoot[0] = slip_foot[0];
        gfoot[1] = slip_foot[1];
        gfoot[2] = slip_foot[2];
      } else {
        *is_found = 1.0;
        if ((leg_sel == 0) || (leg_sel == 2)) {
          gfoot[0] = ((double)good_foot_id_f - 20.0) * map_param[4] - map_param
            [4] / 2.0;
        } else {
          gfoot[0] = ((double)good_foot_id_f - 20.0) * map_param[4] + map_param
            [4] / 2.0;
        }

        gfoot[1] = ((double)Gy_foot_now - 20.0) * map_param[4] - map_param[4] /
          2.0;
      }
    }
  } else {
    /* 后方找离SLIP最近的点 */
    for (i = 0; i <= good_foot_cnt3; i++) {
      dis = (short)(good_foot_id_b - Gx_slip_now);
      if (dis < 0) {
        dis = (short)-dis;
      }

      if (dis <= dis_min) {
        dis_min = dis;
        good_foot_id = good_foot_id_b;
      }
    }

    if (good_foot_id < 99) {
      /* 如果SLIP位置不在危险区域仍然采用 */
      b_dis = (Gx_slip_now + 40 * (Gy_slip_now - 1)) - 1;
      if ((map_local_foot[b_dis] > check_param[1]) && (map_local_foot[b_dis] <
           check_param[2])) {
        gfoot[0] = slip_foot[0];
        gfoot[1] = slip_foot[1];
        gfoot[2] = slip_foot[2];
      } else {
        *is_found = 1.0;
        if ((leg_sel == 0) || (leg_sel == 2)) {
          gfoot[0] = ((double)good_foot_id - 20.0) * map_param[4] - map_param[4]
            / 2.0;
        } else {
          gfoot[0] = ((double)good_foot_id - 20.0) * map_param[4] + map_param[4]
            / 2.0;
        }

        gfoot[1] = ((double)Gy_foot_now - 20.0) * map_param[4] - map_param[4] /
          2.0;
      }
    } else {
      gfoot[0] = slip_foot[0];
      gfoot[1] = slip_foot[1];
      gfoot[2] = slip_foot[2];
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void foot_good_foot_found_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_foot_good_foot_found = true;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void foot_good_foot_found_terminate(void)
{
  /* (no terminate code required) */
  isInitialized_foot_good_foot_found = false;
}

/*
 * File trailer for foot_good_foot_found.c
 *
 * [EOF]
 */
