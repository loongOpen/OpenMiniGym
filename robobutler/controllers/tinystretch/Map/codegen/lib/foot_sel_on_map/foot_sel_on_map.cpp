/*
 * File: foot_sel_on_map.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 16-Dec-2021 14:45:04
 */

/* Include Files */
#include "foot_sel_on_map.h"
#include "foot_sel_on_map_emxutil.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static void b_sort(double x[3]);
static double find_3x3_height_on_grid(const double grid[2], const double
  map_local[1600], const double map_param[5]);
static double rt_roundd_snf(double u);
static void sort(double x[9]);

/* Function Definitions */

/*
 * Arguments    : double x[3]
 * Return Type  : void
 */
static void b_sort(double x[3])
{
  double tmp;
  if (x[0] >= x[1]) {
    if (!(x[1] >= x[2])) {
      if (x[0] >= x[2]) {
        tmp = x[1];
        x[1] = x[2];
        x[2] = tmp;
      } else {
        tmp = x[2];
        x[2] = x[1];
        x[1] = x[0];
        x[0] = tmp;
      }
    }
  } else if (x[0] >= x[2]) {
    tmp = x[0];
    x[0] = x[1];
    x[1] = tmp;
  } else if (x[1] >= x[2]) {
    tmp = x[0];
    x[0] = x[1];
    x[1] = x[2];
    x[2] = tmp;
  } else {
    tmp = x[0];
    x[0] = x[2];
    x[2] = tmp;
  }
}

/*
 * 查找一个坐标在全局地图上的位置
 * Arguments    : const double grid[2]
 *                const double map_local[1600]
 *                const double map_param[5]
 * Return Type  : double
 */
static double find_3x3_height_on_grid(const double grid[2], const double
  map_local[1600], const double map_param[5])
{
  double grid_height;
  double height[25];
  double out;
  double b_out;
  int height_tmp;
  memset(&height[0], 0, 25U * sizeof(double));
  out = grid[0];
  if (grid[0] < 1.0) {
    out = 1.0;
  }

  if (grid[0] > map_param[0]) {
    out = map_param[0];
  }

  b_out = grid[1];
  if (grid[1] < 1.0) {
    b_out = 1.0;
  }

  if (grid[1] > map_param[1]) {
    b_out = map_param[1];
  }

  height_tmp = ((int)out + 40 * ((int)b_out - 1)) - 1;
  height[0] = map_local[height_tmp];
  out = grid[0] + 1.0;
  if (grid[0] + 1.0 < 1.0) {
    out = 1.0;
  }

  if (grid[0] + 1.0 > map_param[0]) {
    out = map_param[0];
  }

  b_out = grid[1];
  if (grid[1] < 1.0) {
    b_out = 1.0;
  }

  if (grid[1] > map_param[1]) {
    b_out = map_param[1];
  }

  height[1] = map_local[((int)out + 40 * ((int)b_out - 1)) - 1];
  out = grid[0] - 1.0;
  if (grid[0] - 1.0 < 1.0) {
    out = 1.0;
  }

  if (grid[0] - 1.0 > map_param[0]) {
    out = map_param[0];
  }

  b_out = grid[1];
  if (grid[1] < 1.0) {
    b_out = 1.0;
  }

  if (grid[1] > map_param[1]) {
    b_out = map_param[1];
  }

  height[2] = map_local[((int)out + 40 * ((int)b_out - 1)) - 1];
  out = grid[0];
  if (grid[0] < 1.0) {
    out = 1.0;
  }

  if (grid[0] > map_param[0]) {
    out = map_param[0];
  }

  b_out = grid[1] + 1.0;
  if (grid[1] + 1.0 < 1.0) {
    b_out = 1.0;
  }

  if (grid[1] + 1.0 > map_param[1]) {
    b_out = map_param[1];
  }

  height[3] = map_local[((int)out + 40 * ((int)b_out - 1)) - 1];
  out = grid[0];
  if (grid[0] < 1.0) {
    out = 1.0;
  }

  if (grid[0] > map_param[0]) {
    out = map_param[0];
  }

  b_out = grid[1] - 1.0;
  if (grid[1] - 1.0 < 1.0) {
    b_out = 1.0;
  }

  if (grid[1] - 1.0 > map_param[1]) {
    b_out = map_param[1];
  }

  height[4] = map_local[((int)out + 40 * ((int)b_out - 1)) - 1];
  grid_height = map_local[height_tmp];
  for (height_tmp = 0; height_tmp < 5; height_tmp++) {
    if (height[height_tmp] > grid_height) {
      grid_height = height[height_tmp];
    }
  }

  return grid_height;
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
 * Arguments    : double x[9]
 * Return Type  : void
 */
static void sort(double x[9])
{
  int j;
  double vwork_idx_0;
  double vwork_idx_1;
  double vwork_idx_2;
  double tmp;
  for (j = 0; j < 3; j++) {
    vwork_idx_0 = x[j];
    vwork_idx_1 = x[j + 3];
    vwork_idx_2 = x[j + 6];
    if (vwork_idx_0 >= vwork_idx_1) {
      if (!(vwork_idx_1 >= vwork_idx_2)) {
        if (vwork_idx_0 >= vwork_idx_2) {
          tmp = vwork_idx_1;
          vwork_idx_1 = vwork_idx_2;
          vwork_idx_2 = tmp;
        } else {
          tmp = vwork_idx_2;
          vwork_idx_2 = vwork_idx_1;
          vwork_idx_1 = vwork_idx_0;
          vwork_idx_0 = tmp;
        }
      }
    } else if (vwork_idx_0 >= vwork_idx_2) {
      tmp = vwork_idx_0;
      vwork_idx_0 = vwork_idx_1;
      vwork_idx_1 = tmp;
    } else if (vwork_idx_1 >= vwork_idx_2) {
      tmp = vwork_idx_0;
      vwork_idx_0 = vwork_idx_1;
      vwork_idx_1 = vwork_idx_2;
      vwork_idx_2 = tmp;
    } else {
      tmp = vwork_idx_0;
      vwork_idx_0 = vwork_idx_2;
      vwork_idx_2 = tmp;
    }

    x[j] = vwork_idx_0;
    x[j + 3] = vwork_idx_1;
    x[j + 6] = vwork_idx_2;
  }
}

/*
 * Arguments    : const double map_local[1600]
 *                const double map_param[5]
 *                const double check_param[3]
 *                const double robot_param[3]
 *                double map_local_foot[3]
 *                double location_map[1600]
 *                double map_edge_out[1600]
 * Return Type  : void
 */
void foot_sel_on_map(const double map_local[1600], const double map_param[5],
                     const double check_param[3], const double robot_param[3],
                     double map_local_foot[3], double location_map[1600], double
                     map_edge_out[1600])
{
  double COL;
  double MAP_BIT[1600];
  int i;
  double map_mid[1600];
  int j;
  int b_i;
  int i1;
  int r;
  emxArray_int8_T *sobel_Img;
  int c;
  double median3x3[9];
  int i2;
  int i3;
  int median3x3_tmp;
  int b_median3x3_tmp;
  emxArray_real_T *good_Img;
  double dv[3];
  double a;
  double d;
  double dv1[3];
  double b_a;
  emxArray_real_T *erosion_img;
  double dv2[3];
  unsigned long long u;
  double b_r[2];
  unsigned long long u1;
  unsigned long long u2;
  unsigned long long u3;
  unsigned long long u4;
  (void)robot_param;
  COL = map_param[1];
  memset(&MAP_BIT[0], 0, 1600U * sizeof(double));
  memset(&location_map[0], 0, 1600U * sizeof(double));
  memset(&map_edge_out[0], 0, 1600U * sizeof(double));
  map_local_foot[0] = 0.0;
  map_local_foot[1] = 0.0;
  map_local_foot[2] = 0.0;

  /* 安全落足区域 */
  for (i = 0; i < 40; i++) {
    /* 获取全局地图 */
    for (j = 0; j < 40; j++) {
      b_i = i + 40 * j;
      if ((map_local[b_i] < check_param[2]) && (map_local[b_i] > check_param[1]))
      {
        MAP_BIT[b_i] = 1.0;

        /* 构建二值图 */
      }
    }
  }

  /* 中值滤波 */
  memcpy(&map_mid[0], &MAP_BIT[0], 1600U * sizeof(double));
  b_i = (int)((map_param[0] - 1.0) + -1.0);
  if (0 <= b_i - 1) {
    i1 = (int)((COL - 1.0) + -1.0);
  }

  for (r = 0; r < b_i; r++) {
    for (c = 0; c < i1; c++) {
      i = r + 40 * c;
      median3x3[0] = MAP_BIT[i];
      j = r + 40 * (c + 1);
      median3x3[3] = MAP_BIT[j];
      median3x3_tmp = r + 40 * (c + 2);
      median3x3[6] = MAP_BIT[median3x3_tmp];
      median3x3[1] = MAP_BIT[i + 1];
      b_median3x3_tmp = j + 1;
      median3x3[4] = MAP_BIT[b_median3x3_tmp];
      median3x3[7] = MAP_BIT[median3x3_tmp + 1];
      median3x3[2] = MAP_BIT[i + 2];
      median3x3[5] = MAP_BIT[j + 2];
      median3x3[8] = MAP_BIT[median3x3_tmp + 2];
      sort(median3x3);
      dv[0] = median3x3[0];
      dv[1] = median3x3[3];
      dv[2] = median3x3[6];
      b_sort(dv);
      dv1[0] = median3x3[1];
      dv1[1] = median3x3[4];
      dv1[2] = median3x3[7];
      b_sort(dv1);
      dv2[0] = median3x3[2];
      dv2[1] = median3x3[5];
      dv2[2] = median3x3[8];
      b_sort(dv2);
      dv[0] = dv[2];
      dv[1] = dv1[1];
      dv[2] = dv2[0];
      b_sort(dv);
      map_mid[b_median3x3_tmp] = dv[1];
    }
  }

  emxInit_int8_T(&sobel_Img, 2);

  /* 边沿检测 不需要满足高度要求 */
  /* Median_Img = double(map_mid); */
  i1 = (int)map_param[0];
  i2 = sobel_Img->size[0] * sobel_Img->size[1];
  sobel_Img->size[0] = i1;
  i3 = (int)map_param[1];
  sobel_Img->size[1] = i3;
  emxEnsureCapacity_int8_T(sobel_Img, i2);
  i = i1 * i3;
  for (i2 = 0; i2 < i; i2++) {
    sobel_Img->data[i2] = 0;
  }

  for (r = 0; r < b_i; r++) {
    i2 = (int)((COL - 1.0) + -1.0);
    for (c = 0; c < i2; c++) {
      j = r + 40 * (c + 2);
      a = map_local[j + 2];
      median3x3_tmp = r + 40 * c;
      d = map_local[median3x3_tmp + 2];
      b_a = ((((map_local[j] + 2.0 * map_local[j + 1]) + a) -
              map_local[median3x3_tmp]) - 2.0 * map_local[median3x3_tmp + 1]) -
        d;
      b_median3x3_tmp = r + 40 * (c + 1);
      a = ((((map_local[median3x3_tmp] + 2.0 * map_local[b_median3x3_tmp]) +
             map_local[j]) - d) - 2.0 * map_local[b_median3x3_tmp + 2]) - a;

      /* Sobel_Num = abs(Sobel_x) + abs(Sobel_y); */
      sobel_Img->data[(r + sobel_Img->size[0] * (c + 1)) + 1] = (signed char)
        (sqrt(b_a * b_a + a * a) > 0.01);
    }
  }

  emxInit_real_T(&good_Img, 2);

  /* 去除边沿 */
  i2 = good_Img->size[0] * good_Img->size[1];
  good_Img->size[0] = i1;
  good_Img->size[1] = i3;
  emxEnsureCapacity_real_T(good_Img, i2);

  /* sobel_Img*map_mid; */
  for (r = 0; r < i1; r++) {
    for (c = 0; c < i3; c++) {
      if ((sobel_Img->data[r + sobel_Img->size[0] * c] < 0.5) && (map_mid[r + 40
           * c] > 0.5)) {
        good_Img->data[r + good_Img->size[0] * c] = 1.0;
      } else {
        good_Img->data[r + good_Img->size[0] * c] = 0.0;
      }
    }
  }

  /* 膨胀安全区域 */
  emxInit_real_T(&erosion_img, 2);
  if (check_param[0] != 0.0) {
    i2 = erosion_img->size[0] * erosion_img->size[1];
    erosion_img->size[0] = i1;
    erosion_img->size[1] = i3;
    emxEnsureCapacity_real_T(erosion_img, i2);
    for (i2 = 0; i2 < i; i2++) {
      erosion_img->data[i2] = 0.0;
    }

    for (r = 0; r < b_i; r++) {
      i2 = (int)((COL - 1.0) + -1.0);
      for (c = 0; c < i2; c++) {
        a = rt_roundd_snf(good_Img->data[r + good_Img->size[0] * c]);
        if (a < 1.8446744073709552E+19) {
          if (a >= 0.0) {
            u = (unsigned long long)a;
          } else {
            u = 0ULL;
          }
        } else if (a >= 1.8446744073709552E+19) {
          u = MAX_uint64_T;
        } else {
          u = 0ULL;
        }

        a = rt_roundd_snf(good_Img->data[r + good_Img->size[0] * (c + 1)]);
        if (a < 1.8446744073709552E+19) {
          if (a >= 0.0) {
            u1 = (unsigned long long)a;
          } else {
            u1 = 0ULL;
          }
        } else if (a >= 1.8446744073709552E+19) {
          u1 = MAX_uint64_T;
        } else {
          u1 = 0ULL;
        }

        a = rt_roundd_snf(good_Img->data[r + good_Img->size[0] * (c + 2)]);
        if (a < 1.8446744073709552E+19) {
          if (a >= 0.0) {
            u2 = (unsigned long long)a;
          } else {
            u2 = 0ULL;
          }
        } else if (a >= 1.8446744073709552E+19) {
          u2 = MAX_uint64_T;
        } else {
          u2 = 0ULL;
        }

        a = rt_roundd_snf((double)(u1 & u2));
        if (a < 1.8446744073709552E+19) {
          if (a >= 0.0) {
            u1 = (unsigned long long)a;
          } else {
            u1 = 0ULL;
          }
        } else if (a >= 1.8446744073709552E+19) {
          u1 = MAX_uint64_T;
        } else {
          u1 = 0ULL;
        }

        a = rt_roundd_snf((double)(u & u1));
        if (a < 1.8446744073709552E+19) {
          if (a >= 0.0) {
            u = (unsigned long long)a;
          } else {
            u = 0ULL;
          }
        } else if (a >= 1.8446744073709552E+19) {
          u = MAX_uint64_T;
        } else {
          u = 0ULL;
        }

        a = rt_roundd_snf(good_Img->data[(r + good_Img->size[0] * c) + 1]);
        if (a < 1.8446744073709552E+19) {
          if (a >= 0.0) {
            u1 = (unsigned long long)a;
          } else {
            u1 = 0ULL;
          }
        } else if (a >= 1.8446744073709552E+19) {
          u1 = MAX_uint64_T;
        } else {
          u1 = 0ULL;
        }

        a = rt_roundd_snf(good_Img->data[(r + good_Img->size[0] * (c + 1)) + 1]);
        if (a < 1.8446744073709552E+19) {
          if (a >= 0.0) {
            u2 = (unsigned long long)a;
          } else {
            u2 = 0ULL;
          }
        } else if (a >= 1.8446744073709552E+19) {
          u2 = MAX_uint64_T;
        } else {
          u2 = 0ULL;
        }

        a = rt_roundd_snf(good_Img->data[(r + good_Img->size[0] * (c + 2)) + 1]);
        if (a < 1.8446744073709552E+19) {
          if (a >= 0.0) {
            u3 = (unsigned long long)a;
          } else {
            u3 = 0ULL;
          }
        } else if (a >= 1.8446744073709552E+19) {
          u3 = MAX_uint64_T;
        } else {
          u3 = 0ULL;
        }

        a = rt_roundd_snf((double)(u2 & u3));
        if (a < 1.8446744073709552E+19) {
          if (a >= 0.0) {
            u2 = (unsigned long long)a;
          } else {
            u2 = 0ULL;
          }
        } else if (a >= 1.8446744073709552E+19) {
          u2 = MAX_uint64_T;
        } else {
          u2 = 0ULL;
        }

        a = rt_roundd_snf((double)(u1 & u2));
        if (a < 1.8446744073709552E+19) {
          if (a >= 0.0) {
            u1 = (unsigned long long)a;
          } else {
            u1 = 0ULL;
          }
        } else if (a >= 1.8446744073709552E+19) {
          u1 = MAX_uint64_T;
        } else {
          u1 = 0ULL;
        }

        a = rt_roundd_snf(good_Img->data[(r + good_Img->size[0] * c) + 2]);
        if (a < 1.8446744073709552E+19) {
          if (a >= 0.0) {
            u2 = (unsigned long long)a;
          } else {
            u2 = 0ULL;
          }
        } else if (a >= 1.8446744073709552E+19) {
          u2 = MAX_uint64_T;
        } else {
          u2 = 0ULL;
        }

        a = rt_roundd_snf(good_Img->data[(r + good_Img->size[0] * (c + 1)) + 2]);
        if (a < 1.8446744073709552E+19) {
          if (a >= 0.0) {
            u3 = (unsigned long long)a;
          } else {
            u3 = 0ULL;
          }
        } else if (a >= 1.8446744073709552E+19) {
          u3 = MAX_uint64_T;
        } else {
          u3 = 0ULL;
        }

        a = rt_roundd_snf(good_Img->data[(r + good_Img->size[0] * (c + 2)) + 2]);
        if (a < 1.8446744073709552E+19) {
          if (a >= 0.0) {
            u4 = (unsigned long long)a;
          } else {
            u4 = 0ULL;
          }
        } else if (a >= 1.8446744073709552E+19) {
          u4 = MAX_uint64_T;
        } else {
          u4 = 0ULL;
        }

        a = rt_roundd_snf((double)(u3 & u4));
        if (a < 1.8446744073709552E+19) {
          if (a >= 0.0) {
            u3 = (unsigned long long)a;
          } else {
            u3 = 0ULL;
          }
        } else if (a >= 1.8446744073709552E+19) {
          u3 = MAX_uint64_T;
        } else {
          u3 = 0ULL;
        }

        a = rt_roundd_snf((double)(u2 & u3));
        if (a < 1.8446744073709552E+19) {
          if (a >= 0.0) {
            u2 = (unsigned long long)a;
          } else {
            u2 = 0ULL;
          }
        } else if (a >= 1.8446744073709552E+19) {
          u2 = MAX_uint64_T;
        } else {
          u2 = 0ULL;
        }

        a = rt_roundd_snf((double)(u1 & u2));
        if (a < 1.8446744073709552E+19) {
          if (a >= 0.0) {
            u1 = (unsigned long long)a;
          } else {
            u1 = 0ULL;
          }
        } else if (a >= 1.8446744073709552E+19) {
          u1 = MAX_uint64_T;
        } else {
          u1 = 0ULL;
        }

        erosion_img->data[(r + erosion_img->size[0] * (c + 1)) + 1] = (double)(u
          & u1);
      }
    }
  } else {
    b_i = erosion_img->size[0] * erosion_img->size[1];
    erosion_img->size[0] = good_Img->size[0];
    erosion_img->size[1] = good_Img->size[1];
    emxEnsureCapacity_real_T(erosion_img, b_i);
    i = good_Img->size[0] * good_Img->size[1];
    for (b_i = 0; b_i < i; b_i++) {
      erosion_img->data[b_i] = good_Img->data[b_i];
    }
  }

  emxFree_real_T(&good_Img);

  /* 输出高程图 */
  /* 输出边沿图 */
  for (b_i = 0; b_i < 1600; b_i++) {
    MAP_BIT[b_i] = -0.1;
  }

  for (r = 0; r < i1; r++) {
    for (c = 0; c < i3; c++) {
      if (sobel_Img->data[r + sobel_Img->size[0] * c] > 0.5) {
        b_r[0] = (double)r + 1.0;
        b_r[1] = (double)c + 1.0;
        MAP_BIT[r + 40 * c] = find_3x3_height_on_grid(b_r, map_local, map_param);
      }
    }
  }

  emxFree_int8_T(&sobel_Img);
  for (r = 0; r < i1; r++) {
    for (c = 0; c < i3; c++) {
      i = r + 40 * c;
      map_edge_out[i] = MAP_BIT[i];
      location_map[i] = erosion_img->data[r + erosion_img->size[0] * c];
    }
  }

  emxFree_real_T(&erosion_img);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void foot_sel_on_map_initialize(void)
{
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void foot_sel_on_map_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for foot_sel_on_map.c
 *
 * [EOF]
 */
