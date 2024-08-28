/*
 * File: foot_sel_on_map.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 01-Oct-2022 23:15:16
 */

/* Include Files */
#include "foot_sel_on_map.h"
#include "foot_sel_on_map_emxutil.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static double find_3x3_height_on_grid(const double grid[2], const double
  map_local[6400], const double map_param[5]);
static double rt_roundd_snf(double u);

/* Function Definitions */

/*
 * 查找一个坐标在全局地图上的位置
 * Arguments    : const double grid[2]
 *                const double map_local[6400]
 *                const double map_param[5]
 * Return Type  : double
 */
static double find_3x3_height_on_grid(const double grid[2], const double
  map_local[6400], const double map_param[5])
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

  height_tmp = ((int)out + 80 * ((int)b_out - 1)) - 1;
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

  height[1] = map_local[((int)out + 80 * ((int)b_out - 1)) - 1];
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

  height[2] = map_local[((int)out + 80 * ((int)b_out - 1)) - 1];
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

  height[3] = map_local[((int)out + 80 * ((int)b_out - 1)) - 1];
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

  height[4] = map_local[((int)out + 80 * ((int)b_out - 1)) - 1];
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
 * 输入高程图 输出安全落足区域 和 梅花桩位置  导出库用
 * Arguments    : const double map_local[6400]
 *                const double map_param[5]
 *                const double check_param[3]
 *                double location_map[6400]
 *                double map_edge_out[6400]
 * Return Type  : void
 */
void foot_sel_on_map(const double map_local[6400], const double map_param[5],
                     const double check_param[3], double location_map[6400],
                     double map_edge_out[6400])
{
  double COL;
  int i;
  double MAP[6400];
  int b_i;
  double MAP_BIT[6400];
  emxArray_int8_T *sobel_Img;
  int j;
  int i1;
  int i2;
  int loop_ub_tmp;
  int r;
  emxArray_real_T *good_Img;
  int i3;
  int c;
  double a;
  double d;
  double b_a;
  int a_tmp;
  emxArray_real_T *erosion_img;
  unsigned long long u;
  unsigned long long u1;
  unsigned long long u2;
  double b_r[2];
  unsigned long long u3;
  unsigned long long u4;
  COL = map_param[1];
  for (i = 0; i < 6400; i++) {
    MAP[i] = -1.0;
    MAP_BIT[i] = 0.0;
    location_map[i] = 0.0;
    map_edge_out[i] = 0.0;
  }

  /* 安全落足区域 */
  for (b_i = 0; b_i < 80; b_i++) {
    /* 获取全局地图 */
    for (j = 0; j < 80; j++) {
      i = b_i + 80 * j;
      if ((map_local[i] < check_param[2]) && (map_local[i] > check_param[1])) {
        MAP[i] = map_local[i];
        MAP_BIT[i] = 1.0;

        /* 构建二值图 */
      }
    }
  }

  emxInit_int8_T(&sobel_Img, 2);

  /*  %中值滤波 */
  /*  map_mid = MAP_BIT; */
  /*  for r = 2:ROW-1 */
  /*     for c = 2:COL-1 */
  /*           median3x3 =[MAP_BIT(r-1,c-1)      MAP_BIT(r-1,c) MAP_BIT(r-1,c+1) */
  /*                       MAP_BIT(r,c-1)        MAP_BIT(r,c)   MAP_BIT(r,c+1) */
  /*                       MAP_BIT(r+1,c-1)      MAP_BIT(r+1,c) MAP_BIT(r+1,c+1)]; */
  /*           sort1 = sort(median3x3, 2, 'descend'); */
  /*           sort2 = sort([sort1(1), sort1(4), sort1(7)], 'descend'); */
  /*           sort3 = sort([sort1(2), sort1(5), sort1(8)], 'descend'); */
  /*           sort4 = sort([sort1(3), sort1(6), sort1(9)], 'descend'); */
  /*           mid_num = sort([sort2(3), sort3(2), sort4(1)], 'descend'); */
  /*           map_mid(r,c) = mid_num(2); */
  /*       end */
  /*  end */
  /* 边沿检测 不需要满足高度要求 */
  /* Median_Img = double(map_mid); */
  i = (int)map_param[0];
  i1 = sobel_Img->size[0] * sobel_Img->size[1];
  sobel_Img->size[0] = i;
  i2 = (int)map_param[1];
  sobel_Img->size[1] = i2;
  emxEnsureCapacity_int8_T(sobel_Img, i1);
  loop_ub_tmp = i * i2;
  for (i1 = 0; i1 < loop_ub_tmp; i1++) {
    sobel_Img->data[i1] = 0;
  }

  i1 = (int)((map_param[0] - 1.0) + -1.0);
  for (r = 0; r < i1; r++) {
    i3 = (int)((COL - 1.0) + -1.0);
    for (c = 0; c < i3; c++) {
      b_i = r + 80 * (c + 2);
      a = map_local[b_i + 2];
      j = r + 80 * c;
      d = map_local[j + 2];
      b_a = ((((map_local[b_i] + 2.0 * map_local[b_i + 1]) + a) - map_local[j])
             - 2.0 * map_local[j + 1]) - d;
      a_tmp = r + 80 * (c + 1);
      a = ((((map_local[j] + 2.0 * map_local[a_tmp]) + map_local[b_i]) - d) -
           2.0 * map_local[a_tmp + 2]) - a;

      /* Sobel_Num = abs(Sobel_x) + abs(Sobel_y); */
      sobel_Img->data[(r + sobel_Img->size[0] * (c + 1)) + 1] = (signed char)
        (sqrt(b_a * b_a + a * a) > 0.01);
    }
  }

  emxInit_real_T(&good_Img, 2);

  /* 去除边沿 */
  i3 = good_Img->size[0] * good_Img->size[1];
  good_Img->size[0] = i;
  good_Img->size[1] = i2;
  emxEnsureCapacity_real_T(good_Img, i3);

  /* sobel_Img*map_mid; */
  for (r = 0; r < i; r++) {
    for (c = 0; c < i2; c++) {
      if ((sobel_Img->data[r + sobel_Img->size[0] * c] < 0.5) && (MAP_BIT[r + 80
           * c] > 0.5)) {
        good_Img->data[r + good_Img->size[0] * c] = 1.0;
      } else {
        good_Img->data[r + good_Img->size[0] * c] = 0.0;
      }
    }
  }

  /* 膨胀安全区域 */
  emxInit_real_T(&erosion_img, 2);
  if (check_param[0] >= 1.0) {
    i3 = erosion_img->size[0] * erosion_img->size[1];
    erosion_img->size[0] = i;
    erosion_img->size[1] = i2;
    emxEnsureCapacity_real_T(erosion_img, i3);
    for (i3 = 0; i3 < loop_ub_tmp; i3++) {
      erosion_img->data[i3] = 0.0;
    }

    for (r = 0; r < i1; r++) {
      i3 = (int)((COL - 1.0) + -1.0);
      for (c = 0; c < i3; c++) {
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
    i3 = erosion_img->size[0] * erosion_img->size[1];
    erosion_img->size[0] = good_Img->size[0];
    erosion_img->size[1] = good_Img->size[1];
    emxEnsureCapacity_real_T(erosion_img, i3);
    b_i = good_Img->size[0] * good_Img->size[1];
    for (i3 = 0; i3 < b_i; i3++) {
      erosion_img->data[i3] = good_Img->data[i3];
    }
  }

  if (check_param[0] >= 2.0) {
    i3 = good_Img->size[0] * good_Img->size[1];
    good_Img->size[0] = i;
    good_Img->size[1] = i2;
    emxEnsureCapacity_real_T(good_Img, i3);
    for (i3 = 0; i3 < loop_ub_tmp; i3++) {
      good_Img->data[i3] = 0.0;
    }

    for (r = 0; r < i1; r++) {
      i3 = (int)((COL - 1.0) + -1.0);
      for (c = 0; c < i3; c++) {
        a = rt_roundd_snf(erosion_img->data[r + erosion_img->size[0] * c]);
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

        a = rt_roundd_snf(erosion_img->data[r + erosion_img->size[0] * (c + 1)]);
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

        a = rt_roundd_snf(erosion_img->data[r + erosion_img->size[0] * (c + 2)]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * c) + 1]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * (c + 1))
                          + 1]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * (c + 2))
                          + 1]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * c) + 2]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * (c + 1))
                          + 2]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * (c + 2))
                          + 2]);
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

        good_Img->data[(r + good_Img->size[0] * (c + 1)) + 1] = (double)(u & u1);
      }
    }

    i3 = erosion_img->size[0] * erosion_img->size[1];
    erosion_img->size[0] = good_Img->size[0];
    erosion_img->size[1] = good_Img->size[1];
    emxEnsureCapacity_real_T(erosion_img, i3);
    b_i = good_Img->size[0] * good_Img->size[1];
    for (i3 = 0; i3 < b_i; i3++) {
      erosion_img->data[i3] = good_Img->data[i3];
    }
  }

  if (check_param[0] >= 3.0) {
    i3 = good_Img->size[0] * good_Img->size[1];
    good_Img->size[0] = i;
    good_Img->size[1] = i2;
    emxEnsureCapacity_real_T(good_Img, i3);
    for (i3 = 0; i3 < loop_ub_tmp; i3++) {
      good_Img->data[i3] = 0.0;
    }

    for (r = 0; r < i1; r++) {
      i3 = (int)((COL - 1.0) + -1.0);
      for (c = 0; c < i3; c++) {
        a = rt_roundd_snf(erosion_img->data[r + erosion_img->size[0] * c]);
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

        a = rt_roundd_snf(erosion_img->data[r + erosion_img->size[0] * (c + 1)]);
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

        a = rt_roundd_snf(erosion_img->data[r + erosion_img->size[0] * (c + 2)]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * c) + 1]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * (c + 1))
                          + 1]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * (c + 2))
                          + 1]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * c) + 2]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * (c + 1))
                          + 2]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * (c + 2))
                          + 2]);
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

        good_Img->data[(r + good_Img->size[0] * (c + 1)) + 1] = (double)(u & u1);
      }
    }

    i3 = erosion_img->size[0] * erosion_img->size[1];
    erosion_img->size[0] = good_Img->size[0];
    erosion_img->size[1] = good_Img->size[1];
    emxEnsureCapacity_real_T(erosion_img, i3);
    b_i = good_Img->size[0] * good_Img->size[1];
    for (i3 = 0; i3 < b_i; i3++) {
      erosion_img->data[i3] = good_Img->data[i3];
    }
  }

  if (check_param[0] >= 4.0) {
    i3 = good_Img->size[0] * good_Img->size[1];
    good_Img->size[0] = i;
    good_Img->size[1] = i2;
    emxEnsureCapacity_real_T(good_Img, i3);
    for (i3 = 0; i3 < loop_ub_tmp; i3++) {
      good_Img->data[i3] = 0.0;
    }

    for (r = 0; r < i1; r++) {
      i3 = (int)((COL - 1.0) + -1.0);
      for (c = 0; c < i3; c++) {
        a = rt_roundd_snf(erosion_img->data[r + erosion_img->size[0] * c]);
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

        a = rt_roundd_snf(erosion_img->data[r + erosion_img->size[0] * (c + 1)]);
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

        a = rt_roundd_snf(erosion_img->data[r + erosion_img->size[0] * (c + 2)]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * c) + 1]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * (c + 1))
                          + 1]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * (c + 2))
                          + 1]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * c) + 2]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * (c + 1))
                          + 2]);
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

        a = rt_roundd_snf(erosion_img->data[(r + erosion_img->size[0] * (c + 2))
                          + 2]);
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

        good_Img->data[(r + good_Img->size[0] * (c + 1)) + 1] = (double)(u & u1);
      }
    }

    i1 = erosion_img->size[0] * erosion_img->size[1];
    erosion_img->size[0] = good_Img->size[0];
    erosion_img->size[1] = good_Img->size[1];
    emxEnsureCapacity_real_T(erosion_img, i1);
    b_i = good_Img->size[0] * good_Img->size[1];
    for (i1 = 0; i1 < b_i; i1++) {
      erosion_img->data[i1] = good_Img->data[i1];
    }
  }

  emxFree_real_T(&good_Img);

  /* 输出高程图 */
  for (r = 0; r < i; r++) {
    for (c = 0; c < i2; c++) {
      if (erosion_img->data[r + erosion_img->size[0] * c] < 0.5) {
        MAP[r + 80 * c] = -1.0;
      }
    }
  }

  emxFree_real_T(&erosion_img);

  /* 输出边沿图 */
  for (i1 = 0; i1 < 6400; i1++) {
    MAP_BIT[i1] = -1.0;
  }

  for (r = 0; r < i; r++) {
    for (c = 0; c < i2; c++) {
      if (sobel_Img->data[r + sobel_Img->size[0] * c] > 0.5) {
        b_r[0] = (double)r + 1.0;
        b_r[1] = (double)c + 1.0;
        MAP_BIT[r + 80 * c] = find_3x3_height_on_grid(b_r, map_local, map_param);
      }
    }
  }

  emxFree_int8_T(&sobel_Img);
  for (r = 0; r < i; r++) {
    for (c = 0; c < i2; c++) {
      b_i = r + 80 * c;
      location_map[b_i] = MAP[b_i];
      map_edge_out[b_i] = MAP_BIT[b_i] + 0.005;
    }
  }
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
