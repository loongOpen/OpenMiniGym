//
// File: draw_local_map_good_location_cube.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 27-Sep-2022 17:30:11
//

// Include Files
#include "draw_local_map_good_location_cube.h"
#include "bwlabel.h"
#include <cstring>

// Function Definitions

//
// 梅花桩下提取块中心
// Arguments    : const double map_local[6400]
//                const double map_param[5]
//                const double check_param[3]
//                double centroids[40]
//                double *num
// Return Type  : void
//
void draw_local_map_good_location_cube(const double map_local[6400], const
  double [5], const double check_param[3], double centroids[40], double *num)
{
  double MAP_BIT[6400];
  int i;
  double L[6400];
  int j;
  int b_i;
  double X_c;
  double Y_c;
  std::memset(&MAP_BIT[0], 0, 6400U * sizeof(double));
  for (i = 0; i < 80; i++) {
    // 获取全局地图 高度限制
    for (j = 0; j < 80; j++) {
      b_i = i + 80 * j;
      if ((map_local[b_i] < check_param[2]) && (map_local[b_i] > check_param[1]))
      {
        MAP_BIT[b_i] = 1.0;

        // 构建二值图
      }
    }
  }

  // 扣取包络
  bwlabel(MAP_BIT, L, num);

  // Cluster
  std::memset(&centroids[0], 0, 40U * sizeof(double));
  b_i = static_cast<int>(*num);
  for (i = 0; i < b_i; i++) {
    double Cnt;
    X_c = 0.0;
    Y_c = 0.0;
    Cnt = 0.0;
    for (j = 0; j < 80; j++) {
      for (int ys = 0; ys < 80; ys++) {
        if (L[j + 80 * ys] == static_cast<double>(i) + 1.0) {
          X_c += static_cast<double>(j) + 1.0;
          Y_c += static_cast<double>(ys) + 1.0;
          Cnt++;
        }
      }
    }

    centroids[i] = X_c / Cnt;
    centroids[i + 20] = Y_c / Cnt;
  }
}

//
// File trailer for draw_local_map_good_location_cube.cpp
//
// [EOF]
//
