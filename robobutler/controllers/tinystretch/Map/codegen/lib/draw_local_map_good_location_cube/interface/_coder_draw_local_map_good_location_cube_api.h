/*
 * File: _coder_draw_local_map_good_location_cube_api.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 27-Sep-2022 17:30:11
 */

#ifndef _CODER_DRAW_LOCAL_MAP_GOOD_LOCATION_CUBE_API_H
#define _CODER_DRAW_LOCAL_MAP_GOOD_LOCATION_CUBE_API_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void draw_local_map_good_location_cube(real_T map_local[6400], real_T
  map_param[5], real_T check_param[3], real_T centroids[40], real_T *num);
extern void draw_local_map_good_location_cube_api(const mxArray * const prhs[3],
  int32_T nlhs, const mxArray *plhs[2]);
extern void draw_local_map_good_location_cube_atexit(void);
extern void draw_local_map_good_location_cube_initialize(void);
extern void draw_local_map_good_location_cube_terminate(void);
extern void draw_local_map_good_location_cube_xil_shutdown(void);
extern void draw_local_map_good_location_cube_xil_terminate(void);

#endif

/*
 * File trailer for _coder_draw_local_map_good_location_cube_api.h
 *
 * [EOF]
 */
