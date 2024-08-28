/*
 * File: _coder_foot_good_foot_found_api.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 22-Dec-2021 17:35:42
 */

#ifndef _CODER_FOOT_GOOD_FOOT_FOUND_API_H
#define _CODER_FOOT_GOOD_FOOT_FOUND_API_H

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
extern void foot_good_foot_found(int16_T leg_sel, real_T spd_fb, real_T
  now_foot[3], real_T slip_foot[3], real_T map_local_foot[1600], real_T
  map_param[5], real_T check_param[3], real_T robot_param[3], real_T gfoot[3],
  real_T *is_found);
extern void foot_good_foot_found_api(const mxArray * const prhs[8], int32_T nlhs,
  const mxArray *plhs[2]);
extern void foot_good_foot_found_atexit(void);
extern void foot_good_foot_found_initialize(void);
extern void foot_good_foot_found_terminate(void);
extern void foot_good_foot_found_xil_shutdown(void);
extern void foot_good_foot_found_xil_terminate(void);

#endif

/*
 * File trailer for _coder_foot_good_foot_found_api.h
 *
 * [EOF]
 */
