/*
 * File: _coder_move_local_map_cplus_api.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 16-Dec-2021 16:11:24
 */

#ifndef _CODER_MOVE_LOCAL_MAP_CPLUS_API_H
#define _CODER_MOVE_LOCAL_MAP_CPLUS_API_H

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
extern void move_local_map_cplus(real_T cog_n[3], real_T map_n[1000000], real_T
  map_param[5], real_T map_local[1600]);
extern void move_local_map_cplus_api(const mxArray * const prhs[3], int32_T nlhs,
  const mxArray *plhs[1]);
extern void move_local_map_cplus_atexit(void);
extern void move_local_map_cplus_initialize(void);
extern void move_local_map_cplus_terminate(void);
extern void move_local_map_cplus_xil_shutdown(void);
extern void move_local_map_cplus_xil_terminate(void);

#endif

/*
 * File trailer for _coder_move_local_map_cplus_api.h
 *
 * [EOF]
 */
