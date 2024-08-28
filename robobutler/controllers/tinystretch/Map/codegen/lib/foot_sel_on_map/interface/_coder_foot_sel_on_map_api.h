/*
 * File: _coder_foot_sel_on_map_api.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 01-Oct-2022 23:15:16
 */

#ifndef _CODER_FOOT_SEL_ON_MAP_API_H
#define _CODER_FOOT_SEL_ON_MAP_API_H

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
extern void foot_sel_on_map(real_T map_local[6400], real_T map_param[5], real_T
  check_param[3], real_T location_map[6400], real_T map_edge_out[6400]);
extern void foot_sel_on_map_api(const mxArray * const prhs[3], int32_T nlhs,
  const mxArray *plhs[2]);
extern void foot_sel_on_map_atexit(void);
extern void foot_sel_on_map_initialize(void);
extern void foot_sel_on_map_terminate(void);
extern void foot_sel_on_map_xil_shutdown(void);
extern void foot_sel_on_map_xil_terminate(void);

#endif

/*
 * File trailer for _coder_foot_sel_on_map_api.h
 *
 * [EOF]
 */
