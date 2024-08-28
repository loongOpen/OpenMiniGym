/*
 * File: _coder_ik_7dof_fourier_api.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 07-Oct-2023 11:12:14
 */

#ifndef _CODER_IK_7DOF_FOURIER_API_H
#define _CODER_IK_7DOF_FOURIER_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_ik_7dof_fourier_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void ik_7dof_fourier(real_T R_x, real_T R_y, real_T R_z, real_T p_x,
  real_T p_y, real_T p_z, real_T bet, real_T theta[7]);
extern void ik_7dof_fourier_api(const mxArray * const prhs[7], const mxArray
  *plhs[1]);
extern void ik_7dof_fourier_atexit(void);
extern void ik_7dof_fourier_initialize(void);
extern void ik_7dof_fourier_terminate(void);
extern void ik_7dof_fourier_xil_terminate(void);

#endif

/*
 * File trailer for _coder_ik_7dof_fourier_api.h
 *
 * [EOF]
 */
