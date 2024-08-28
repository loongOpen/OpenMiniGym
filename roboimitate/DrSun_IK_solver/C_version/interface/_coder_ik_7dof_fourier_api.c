/*
 * File: _coder_ik_7dof_fourier_api.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 07-Oct-2023 11:12:14
 */

/* Include Files */
#include "tmwtypes.h"
#include "_coder_ik_7dof_fourier_api.h"
#include "_coder_ik_7dof_fourier_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true, false, 131434U, NULL,
  "ik_7dof_fourier", NULL, false, { 2045744189U, 2170104910U, 2743257031U,
    4284093946U }, NULL };

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *R_x, const
  char_T *identifier);
static const mxArray *emlrt_marshallOut(const real_T u[7]);

/* Function Definitions */

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *R_x
 *                const char_T *identifier
 * Return Type  : real_T
 */
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *R_x, const
  char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(R_x), &thisId);
  emlrtDestroyArray(&R_x);
  return y;
}

/*
 * Arguments    : const real_T u[7]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u[7])
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[2] = { 0, 0 };

  static const int32_T iv1[2] = { 1, 7 };

  y = NULL;
  m0 = emlrtCreateNumericArray(2, iv0, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)u);
  emlrtSetDimensions((mxArray *)m0, iv1, 2);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[7]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void ik_7dof_fourier_api(const mxArray * const prhs[7], const mxArray *plhs[1])
{
  real_T (*theta)[7];
  real_T R_x;
  real_T R_y;
  real_T R_z;
  real_T p_x;
  real_T p_y;
  real_T p_z;
  real_T bet;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  theta = (real_T (*)[7])mxMalloc(sizeof(real_T [7]));

  /* Marshall function inputs */
  R_x = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "R_x");
  R_y = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "R_y");
  R_z = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "R_z");
  p_x = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "p_x");
  p_y = emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "p_y");
  p_z = emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "p_z");
  bet = emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "bet");

  /* Invoke the target function */
  ik_7dof_fourier(R_x, R_y, R_z, p_x, p_y, p_z, bet, *theta);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*theta);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ik_7dof_fourier_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  ik_7dof_fourier_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ik_7dof_fourier_initialize(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ik_7dof_fourier_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_ik_7dof_fourier_api.c
 *
 * [EOF]
 */
