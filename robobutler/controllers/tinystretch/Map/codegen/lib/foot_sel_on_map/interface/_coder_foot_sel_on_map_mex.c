/*
 * File: _coder_foot_sel_on_map_mex.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 01-Oct-2022 23:15:16
 */

/* Include Files */
#include "_coder_foot_sel_on_map_mex.h"
#include "_coder_foot_sel_on_map_api.h"

/* Function Declarations */
MEXFUNCTION_LINKAGE void foot_sel_on_map_mexFunction(int32_T nlhs, mxArray *
  plhs[2], int32_T nrhs, const mxArray *prhs[3]);

/* Function Definitions */

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[2]
 *                int32_T nrhs
 *                const mxArray *prhs[3]
 * Return Type  : void
 */
void foot_sel_on_map_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T nrhs,
  const mxArray *prhs[3])
{
  const mxArray *outputs[2];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        15, "foot_sel_on_map");
  }

  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 15,
                        "foot_sel_on_map");
  }

  /* Call the function. */
  foot_sel_on_map_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[]
 *                int32_T nrhs
 *                const mxArray *prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(&foot_sel_on_map_atexit);

  /* Module initialization. */
  foot_sel_on_map_initialize();

  /* Dispatch the entry-point. */
  foot_sel_on_map_mexFunction(nlhs, plhs, nrhs, prhs);

  /* Module termination. */
  foot_sel_on_map_terminate();
}

/*
 * Arguments    : void
 * Return Type  : emlrtCTX
 */
emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/*
 * File trailer for _coder_foot_sel_on_map_mex.c
 *
 * [EOF]
 */
