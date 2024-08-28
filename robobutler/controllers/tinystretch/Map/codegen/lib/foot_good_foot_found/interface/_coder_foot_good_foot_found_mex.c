/*
 * File: _coder_foot_good_foot_found_mex.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 22-Dec-2021 17:35:42
 */

/* Include Files */
#include "_coder_foot_good_foot_found_mex.h"
#include "_coder_foot_good_foot_found_api.h"

/* Function Declarations */
MEXFUNCTION_LINKAGE void c_foot_good_foot_found_mexFunct(int32_T nlhs, mxArray
  *plhs[2], int32_T nrhs, const mxArray *prhs[8]);

/* Function Definitions */

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[2]
 *                int32_T nrhs
 *                const mxArray *prhs[8]
 * Return Type  : void
 */
void c_foot_good_foot_found_mexFunct(int32_T nlhs, mxArray *plhs[2], int32_T
  nrhs, const mxArray *prhs[8])
{
  const mxArray *outputs[2];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 8) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 8, 4,
                        20, "foot_good_foot_found");
  }

  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 20,
                        "foot_good_foot_found");
  }

  /* Call the function. */
  foot_good_foot_found_api(prhs, nlhs, outputs);

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
  mexAtExit(&foot_good_foot_found_atexit);

  /* Module initialization. */
  foot_good_foot_found_initialize();

  /* Dispatch the entry-point. */
  c_foot_good_foot_found_mexFunct(nlhs, plhs, nrhs, prhs);

  /* Module termination. */
  foot_good_foot_found_terminate();
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
 * File trailer for _coder_foot_good_foot_found_mex.c
 *
 * [EOF]
 */
