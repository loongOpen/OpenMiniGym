/*
 * File: _coder_move_local_map_cplus_mex.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 16-Dec-2021 16:11:24
 */

/* Include Files */
#include "_coder_move_local_map_cplus_mex.h"
#include "_coder_move_local_map_cplus_api.h"

/* Function Declarations */
MEXFUNCTION_LINKAGE void c_move_local_map_cplus_mexFunct(int32_T nlhs, mxArray
  *plhs[1], int32_T nrhs, const mxArray *prhs[3]);

/* Function Definitions */

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[1]
 *                int32_T nrhs
 *                const mxArray *prhs[3]
 * Return Type  : void
 */
void c_move_local_map_cplus_mexFunct(int32_T nlhs, mxArray *plhs[1], int32_T
  nrhs, const mxArray *prhs[3])
{
  const mxArray *outputs[1];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        20, "move_local_map_cplus");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 20,
                        "move_local_map_cplus");
  }

  /* Call the function. */
  move_local_map_cplus_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, plhs, outputs);
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
  mexAtExit(&move_local_map_cplus_atexit);

  /* Module initialization. */
  move_local_map_cplus_initialize();

  /* Dispatch the entry-point. */
  c_move_local_map_cplus_mexFunct(nlhs, plhs, nrhs, prhs);

  /* Module termination. */
  move_local_map_cplus_terminate();
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
 * File trailer for _coder_move_local_map_cplus_mex.c
 *
 * [EOF]
 */
