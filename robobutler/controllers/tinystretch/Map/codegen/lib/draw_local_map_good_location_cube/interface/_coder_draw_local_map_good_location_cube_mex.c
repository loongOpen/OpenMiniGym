/*
 * File: _coder_draw_local_map_good_location_cube_mex.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 27-Sep-2022 17:30:11
 */

/* Include Files */
#include "_coder_draw_local_map_good_location_cube_mex.h"
#include "_coder_draw_local_map_good_location_cube_api.h"

/* Function Declarations */
MEXFUNCTION_LINKAGE void c_draw_local_map_good_location_(int32_T nlhs, mxArray
  *plhs[2], int32_T nrhs, const mxArray *prhs[3]);

/* Function Definitions */

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[2]
 *                int32_T nrhs
 *                const mxArray *prhs[3]
 * Return Type  : void
 */
void c_draw_local_map_good_location_(int32_T nlhs, mxArray *plhs[2], int32_T
  nrhs, const mxArray *prhs[3])
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
                        33, "draw_local_map_good_location_cube");
  }

  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 33,
                        "draw_local_map_good_location_cube");
  }

  /* Call the function. */
  draw_local_map_good_location_cube_api(prhs, nlhs, outputs);

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
  mexAtExit(&draw_local_map_good_location_cube_atexit);

  /* Module initialization. */
  draw_local_map_good_location_cube_initialize();

  /* Dispatch the entry-point. */
  c_draw_local_map_good_location_(nlhs, plhs, nrhs, prhs);

  /* Module termination. */
  draw_local_map_good_location_cube_terminate();
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
 * File trailer for _coder_draw_local_map_good_location_cube_mex.c
 *
 * [EOF]
 */
