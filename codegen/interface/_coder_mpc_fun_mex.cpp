//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_mpc_fun_mex.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "_coder_mpc_fun_mex.h"
#include "_coder_mpc_fun_api.h"

// Function Definitions
//
// Arguments    : int32_T nlhs
//                mxArray *plhs[]
//                int32_T nrhs
//                const mxArray *prhs[]
// Return Type  : void
//
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&mpc_fun_atexit);
  // Module initialization.
  mpc_fun_initialize();
  try {
    emlrtShouldCleanupOnError((emlrtCTX *)emlrtRootTLSGlobal, false);
    // Dispatch the entry-point.
    unsafe_mpc_fun_mexFunction(nlhs, plhs, nrhs, prhs);
    // Module termination.
    mpc_fun_terminate();
  } catch (...) {
    emlrtCleanupOnException((emlrtCTX *)emlrtRootTLSGlobal);
    throw;
  }
}

//
// Arguments    : void
// Return Type  : emlrtCTX
//
emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2021a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           nullptr);
  return emlrtRootTLSGlobal;
}

//
// Arguments    : int32_T nlhs
//                mxArray *plhs[1]
//                int32_T nrhs
//                const mxArray *prhs[20]
// Return Type  : void
//
void unsafe_mpc_fun_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
                                const mxArray *prhs[20])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 20) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 20, 4,
                        7, "mpc_fun");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 7,
                        "mpc_fun");
  }
  // Call the function.
  mpc_fun_api(prhs, &outputs);
  // Copy over outputs to the caller.
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

//
// File trailer for _coder_mpc_fun_mex.cpp
//
// [EOF]
//
