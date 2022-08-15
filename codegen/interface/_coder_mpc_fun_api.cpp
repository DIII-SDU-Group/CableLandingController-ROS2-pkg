//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_mpc_fun_api.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "_coder_mpc_fun_api.h"
#include "_coder_mpc_fun_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131610U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "mpc_fun",                                            // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *dt,
                                 const char_T *identifier);

static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[6];

static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *x,
                                 const char_T *identifier))[6];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[6];

static void emlrt_marshallOut(const real_T u[6], const mxArray *y);

// Function Definitions
//
// Arguments    : const emlrtStack *sp
//                const mxArray *dt
//                const char_T *identifier
// Return Type  : real_T
//
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *dt,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(dt), &thisId);
  emlrtDestroyArray(&dt);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T
//
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = d_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[6]
//
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[6]
{
  static const int32_T dims{6};
  real_T(*ret)[6];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  ret = (real_T(*)[6])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T
//
static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 0U, (void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *x
//                const char_T *identifier
// Return Type  : real_T (*)[6]
//
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *x,
                                 const char_T *identifier))[6]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[6];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(sp, emlrtAlias(x), &thisId);
  emlrtDestroyArray(&x);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[6]
//
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[6]
{
  real_T(*y)[6];
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const real_T u[6]
//                const mxArray *y
// Return Type  : void
//
static void emlrt_marshallOut(const real_T u[6], const mxArray *y)
{
  static const int32_T i{6};
  emlrtMxSetData((mxArray *)y, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)y, &i, 1);
}

//
// Arguments    : const mxArray * const prhs[20]
//                const mxArray **plhs
// Return Type  : void
//
void mpc_fun_api(const mxArray *const prhs[20], const mxArray **plhs)
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *prhs_copy_idx_0;
  real_T(*x)[6];
  real_T approach_region_slope;
  real_T approach_width;
  real_T apsi_max;
  real_T ay_max;
  real_T az_max;
  real_T dt;
  real_T hp;
  real_T min_cable_height;
  real_T vpsi_max;
  real_T vy_max;
  real_T vz_max;
  real_T we_approach_region;
  real_T we_ground;
  real_T wpsi;
  real_T wvpsi;
  real_T wvy;
  real_T wvz;
  real_T wy;
  real_T wz;
  st.tls = emlrtRootTLSGlobal;
  prhs_copy_idx_0 = emlrtProtectR2012b(prhs[0], 0, true, -1);
  // Marshall function inputs
  x = emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_0), "x");
  dt = b_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "dt");
  hp = b_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "hp");
  vy_max = b_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "vy_max");
  vz_max = b_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "vz_max");
  vpsi_max = b_emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "vpsi_max");
  ay_max = b_emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "ay_max");
  az_max = b_emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "az_max");
  apsi_max = b_emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "apsi_max");
  approach_width =
      b_emlrt_marshallIn(&st, emlrtAliasP(prhs[9]), "approach_width");
  approach_region_slope =
      b_emlrt_marshallIn(&st, emlrtAliasP(prhs[10]), "approach_region_slope");
  min_cable_height =
      b_emlrt_marshallIn(&st, emlrtAliasP(prhs[11]), "min_cable_height");
  wy = b_emlrt_marshallIn(&st, emlrtAliasP(prhs[12]), "wy");
  wz = b_emlrt_marshallIn(&st, emlrtAliasP(prhs[13]), "wz");
  wpsi = b_emlrt_marshallIn(&st, emlrtAliasP(prhs[14]), "wpsi");
  wvy = b_emlrt_marshallIn(&st, emlrtAliasP(prhs[15]), "wvy");
  wvz = b_emlrt_marshallIn(&st, emlrtAliasP(prhs[16]), "wvz");
  wvpsi = b_emlrt_marshallIn(&st, emlrtAliasP(prhs[17]), "wvpsi");
  we_approach_region =
      b_emlrt_marshallIn(&st, emlrtAliasP(prhs[18]), "we_approach_region");
  we_ground = b_emlrt_marshallIn(&st, emlrtAliasP(prhs[19]), "we_ground");
  // Invoke the target function
  mpc_fun(*x, dt, hp, vy_max, vz_max, vpsi_max, ay_max, az_max, apsi_max,
          approach_width, approach_region_slope, min_cable_height, wy, wz, wpsi,
          wvy, wvz, wvpsi, we_approach_region, we_ground);
  // Marshall function outputs
  emlrt_marshallOut(*x, prhs_copy_idx_0);
  *plhs = prhs_copy_idx_0;
}

//
// Arguments    : void
// Return Type  : void
//
void mpc_fun_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  mpc_fun_xil_terminate();
  mpc_fun_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void mpc_fun_initialize()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void mpc_fun_terminate()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

//
// File trailer for _coder_mpc_fun_api.cpp
//
// [EOF]
//
