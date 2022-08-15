//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_mpc_fun_api.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef _CODER_MPC_FUN_API_H
#define _CODER_MPC_FUN_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void mpc_fun(real_T x[6], real_T dt, real_T hp, real_T vy_max, real_T vz_max,
             real_T vpsi_max, real_T ay_max, real_T az_max, real_T apsi_max,
             real_T approach_width, real_T approach_region_slope,
             real_T min_cable_height, real_T wy, real_T wz, real_T wpsi,
             real_T wvy, real_T wvz, real_T wvpsi, real_T we_approach_region,
             real_T we_ground);

void mpc_fun_api(const mxArray *const prhs[20], const mxArray **plhs);

void mpc_fun_atexit();

void mpc_fun_initialize();

void mpc_fun_terminate();

void mpc_fun_xil_shutdown();

void mpc_fun_xil_terminate();

#endif
//
// File trailer for _coder_mpc_fun_api.h
//
// [EOF]
//
