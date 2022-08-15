//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mpc_fun_initialize.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "mpc_fun_initialize.h"
#include "mpc_fun.h"
#include "mpc_fun_data.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void mpc_fun_initialize()
{
  B_not_empty_init();
  N_variables_not_empty_init();
  N_not_empty_init();
  mpc_fun_init();
  isInitialized_mpc_fun = true;
}

//
// File trailer for mpc_fun_initialize.cpp
//
// [EOF]
//
