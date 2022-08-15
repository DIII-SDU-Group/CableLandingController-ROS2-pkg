//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeFval.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "computeFval.h"
#include "linearForm_.h"
#include "mpc_fun_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : const c_struct_T *obj
//                ::coder::array<double, 2U> &workspace
//                const ::coder::array<double, 2U> &H
//                const ::coder::array<double, 1U> &x
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
double computeFval(const c_struct_T *obj, ::coder::array<double, 2U> &workspace,
                   const ::coder::array<double, 2U> &H,
                   const ::coder::array<double, 1U> &x)
{
  double val;
  switch (obj->objtype) {
  case 5:
    val = x[obj->nvar - 1];
    break;
  case 3: {
    linearForm_(obj->hasLinear, obj->nvar, workspace, H, x);
    val = 0.0;
    if (obj->nvar >= 1) {
      int ixlast;
      ixlast = obj->nvar;
      for (int k{0}; k < ixlast; k++) {
        val += x[k] * workspace[k];
      }
    }
  } break;
  default: {
    int ixlast;
    int k;
    linearForm_(obj->hasLinear, obj->nvar, workspace, H, x);
    ixlast = obj->nvar + 1;
    k = obj->maxVar - 1;
    for (int idx{ixlast}; idx <= k; idx++) {
      workspace[idx - 1] = 0.0 * x[idx - 1];
    }
    val = 0.0;
    if (k >= 1) {
      ixlast = obj->maxVar;
      for (k = 0; k <= ixlast - 2; k++) {
        val += x[k] * workspace[k];
      }
    }
  } break;
  }
  return val;
}

} // namespace Objective
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeFval.cpp
//
// [EOF]
//
