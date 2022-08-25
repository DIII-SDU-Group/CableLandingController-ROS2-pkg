//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeFval_ReuseHx.cpp
//
// Code generation for function 'computeFval_ReuseHx'
//

// Include files
#include "computeFval_ReuseHx.h"
#include "PositionMPCStepFunction_internal_types.h"
#include "rt_nonfinite.h"
#include <algorithm>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
double computeFval_ReuseHx(const b_struct_T *obj, double workspace[90321],
                           const double f[160], const double x[161])
{
  double val;
  switch (obj->objtype) {
  case 5:
    val = obj->gammaScalar * x[obj->nvar - 1];
    break;
  case 3: {
    if (obj->hasLinear) {
      int ixlast;
      int k;
      ixlast = obj->nvar;
      for (k = 0; k < ixlast; k++) {
        workspace[k] = 0.5 * obj->Hx[k] + f[k];
      }
      val = 0.0;
      if (obj->nvar >= 1) {
        ixlast = obj->nvar;
        for (k = 0; k < ixlast; k++) {
          val += x[k] * workspace[k];
        }
      }
    } else {
      val = 0.0;
      if (obj->nvar >= 1) {
        int ixlast;
        ixlast = obj->nvar;
        for (int k{0}; k < ixlast; k++) {
          val += x[k] * obj->Hx[k];
        }
      }
      val *= 0.5;
    }
  } break;
  default: {
    if (obj->hasLinear) {
      int ixlast;
      int k;
      ixlast = obj->nvar;
      if (0 <= ixlast - 1) {
        std::copy(&f[0], &f[ixlast], &workspace[0]);
      }
      ixlast = 159 - obj->nvar;
      for (k = 0; k <= ixlast; k++) {
        workspace[obj->nvar + k] = 0.0;
      }
      val = 0.0;
      for (k = 0; k < 160; k++) {
        workspace[k] += 0.5 * obj->Hx[k];
        val += x[k] * workspace[k];
      }
    } else {
      int ixlast;
      int k;
      val = 0.0;
      for (k = 0; k < 160; k++) {
        val += x[k] * obj->Hx[k];
      }
      val *= 0.5;
      ixlast = obj->nvar + 1;
      for (k = ixlast; k < 161; k++) {
        val += x[k - 1] * 0.0;
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
} // namespace pos_MPC

// End of code generation (computeFval_ReuseHx.cpp)
