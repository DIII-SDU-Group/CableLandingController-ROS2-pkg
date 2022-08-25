//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeFval.cpp
//
// Code generation for function 'computeFval'
//

// Include files
#include "computeFval.h"
#include "MPCStepFunction_internal_types.h"
#include "rt_nonfinite.h"
#include <algorithm>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
double computeFval(const b_struct_T *obj, double workspace[87061],
                   const double H[32400], const double f[180],
                   const double x[181])
{
  double val;
  switch (obj->objtype) {
  case 5:
    val = x[obj->nvar - 1];
    break;
  case 3: {
    int i;
    int ix;
    int ixlast;
    i = obj->nvar;
    if (0 <= i - 1) {
      std::copy(&f[0], &f[i], &workspace[0]);
    }
    ixlast = obj->nvar;
    ix = 0;
    i = obj->nvar * (obj->nvar - 1) + 1;
    for (int iac{1}; ixlast < 0 ? iac >= i : iac <= i; iac += ixlast) {
      double c;
      int i1;
      c = 0.5 * x[ix];
      i1 = (iac + obj->nvar) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        int i2;
        i2 = ia - iac;
        workspace[i2] += H[ia - 1] * c;
      }
      ix++;
    }
    val = 0.0;
    if (obj->nvar >= 1) {
      ixlast = obj->nvar;
      for (ix = 0; ix < ixlast; ix++) {
        val += x[ix] * workspace[ix];
      }
    }
  } break;
  default: {
    int i;
    int ix;
    int ixlast;
    ixlast = obj->nvar;
    if (0 <= ixlast - 1) {
      std::copy(&f[0], &f[ixlast], &workspace[0]);
    }
    ix = 0;
    i = obj->nvar * (obj->nvar - 1) + 1;
    for (int iac{1}; ixlast < 0 ? iac >= i : iac <= i; iac += ixlast) {
      double c;
      int i1;
      c = 0.5 * x[ix];
      i1 = (iac + obj->nvar) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        int i2;
        i2 = ia - iac;
        workspace[i2] += H[ia - 1] * c;
      }
      ix++;
    }
    val = 0.0;
    for (ix = 0; ix < 180; ix++) {
      val += x[ix] * workspace[ix];
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

// End of code generation (computeFval.cpp)
