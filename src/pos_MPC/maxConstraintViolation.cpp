//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// maxConstraintViolation.cpp
//
// Code generation for function 'maxConstraintViolation'
//

// Include files
#include "maxConstraintViolation.h"
#include "MPCStepFunction_internal_types.h"
#include "MPCStepFunction_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
double maxConstraintViolation(g_struct_T *obj, const double x[181])
{
  double v;
  int iac;
  int idxLB;
  int mFixed;
  int mLB;
  int mUB;
  mLB = obj->sizes[3];
  mUB = obj->sizes[4];
  mFixed = obj->sizes[0];
  switch (obj->probType) {
  case 2: {
    v = 0.0;
    for (idxLB = 0; idxLB < 120; idxLB++) {
      obj->maxConstrWorkspace[idxLB] = obj->beq[idxLB];
      obj->maxConstrWorkspace[idxLB] = -obj->maxConstrWorkspace[idxLB];
    }
    for (iac = 0; iac <= 21539; iac += 181) {
      double c;
      c = 0.0;
      idxLB = iac + 180;
      for (int ia{iac + 1}; ia <= idxLB; ia++) {
        c += obj->Aeq[ia - 1] * x[(ia - iac) - 1];
      }
      idxLB = div_nde_s32_floor(iac, 181);
      obj->maxConstrWorkspace[idxLB] += c;
    }
    for (iac = 0; iac < 120; iac++) {
      obj->maxConstrWorkspace[iac] =
          (obj->maxConstrWorkspace[iac] - x[180]) + x[iac + 300];
      v = std::fmax(v, std::abs(obj->maxConstrWorkspace[iac]));
    }
  } break;
  default: {
    v = 0.0;
    for (idxLB = 0; idxLB < 120; idxLB++) {
      obj->maxConstrWorkspace[idxLB] = obj->beq[idxLB];
      obj->maxConstrWorkspace[idxLB] = -obj->maxConstrWorkspace[idxLB];
    }
    for (iac = 0; iac <= 21539; iac += 181) {
      double c;
      c = 0.0;
      idxLB = iac + obj->nVar;
      for (int ia{iac + 1}; ia <= idxLB; ia++) {
        c += obj->Aeq[ia - 1] * x[(ia - iac) - 1];
      }
      idxLB = div_nde_s32_floor(iac, 181);
      obj->maxConstrWorkspace[idxLB] += c;
    }
    for (iac = 0; iac < 120; iac++) {
      v = std::fmax(v, std::abs(obj->maxConstrWorkspace[iac]));
    }
  } break;
  }
  if (obj->sizes[3] > 0) {
    for (iac = 0; iac < mLB; iac++) {
      idxLB = obj->indexLB[iac] - 1;
      v = std::fmax(v, -x[idxLB] - obj->lb[idxLB]);
    }
  }
  if (obj->sizes[4] > 0) {
    for (iac = 0; iac < mUB; iac++) {
      idxLB = obj->indexUB[iac] - 1;
      v = std::fmax(v, x[idxLB] - obj->ub[idxLB]);
    }
  }
  if (obj->sizes[0] > 0) {
    for (iac = 0; iac < mFixed; iac++) {
      v = std::fmax(v, std::abs(x[obj->indexFixed[iac] - 1] -
                                obj->ub[obj->indexFixed[iac] - 1]));
    }
  }
  return v;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (maxConstraintViolation.cpp)
