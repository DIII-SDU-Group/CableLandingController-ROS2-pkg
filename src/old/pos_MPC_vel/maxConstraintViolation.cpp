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
#include "PositionMPCStepFunction_internal_types.h"
#include "PositionMPCStepFunction_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
double b_maxConstraintViolation(d_struct_T *obj, const double x[161])
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
    double c;
    int ia;
    v = 0.0;
    for (idxLB = 0; idxLB < 160; idxLB++) {
      obj->maxConstrWorkspace[idxLB] = obj->bineq[idxLB];
      obj->maxConstrWorkspace[idxLB] = -obj->maxConstrWorkspace[idxLB];
    }
    for (iac = 0; iac <= 25599; iac += 161) {
      c = 0.0;
      idxLB = iac + 160;
      for (ia = iac + 1; ia <= idxLB; ia++) {
        c += obj->Aineq[ia - 1] * x[(ia - iac) - 1];
      }
      idxLB = div_nde_s32_floor(iac, 161);
      obj->maxConstrWorkspace[idxLB] += c;
    }
    for (iac = 0; iac < 160; iac++) {
      obj->maxConstrWorkspace[iac] -= x[160];
      v = std::fmax(v, obj->maxConstrWorkspace[iac]);
    }
    for (idxLB = 0; idxLB < 80; idxLB++) {
      obj->maxConstrWorkspace[idxLB] = obj->beq[idxLB];
      obj->maxConstrWorkspace[idxLB] = -obj->maxConstrWorkspace[idxLB];
    }
    for (iac = 0; iac <= 12719; iac += 161) {
      c = 0.0;
      idxLB = iac + 160;
      for (ia = iac + 1; ia <= idxLB; ia++) {
        c += obj->Aeq[ia - 1] * x[(ia - iac) - 1];
      }
      idxLB = div_nde_s32_floor(iac, 161);
      obj->maxConstrWorkspace[idxLB] += c;
    }
    for (iac = 0; iac < 80; iac++) {
      obj->maxConstrWorkspace[iac] =
          (obj->maxConstrWorkspace[iac] - x[iac + 320]) + x[iac + 400];
      v = std::fmax(v, std::abs(obj->maxConstrWorkspace[iac]));
    }
  } break;
  default: {
    double c;
    int ia;
    v = 0.0;
    for (idxLB = 0; idxLB < 160; idxLB++) {
      obj->maxConstrWorkspace[idxLB] = obj->bineq[idxLB];
      obj->maxConstrWorkspace[idxLB] = -obj->maxConstrWorkspace[idxLB];
    }
    for (iac = 0; iac <= 25599; iac += 161) {
      c = 0.0;
      idxLB = iac + obj->nVar;
      for (ia = iac + 1; ia <= idxLB; ia++) {
        c += obj->Aineq[ia - 1] * x[(ia - iac) - 1];
      }
      idxLB = div_nde_s32_floor(iac, 161);
      obj->maxConstrWorkspace[idxLB] += c;
    }
    for (iac = 0; iac < 160; iac++) {
      v = std::fmax(v, obj->maxConstrWorkspace[iac]);
    }
    for (idxLB = 0; idxLB < 80; idxLB++) {
      obj->maxConstrWorkspace[idxLB] = obj->beq[idxLB];
      obj->maxConstrWorkspace[idxLB] = -obj->maxConstrWorkspace[idxLB];
    }
    for (iac = 0; iac <= 12719; iac += 161) {
      c = 0.0;
      idxLB = iac + obj->nVar;
      for (ia = iac + 1; ia <= idxLB; ia++) {
        c += obj->Aeq[ia - 1] * x[(ia - iac) - 1];
      }
      idxLB = div_nde_s32_floor(iac, 161);
      obj->maxConstrWorkspace[idxLB] += c;
    }
    for (iac = 0; iac < 80; iac++) {
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

double maxConstraintViolation(d_struct_T *obj, const double x[90321])
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
    double c;
    int ia;
    v = 0.0;
    for (idxLB = 0; idxLB < 160; idxLB++) {
      obj->maxConstrWorkspace[idxLB] = obj->bineq[idxLB];
      obj->maxConstrWorkspace[idxLB] = -obj->maxConstrWorkspace[idxLB];
    }
    for (iac = 0; iac <= 25599; iac += 161) {
      c = 0.0;
      idxLB = iac + 160;
      for (ia = iac + 1; ia <= idxLB; ia++) {
        c += obj->Aineq[ia - 1] * x[(ia - iac) + 560];
      }
      idxLB = div_nde_s32_floor(iac, 161);
      obj->maxConstrWorkspace[idxLB] += c;
    }
    for (iac = 0; iac < 160; iac++) {
      obj->maxConstrWorkspace[iac] -= x[iac + 721];
      v = std::fmax(v, obj->maxConstrWorkspace[iac]);
    }
    for (idxLB = 0; idxLB < 80; idxLB++) {
      obj->maxConstrWorkspace[idxLB] = obj->beq[idxLB];
      obj->maxConstrWorkspace[idxLB] = -obj->maxConstrWorkspace[idxLB];
    }
    for (iac = 0; iac <= 12719; iac += 161) {
      c = 0.0;
      idxLB = iac + 160;
      for (ia = iac + 1; ia <= idxLB; ia++) {
        c += obj->Aeq[ia - 1] * x[(ia - iac) + 560];
      }
      idxLB = div_nde_s32_floor(iac, 161);
      obj->maxConstrWorkspace[idxLB] += c;
    }
    for (iac = 0; iac < 80; iac++) {
      obj->maxConstrWorkspace[iac] =
          (obj->maxConstrWorkspace[iac] - x[iac + 881]) + x[iac + 961];
      v = std::fmax(v, std::abs(obj->maxConstrWorkspace[iac]));
    }
  } break;
  default: {
    double c;
    int ia;
    v = 0.0;
    for (idxLB = 0; idxLB < 160; idxLB++) {
      obj->maxConstrWorkspace[idxLB] = obj->bineq[idxLB];
      obj->maxConstrWorkspace[idxLB] = -obj->maxConstrWorkspace[idxLB];
    }
    for (iac = 0; iac <= 25599; iac += 161) {
      c = 0.0;
      idxLB = iac + obj->nVar;
      for (ia = iac + 1; ia <= idxLB; ia++) {
        c += obj->Aineq[ia - 1] * x[(ia - iac) + 560];
      }
      idxLB = div_nde_s32_floor(iac, 161);
      obj->maxConstrWorkspace[idxLB] += c;
    }
    for (iac = 0; iac < 160; iac++) {
      v = std::fmax(v, obj->maxConstrWorkspace[iac]);
    }
    for (idxLB = 0; idxLB < 80; idxLB++) {
      obj->maxConstrWorkspace[idxLB] = obj->beq[idxLB];
      obj->maxConstrWorkspace[idxLB] = -obj->maxConstrWorkspace[idxLB];
    }
    for (iac = 0; iac <= 12719; iac += 161) {
      c = 0.0;
      idxLB = iac + obj->nVar;
      for (ia = iac + 1; ia <= idxLB; ia++) {
        c += obj->Aeq[ia - 1] * x[(ia - iac) + 560];
      }
      idxLB = div_nde_s32_floor(iac, 161);
      obj->maxConstrWorkspace[idxLB] += c;
    }
    for (iac = 0; iac < 80; iac++) {
      v = std::fmax(v, std::abs(obj->maxConstrWorkspace[iac]));
    }
  } break;
  }
  if (obj->sizes[3] > 0) {
    for (iac = 0; iac < mLB; iac++) {
      idxLB = obj->indexLB[iac];
      v = std::fmax(v, -x[idxLB + 560] - obj->lb[idxLB - 1]);
    }
  }
  if (obj->sizes[4] > 0) {
    for (iac = 0; iac < mUB; iac++) {
      idxLB = obj->indexUB[iac];
      v = std::fmax(v, x[idxLB + 560] - obj->ub[idxLB - 1]);
    }
  }
  if (obj->sizes[0] > 0) {
    for (iac = 0; iac < mFixed; iac++) {
      v = std::fmax(v, std::abs(x[obj->indexFixed[iac] + 560] -
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
