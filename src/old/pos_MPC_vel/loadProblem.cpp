//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// loadProblem.cpp
//
// Code generation for function 'loadProblem'
//

// Include files
#include "loadProblem.h"
#include "PositionMPCStepFunction_internal_types.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void loadProblem(d_struct_T *obj, const double Aineq[25600],
                 const double bineq[160], const double Aeq[12800],
                 const double b_beq[80], int mLB, const double lb[160], int mUB,
                 const double ub[160], int mFixed)
{
  int idx_row;
  int k;
  k = ((mLB + mUB) + mFixed) + 240;
  obj->mConstr = k;
  obj->mConstrOrig = k;
  obj->mConstrMax = 561;
  obj->sizes[0] = mFixed;
  obj->sizes[1] = 80;
  obj->sizes[2] = 160;
  obj->sizes[3] = mLB;
  obj->sizes[4] = mUB;
  obj->sizesPhaseOne[0] = mFixed;
  obj->sizesPhaseOne[1] = 80;
  obj->sizesPhaseOne[2] = 160;
  obj->sizesPhaseOne[3] = mLB + 1;
  obj->sizesPhaseOne[4] = mUB;
  obj->sizesRegularized[0] = mFixed;
  obj->sizesRegularized[1] = 80;
  obj->sizesRegularized[2] = 160;
  obj->sizesRegularized[3] = mLB + 320;
  obj->sizesRegularized[4] = mUB;
  obj->sizesRegPhaseOne[0] = mFixed;
  obj->sizesRegPhaseOne[1] = 80;
  obj->sizesRegPhaseOne[2] = 160;
  obj->sizesRegPhaseOne[3] = mLB + 321;
  obj->sizesRegPhaseOne[4] = mUB;
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = 80;
  obj->isActiveIdxRegPhaseOne[3] = 160;
  obj->isActiveIdxRegPhaseOne[4] = mLB;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  for (k = 0; k < 5; k++) {
    obj->sizesNormal[k] = obj->sizes[k];
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (k = 0; k < 6; k++) {
    obj->isActiveIdx[k] = obj->isActiveIdxRegPhaseOne[k];
    obj->isActiveIdxNormal[k] = obj->isActiveIdxRegPhaseOne[k];
  }
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = 80;
  obj->isActiveIdxRegPhaseOne[3] = 160;
  obj->isActiveIdxRegPhaseOne[4] = mLB + 1;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (k = 0; k < 6; k++) {
    obj->isActiveIdxPhaseOne[k] = obj->isActiveIdxRegPhaseOne[k];
  }
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = 80;
  obj->isActiveIdxRegPhaseOne[3] = 160;
  obj->isActiveIdxRegPhaseOne[4] = mLB + 320;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (k = 0; k < 6; k++) {
    obj->isActiveIdxRegularized[k] = obj->isActiveIdxRegPhaseOne[k];
  }
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = 80;
  obj->isActiveIdxRegPhaseOne[3] = 160;
  obj->isActiveIdxRegPhaseOne[4] = mLB + 321;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (k = 0; k < 160; k++) {
    for (idx_row = 0; idx_row < 160; idx_row++) {
      obj->Aineq[idx_row + 161 * k] = Aineq[k + 160 * idx_row];
    }
    obj->bineq[k] = bineq[k];
  }
  for (k = 0; k < 80; k++) {
    for (idx_row = 0; idx_row < 160; idx_row++) {
      obj->Aeq[idx_row + 161 * k] = Aeq[k + 80 * idx_row];
    }
    obj->beq[k] = b_beq[k];
  }
  for (k = 0; k < 160; k++) {
    obj->lb[k] = -lb[k];
    obj->ub[k] = ub[k];
  }
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (loadProblem.cpp)
