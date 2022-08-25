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
#include "MPCStepFunction_internal_types.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void loadProblem(g_struct_T *obj, const double Aeq[9600],
                 const double b_beq[80], int mLB, const double lb[120], int mUB,
                 const double ub[120], int mFixed)
{
  int k;
  k = ((mLB + mUB) + mFixed) + 80;
  obj->mConstr = k;
  obj->mConstrOrig = k;
  obj->mConstrMax = 321;
  obj->sizes[0] = mFixed;
  obj->sizes[1] = 80;
  obj->sizes[2] = 0;
  obj->sizes[3] = mLB;
  obj->sizes[4] = mUB;
  obj->sizesPhaseOne[0] = mFixed;
  obj->sizesPhaseOne[1] = 80;
  obj->sizesPhaseOne[2] = 0;
  obj->sizesPhaseOne[3] = mLB + 1;
  obj->sizesPhaseOne[4] = mUB;
  obj->sizesRegularized[0] = mFixed;
  obj->sizesRegularized[1] = 80;
  obj->sizesRegularized[2] = 0;
  obj->sizesRegularized[3] = mLB + 160;
  obj->sizesRegularized[4] = mUB;
  obj->sizesRegPhaseOne[0] = mFixed;
  obj->sizesRegPhaseOne[1] = 80;
  obj->sizesRegPhaseOne[2] = 0;
  obj->sizesRegPhaseOne[3] = mLB + 161;
  obj->sizesRegPhaseOne[4] = mUB;
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = 80;
  obj->isActiveIdxRegPhaseOne[3] = 0;
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
  obj->isActiveIdxRegPhaseOne[3] = 0;
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
  obj->isActiveIdxRegPhaseOne[3] = 0;
  obj->isActiveIdxRegPhaseOne[4] = mLB + 160;
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
  obj->isActiveIdxRegPhaseOne[3] = 0;
  obj->isActiveIdxRegPhaseOne[4] = mLB + 161;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (k = 0; k < 80; k++) {
    for (int idx_row{0}; idx_row < 120; idx_row++) {
      obj->Aeq[idx_row + 121 * k] = Aeq[k + 80 * idx_row];
    }
    obj->beq[k] = b_beq[k];
  }
  for (k = 0; k < 120; k++) {
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
