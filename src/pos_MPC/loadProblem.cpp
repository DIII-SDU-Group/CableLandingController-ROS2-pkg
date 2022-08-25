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
void loadProblem(g_struct_T *obj, const double Aeq[21600],
                 const double b_beq[120], int mLB, const double lb[180],
                 int mUB, const double ub[180], int mFixed)
{
  int k;
  k = ((mLB + mUB) + mFixed) + 120;
  obj->mConstr = k;
  obj->mConstrOrig = k;
  obj->mConstrMax = 481;
  obj->sizes[0] = mFixed;
  obj->sizes[1] = 120;
  obj->sizes[2] = 0;
  obj->sizes[3] = mLB;
  obj->sizes[4] = mUB;
  obj->sizesPhaseOne[0] = mFixed;
  obj->sizesPhaseOne[1] = 120;
  obj->sizesPhaseOne[2] = 0;
  obj->sizesPhaseOne[3] = mLB + 1;
  obj->sizesPhaseOne[4] = mUB;
  obj->sizesRegularized[0] = mFixed;
  obj->sizesRegularized[1] = 120;
  obj->sizesRegularized[2] = 0;
  obj->sizesRegularized[3] = mLB + 240;
  obj->sizesRegularized[4] = mUB;
  obj->sizesRegPhaseOne[0] = mFixed;
  obj->sizesRegPhaseOne[1] = 120;
  obj->sizesRegPhaseOne[2] = 0;
  obj->sizesRegPhaseOne[3] = mLB + 241;
  obj->sizesRegPhaseOne[4] = mUB;
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = 120;
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
  obj->isActiveIdxRegPhaseOne[2] = 120;
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
  obj->isActiveIdxRegPhaseOne[2] = 120;
  obj->isActiveIdxRegPhaseOne[3] = 0;
  obj->isActiveIdxRegPhaseOne[4] = mLB + 240;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (k = 0; k < 6; k++) {
    obj->isActiveIdxRegularized[k] = obj->isActiveIdxRegPhaseOne[k];
  }
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = 120;
  obj->isActiveIdxRegPhaseOne[3] = 0;
  obj->isActiveIdxRegPhaseOne[4] = mLB + 241;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (k = 0; k < 120; k++) {
    for (int idx_row{0}; idx_row < 180; idx_row++) {
      obj->Aeq[idx_row + 181 * k] = Aeq[k + 120 * idx_row];
    }
    obj->beq[k] = b_beq[k];
  }
  for (k = 0; k < 180; k++) {
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
