//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: loadProblem.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "loadProblem.h"
#include "mpc_fun_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : f_struct_T *obj
//                int mIneq
//                int mLinIneq
//                const ::coder::array<double, 2U> &Aineq
//                const ::coder::array<double, 1U> &bineq
//                int mEq
//                int mLinEq
//                const ::coder::array<double, 2U> &Aeq
//                const ::coder::array<double, 1U> &beq
//                int mLB
//                const ::coder::array<double, 1U> &lb
//                int mUB
//                const ::coder::array<double, 1U> &ub
//                int mFixed
//                int mConstrMax
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void loadProblem(f_struct_T *obj, int mIneq, int mLinIneq,
                 const ::coder::array<double, 2U> &Aineq,
                 const ::coder::array<double, 1U> &bineq, int mEq, int mLinEq,
                 const ::coder::array<double, 2U> &Aeq,
                 const ::coder::array<double, 1U> &beq, int mLB,
                 const ::coder::array<double, 1U> &lb, int mUB,
                 const ::coder::array<double, 1U> &ub, int mFixed,
                 int mConstrMax)
{
  int idx_row;
  int k;
  int n;
  n = (((mIneq + mEq) + mLB) + mUB) + mFixed;
  obj->mConstr = n;
  obj->mConstrOrig = n;
  obj->mConstrMax = mConstrMax;
  obj->sizes[0] = mFixed;
  obj->sizes[1] = mEq;
  obj->sizes[2] = mIneq;
  obj->sizes[3] = mLB;
  obj->sizes[4] = mUB;
  obj->sizesPhaseOne[0] = mFixed;
  obj->sizesPhaseOne[1] = mEq;
  obj->sizesPhaseOne[2] = mIneq;
  obj->sizesPhaseOne[3] = mLB + 1;
  obj->sizesPhaseOne[4] = mUB;
  obj->sizesRegularized[0] = mFixed;
  obj->sizesRegularized[1] = mEq;
  obj->sizesRegularized[2] = mIneq;
  n = (mLB + mIneq) + (mEq << 1);
  obj->sizesRegularized[3] = n;
  obj->sizesRegularized[4] = mUB;
  obj->sizesRegPhaseOne[0] = mFixed;
  obj->sizesRegPhaseOne[1] = mEq;
  obj->sizesRegPhaseOne[2] = mIneq;
  obj->sizesRegPhaseOne[3] = n + 1;
  obj->sizesRegPhaseOne[4] = mUB;
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = mEq;
  obj->isActiveIdxRegPhaseOne[3] = mIneq;
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
  obj->isActiveIdxRegPhaseOne[2] = mEq;
  obj->isActiveIdxRegPhaseOne[3] = mIneq;
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
  obj->isActiveIdxRegPhaseOne[2] = mEq;
  obj->isActiveIdxRegPhaseOne[3] = mIneq;
  obj->isActiveIdxRegPhaseOne[4] = n;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (k = 0; k < 6; k++) {
    obj->isActiveIdxRegularized[k] = obj->isActiveIdxRegPhaseOne[k];
  }
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = mEq;
  obj->isActiveIdxRegPhaseOne[3] = mIneq;
  obj->isActiveIdxRegPhaseOne[4] = n + 1;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  if (mIneq > 0) {
    for (k = 0; k < mLinIneq; k++) {
      n = obj->nVar;
      for (idx_row = 0; idx_row < n; idx_row++) {
        obj->Aineq[idx_row + obj->ldA * k] = Aineq[k + mLinIneq * idx_row];
      }
    }
    if (bineq.size(0) != 0) {
      for (k = 0; k < mLinIneq; k++) {
        obj->bineq[k] = bineq[k];
      }
    }
  }
  if (mEq > 0) {
    for (k = 0; k < mLinEq; k++) {
      n = obj->nVar;
      for (idx_row = 0; idx_row < n; idx_row++) {
        obj->Aeq[idx_row + obj->ldA * k] = Aeq[k + mLinEq * idx_row];
      }
    }
    if (beq.size(0) != 0) {
      for (k = 0; k < mLinEq; k++) {
        obj->beq[k] = beq[k];
      }
    }
  }
  if (lb.size(0) != 0) {
    n = obj->nVar;
    for (k = 0; k < n; k++) {
      obj->lb[k] = -lb[k];
    }
  }
  if (ub.size(0) != 0) {
    n = obj->nVar;
    for (k = 0; k < n; k++) {
      obj->ub[k] = ub[k];
    }
  }
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for loadProblem.cpp
//
// [EOF]
//
