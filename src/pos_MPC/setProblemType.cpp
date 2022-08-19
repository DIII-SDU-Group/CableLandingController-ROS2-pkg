//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// setProblemType.cpp
//
// Code generation for function 'setProblemType'
//

// Include files
#include "setProblemType.h"
#include "PositionMPCStepFunction_internal_types.h"
#include "modifyOverheadPhaseOne_.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void setProblemType(d_struct_T *obj, int PROBLEM_TYPE)
{
  switch (PROBLEM_TYPE) {
  case 3: {
    int i;
    obj->nVar = 160;
    obj->mConstr = obj->mConstrOrig;
    if (obj->nWConstr[4] > 0) {
      i = obj->sizesNormal[4];
      for (int colOffsetAeq{0}; colOffsetAeq < i; colOffsetAeq++) {
        obj->isActiveConstr[(obj->isActiveIdxNormal[4] + colOffsetAeq) - 1] =
            obj->isActiveConstr[(obj->isActiveIdx[4] + colOffsetAeq) - 1];
      }
    }
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesNormal[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxNormal[i];
    }
  } break;
  case 1: {
    int i;
    obj->nVar = 161;
    obj->mConstr = obj->mConstrOrig + 1;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxPhaseOne[i];
    }
    modifyOverheadPhaseOne_(obj);
  } break;
  case 2: {
    int i;
    obj->nVar = 160;
    obj->mConstr = 560;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegularized[i];
    }
    if (obj->probType != 4) {
      int colOffsetATw;
      int colOffsetAeq;
      int colOffsetAineq;
      int i1;
      int idx_col;
      for (idx_col = 0; idx_col < 160; idx_col++) {
        colOffsetAineq = 161 * idx_col;
        i = idx_col + 160;
        if (161 <= i) {
          std::memset(&obj->Aineq[colOffsetAineq + 160], 0,
                      (((i + colOffsetAineq) - colOffsetAineq) + -160) *
                          sizeof(double));
        }
        obj->Aineq[(idx_col + colOffsetAineq) + 160] = -1.0;
      }
      for (idx_col = 0; idx_col < 80; idx_col++) {
        colOffsetAineq = idx_col + 320;
        colOffsetAeq = 161 * idx_col - 1;
        colOffsetATw = colOffsetAeq + 161 * (obj->isActiveIdx[1] - 1);
        std::memset(&obj->Aeq[colOffsetAeq + 161], 0, 160U * sizeof(double));
        std::memset(&obj->ATwset[colOffsetATw + 161], 0, 160U * sizeof(double));
        if (321 <= colOffsetAineq) {
          std::memset(
              &obj->Aeq[colOffsetAeq + 321], 0,
              (((colOffsetAineq + colOffsetAeq) - colOffsetAeq) + -320) *
                  sizeof(double));
          std::memset(
              &obj->ATwset[colOffsetATw + 321], 0,
              (((colOffsetAineq + colOffsetATw) - colOffsetATw) + -320) *
                  sizeof(double));
        }
        i = idx_col + colOffsetAeq;
        obj->Aeq[i + 321] = -1.0;
        obj->ATwset[(idx_col + colOffsetATw) + 321] = -1.0;
        i1 = idx_col + 322;
        if (i1 <= 400) {
          std::memset(&obj->Aeq[i1 + colOffsetAeq], 0,
                      (((colOffsetAeq - i1) - colOffsetAeq) + 401) *
                          sizeof(double));
          std::memset(&obj->ATwset[i1 + colOffsetATw], 0,
                      (((colOffsetATw - i1) - colOffsetATw) + 401) *
                          sizeof(double));
        }
        i1 = idx_col + 400;
        if (401 <= i1) {
          std::memset(&obj->Aeq[colOffsetAeq + 401], 0,
                      (((i1 + colOffsetAeq) - colOffsetAeq) + -400) *
                          sizeof(double));
          std::memset(&obj->ATwset[colOffsetATw + 401], 0,
                      (((i1 + colOffsetATw) - colOffsetATw) + -400) *
                          sizeof(double));
        }
        obj->Aeq[i + 401] = 1.0;
        obj->ATwset[(idx_col + colOffsetATw) + 401] = 1.0;
      }
      colOffsetAineq = 160;
      i = obj->sizesNormal[3] + 1;
      i1 = obj->sizesRegularized[3];
      for (colOffsetAeq = i; colOffsetAeq <= i1; colOffsetAeq++) {
        colOffsetAineq++;
        obj->indexLB[colOffsetAeq - 1] = colOffsetAineq;
      }
      if (obj->nWConstr[4] > 0) {
        i = obj->sizesRegularized[4];
        for (colOffsetAeq = 0; colOffsetAeq < i; colOffsetAeq++) {
          obj->isActiveConstr[obj->isActiveIdxRegularized[4] + colOffsetAeq] =
              obj->isActiveConstr[(obj->isActiveIdx[4] + colOffsetAeq) - 1];
        }
      }
      i = obj->isActiveIdx[4];
      i1 = obj->isActiveIdxRegularized[4] - 1;
      if (i <= i1) {
        std::memset(&obj->isActiveConstr[i + -1], 0,
                    ((i1 - i) + 1) * sizeof(boolean_T));
      }
      obj->lb[160] = 0.0;
      colOffsetAineq = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (idx_col = colOffsetAineq; idx_col <= i; idx_col++) {
        colOffsetATw = 161 * (idx_col - 1) - 1;
        switch (obj->Wid[idx_col - 1]) {
        case 3:
          i1 = obj->Wlocalidx[idx_col - 1] + 159;
          if (161 <= i1) {
            std::memset(&obj->ATwset[colOffsetATw + 161], 0,
                        (((i1 + colOffsetATw) - colOffsetATw) + -160) *
                            sizeof(double));
          }
          obj->ATwset[(obj->Wlocalidx[idx_col - 1] + colOffsetATw) + 160] =
              -1.0;
          i1 = obj->Wlocalidx[idx_col - 1] + 161;
          if (i1 <= 160) {
            std::memset(&obj->ATwset[i1 + colOffsetATw], 0,
                        (((colOffsetATw - i1) - colOffsetATw) + 161) *
                            sizeof(double));
          }
          break;
        }
      }
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegularized[i];
    }
  } break;
  default: {
    int i;
    obj->nVar = 161;
    obj->mConstr = 561;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegPhaseOne[i];
    }
    modifyOverheadPhaseOne_(obj);
  } break;
  }
  obj->probType = PROBLEM_TYPE;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (setProblemType.cpp)
