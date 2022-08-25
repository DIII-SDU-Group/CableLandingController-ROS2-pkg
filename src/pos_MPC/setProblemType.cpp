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
#include "MPCStepFunction_internal_types.h"
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
void setProblemType(g_struct_T *obj, int PROBLEM_TYPE)
{
  switch (PROBLEM_TYPE) {
  case 3: {
    int i;
    obj->nVar = 180;
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
    obj->nVar = 181;
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
    obj->nVar = 180;
    obj->mConstr = 480;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegularized[i];
    }
    if (obj->probType != 4) {
      int colOffsetATw;
      int colOffsetAeq;
      int i1;
      int idx_col;
      int idx_lb;
      for (idx_col = 0; idx_col < 120; idx_col++) {
        idx_lb = idx_col + 180;
        colOffsetAeq = 181 * idx_col - 1;
        colOffsetATw = colOffsetAeq + 181 * (obj->isActiveIdx[1] - 1);
        if (181 <= idx_lb) {
          std::memset(&obj->Aeq[colOffsetAeq + 181], 0,
                      (((idx_lb + colOffsetAeq) - colOffsetAeq) + -180) *
                          sizeof(double));
          std::memset(&obj->ATwset[colOffsetATw + 181], 0,
                      (((idx_lb + colOffsetATw) - colOffsetATw) + -180) *
                          sizeof(double));
        }
        i = idx_col + colOffsetAeq;
        obj->Aeq[i + 181] = -1.0;
        i1 = idx_col + colOffsetATw;
        obj->ATwset[i1 + 181] = -1.0;
        idx_lb = idx_col + 182;
        if (idx_lb <= 300) {
          std::memset(&obj->Aeq[idx_lb + colOffsetAeq], 0,
                      (((colOffsetAeq - idx_lb) - colOffsetAeq) + 301) *
                          sizeof(double));
          std::memset(&obj->ATwset[idx_lb + colOffsetATw], 0,
                      (((colOffsetATw - idx_lb) - colOffsetATw) + 301) *
                          sizeof(double));
        }
        idx_lb = idx_col + 300;
        if (301 <= idx_lb) {
          std::memset(&obj->Aeq[colOffsetAeq + 301], 0,
                      (((idx_lb + colOffsetAeq) - colOffsetAeq) + -300) *
                          sizeof(double));
          std::memset(&obj->ATwset[colOffsetATw + 301], 0,
                      (((idx_lb + colOffsetATw) - colOffsetATw) + -300) *
                          sizeof(double));
        }
        obj->Aeq[i + 301] = 1.0;
        obj->ATwset[i1 + 301] = 1.0;
      }
      idx_lb = 180;
      i = obj->sizesNormal[3] + 1;
      i1 = obj->sizesRegularized[3];
      for (colOffsetAeq = i; colOffsetAeq <= i1; colOffsetAeq++) {
        idx_lb++;
        obj->indexLB[colOffsetAeq - 1] = idx_lb;
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
      obj->lb[180] = 0.0;
      idx_lb = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (idx_col = idx_lb; idx_col <= i; idx_col++) {
        colOffsetATw = 181 * (idx_col - 1) - 1;
        switch (obj->Wid[idx_col - 1]) {
        case 3:
          i1 = obj->Wlocalidx[idx_col - 1] + 179;
          if (181 <= i1) {
            std::memset(&obj->ATwset[colOffsetATw + 181], 0,
                        (((i1 + colOffsetATw) - colOffsetATw) + -180) *
                            sizeof(double));
          }
          obj->ATwset[(obj->Wlocalidx[idx_col - 1] + colOffsetATw) + 180] =
              -1.0;
          i1 = obj->Wlocalidx[idx_col - 1] + 181;
          if (i1 <= 180) {
            std::memset(&obj->ATwset[i1 + colOffsetATw], 0,
                        (((colOffsetATw - i1) - colOffsetATw) + 181) *
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
    obj->nVar = 181;
    obj->mConstr = 481;
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
