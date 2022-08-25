//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// modifyOverheadPhaseOne_.cpp
//
// Code generation for function 'modifyOverheadPhaseOne_'
//

// Include files
#include "modifyOverheadPhaseOne_.h"
#include "MPCStepFunction_internal_types.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void modifyOverheadPhaseOne_(g_struct_T *obj)
{
  int i;
  int idx;
  int idxEq;
  i = obj->sizes[0];
  for (idx = 0; idx < i; idx++) {
    obj->ATwset[181 * idx + 180] = 0.0;
  }
  for (idx = 0; idx < 120; idx++) {
    idxEq = 181 * idx + 180;
    obj->Aeq[idxEq] = 0.0;
    obj->ATwset[idxEq + 181 * (obj->isActiveIdx[1] - 1)] = 0.0;
  }
  obj->indexLB[obj->sizes[3] - 1] = 181;
  obj->lb[180] = obj->SLACK0;
  idxEq = obj->isActiveIdx[2];
  i = obj->nActiveConstr;
  for (idx = idxEq; idx <= i; idx++) {
    obj->ATwset[181 * (idx - 1) + 180] = -1.0;
  }
  idxEq = obj->isActiveIdx[4];
  if (obj->nWConstr[4] > 0) {
    i = obj->sizesNormal[4];
    for (idx = 0; idx < i; idx++) {
      int i1;
      i1 = idxEq + idx;
      obj->isActiveConstr[i1] = obj->isActiveConstr[i1 - 1];
    }
  }
  obj->isActiveConstr[obj->isActiveIdx[4] - 2] = false;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (modifyOverheadPhaseOne_.cpp)
