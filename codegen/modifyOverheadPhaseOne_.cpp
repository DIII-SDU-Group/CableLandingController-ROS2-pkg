//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: modifyOverheadPhaseOne_.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "modifyOverheadPhaseOne_.h"
#include "mpc_fun_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : f_struct_T *obj
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void modifyOverheadPhaseOne_(f_struct_T *obj)
{
  int i;
  int idx;
  int idxEq;
  i = obj->sizes[0];
  for (idx = 0; idx < i; idx++) {
    obj->ATwset[(obj->nVar + obj->ldA * idx) - 1] = 0.0;
  }
  i = obj->sizes[1];
  for (idx = 0; idx < i; idx++) {
    idxEq = (obj->nVar + obj->ldA * idx) - 1;
    obj->Aeq[idxEq] = 0.0;
    obj->ATwset[idxEq + obj->ldA * (obj->isActiveIdx[1] - 1)] = 0.0;
  }
  i = obj->sizes[2];
  for (idx = 0; idx < i; idx++) {
    obj->Aineq[(obj->nVar + obj->ldA * idx) - 1] = -1.0;
  }
  obj->indexLB[obj->sizes[3] - 1] = obj->nVar;
  obj->lb[obj->nVar - 1] = obj->SLACK0;
  idxEq = obj->isActiveIdx[2];
  i = obj->nActiveConstr;
  for (idx = idxEq; idx <= i; idx++) {
    obj->ATwset[(obj->nVar + obj->ldA * (idx - 1)) - 1] = -1.0;
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

//
// File trailer for modifyOverheadPhaseOne_.cpp
//
// [EOF]
//
