//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// addBoundToActiveSetMatrix_.cpp
//
// Code generation for function 'addBoundToActiveSetMatrix_'
//

// Include files
#include "addBoundToActiveSetMatrix_.h"
#include "MPCStepFunction_internal_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void addBoundToActiveSetMatrix_(g_struct_T *obj, int TYPE, int idx_local)
{
  int colOffset;
  int i;
  int idx_bnd_local;
  obj->nWConstr[TYPE - 1]++;
  obj->isActiveConstr[(obj->isActiveIdx[TYPE - 1] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  obj->Wid[obj->nActiveConstr - 1] = TYPE;
  obj->Wlocalidx[obj->nActiveConstr - 1] = idx_local;
  colOffset = 181 * (obj->nActiveConstr - 1) - 1;
  switch (TYPE) {
  case 5:
    idx_bnd_local = obj->indexUB[idx_local - 1];
    obj->bwset[obj->nActiveConstr - 1] = obj->ub[idx_bnd_local - 1];
    break;
  default:
    idx_bnd_local = obj->indexLB[idx_local - 1];
    obj->bwset[obj->nActiveConstr - 1] = obj->lb[idx_bnd_local - 1];
    break;
  }
  if (0 <= idx_bnd_local - 2) {
    std::memset(&obj->ATwset[colOffset + 1], 0,
                (((idx_bnd_local + colOffset) - colOffset) + -1) *
                    sizeof(double));
  }
  obj->ATwset[idx_bnd_local + colOffset] =
      2.0 * static_cast<double>(TYPE == 5) - 1.0;
  idx_bnd_local++;
  i = obj->nVar;
  if (idx_bnd_local <= i) {
    std::memset(&obj->ATwset[idx_bnd_local + colOffset], 0,
                ((((i + colOffset) - idx_bnd_local) - colOffset) + 1) *
                    sizeof(double));
  }
  switch (obj->probType) {
  case 3:
  case 2:
    break;
  default:
    obj->ATwset[obj->nVar + colOffset] = -1.0;
    break;
  }
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (addBoundToActiveSetMatrix_.cpp)
