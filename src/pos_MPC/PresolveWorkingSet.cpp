//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// PresolveWorkingSet.cpp
//
// Code generation for function 'PresolveWorkingSet'
//

// Include files
#include "PresolveWorkingSet.h"
#include "MPCStepFunction_internal_types.h"
#include "RemoveDependentEq_.h"
#include "RemoveDependentIneq_.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
void PresolveWorkingSet(struct_T *solution, e_struct_T *memspace,
                        g_struct_T *workingset, f_struct_T *qrmanager)
{
  int idxStartIneq;
  solution->state = 82;
  idxStartIneq = RemoveDependentEq_(memspace, workingset, qrmanager);
  if ((idxStartIneq != -1) && (workingset->nActiveConstr <= 300)) {
    boolean_T guard1{false};
    boolean_T okWorkingSet;
    RemoveDependentIneq_(workingset, qrmanager, memspace, 100.0);
    okWorkingSet = feasibleX0ForWorkingSet(
        memspace->workspace_double, solution->xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      RemoveDependentIneq_(workingset, qrmanager, memspace, 1000.0);
      okWorkingSet = feasibleX0ForWorkingSet(
          memspace->workspace_double, solution->xstar, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1 && (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                   workingset->nVar)) {
      double constrViolation;
      constrViolation =
          WorkingSet::maxConstraintViolation(workingset, solution->xstar);
      if (constrViolation > 1.0E-8) {
        solution->state = -2;
      }
    }
  } else {
    int idxEndIneq;
    solution->state = -3;
    idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
    idxEndIneq = workingset->nActiveConstr;
    for (int idx_global{idxStartIneq}; idx_global <= idxEndIneq; idx_global++) {
      workingset->isActiveConstr
          [(workingset->isActiveIdx[workingset->Wid[idx_global - 1] - 1] +
            workingset->Wlocalidx[idx_global - 1]) -
           2] = false;
    }
    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr =
        workingset->nWConstr[0] + workingset->nWConstr[1];
  }
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (PresolveWorkingSet.cpp)
