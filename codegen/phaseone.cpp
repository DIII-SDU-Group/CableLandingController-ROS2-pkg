//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: phaseone.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "phaseone.h"
#include "computeFval.h"
#include "iterate.h"
#include "mpc_fun_internal_types.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &H
//                const ::coder::array<double, 1U> &f
//                b_struct_T *solution
//                g_struct_T *memspace
//                f_struct_T *workingset
//                d_struct_T *qrmanager
//                e_struct_T *cholmanager
//                c_struct_T *objective
//                h_struct_T *b_options
//                const struct_T *runTimeOptions
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void phaseone(const ::coder::array<double, 2U> &H,
              const ::coder::array<double, 1U> &f, b_struct_T *solution,
              g_struct_T *memspace, f_struct_T *workingset,
              d_struct_T *qrmanager, e_struct_T *cholmanager,
              c_struct_T *objective, h_struct_T *b_options,
              const struct_T *runTimeOptions)
{
  int PHASEONE;
  int PROBTYPE_ORIG;
  int idxEndIneq_tmp_tmp;
  int idxStartIneq;
  int nVar_tmp;
  PROBTYPE_ORIG = workingset->probType;
  nVar_tmp = workingset->nVar;
  solution->xstar[workingset->nVar] = solution->maxConstr + 1.0;
  if (workingset->probType == 3) {
    PHASEONE = 1;
  } else {
    PHASEONE = 4;
  }
  idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
  idxEndIneq_tmp_tmp = workingset->nActiveConstr;
  for (int idx_global{idxStartIneq}; idx_global <= idxEndIneq_tmp_tmp;
       idx_global++) {
    workingset->isActiveConstr
        [(workingset->isActiveIdx[workingset->Wid[idx_global - 1] - 1] +
          workingset->Wlocalidx[idx_global - 1]) -
         2] = false;
  }
  workingset->nWConstr[2] = 0;
  workingset->nWConstr[3] = 0;
  workingset->nWConstr[4] = 0;
  workingset->nActiveConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  WorkingSet::setProblemType(workingset, PHASEONE);
  objective->prev_objtype = objective->objtype;
  objective->prev_nvar = objective->nvar;
  objective->prev_hasLinear = objective->hasLinear;
  objective->objtype = 5;
  objective->nvar = nVar_tmp + 1;
  objective->gammaScalar = 1.0;
  objective->hasLinear = true;
  b_options->ObjectiveLimit = 1.0E-8 * runTimeOptions->ConstrRelTolFactor;
  b_options->StepTolerance = 1.4901161193847657E-10;
  solution->fstar = Objective::computeFval(
      objective, memspace->workspace_double, H, solution->xstar);
  solution->state = 5;
  iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
          objective, b_options->ObjectiveLimit, b_options->StepTolerance,
          runTimeOptions->MaxIterations, runTimeOptions->ConstrRelTolFactor,
          runTimeOptions->ProbRelTolFactor, runTimeOptions->RemainFeasible);
  if (workingset
          ->isActiveConstr[(workingset->isActiveIdx[3] + workingset->sizes[3]) -
                           2]) {
    boolean_T exitg1;
    PHASEONE = workingset->sizes[0] + workingset->sizes[1];
    exitg1 = false;
    while ((!exitg1) && (PHASEONE + 1 <= workingset->nActiveConstr)) {
      if ((workingset->Wid[PHASEONE] == 4) &&
          (workingset->Wlocalidx[PHASEONE] == workingset->sizes[3])) {
        WorkingSet::removeConstr(workingset, PHASEONE + 1);
        exitg1 = true;
      } else {
        PHASEONE++;
      }
    }
  }
  PHASEONE = workingset->nActiveConstr;
  idxStartIneq = workingset->sizes[0] + workingset->sizes[1];
  while ((PHASEONE > idxStartIneq) && (PHASEONE > nVar_tmp)) {
    WorkingSet::removeConstr(workingset, PHASEONE);
    PHASEONE--;
  }
  solution->maxConstr = solution->xstar[nVar_tmp];
  WorkingSet::setProblemType(workingset, PROBTYPE_ORIG);
  objective->objtype = objective->prev_objtype;
  objective->nvar = objective->prev_nvar;
  objective->hasLinear = objective->prev_hasLinear;
  b_options->ObjectiveLimit = -1.0E+20;
  b_options->StepTolerance = 0.001;
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for phaseone.cpp
//
// [EOF]
//
