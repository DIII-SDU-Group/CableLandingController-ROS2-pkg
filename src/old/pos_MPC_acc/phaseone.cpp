//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// phaseone.cpp
//
// Code generation for function 'phaseone'
//

// Include files
#include "phaseone.h"
#include "MPCStepFunction_data.h"
#include "MPCStepFunction_internal_types.h"
#include "computeFval.h"
#include "iterate.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include <cstring>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void phaseone(const double H[14400], const double f[120], struct_T *solution,
              e_struct_T *memspace, g_struct_T *workingset,
              f_struct_T *qrmanager, c_struct_T *cholmanager,
              const d_struct_T *runTimeOptions, b_struct_T *objective,
              h_struct_T *options)
{
  static const char t1_SolverName[8]{'q', 'u', 'a', 'd', 'p', 'r', 'o', 'g'};
  static const char t1_FiniteDifferenceType[7]{'f', 'o', 'r', 'w',
                                               'a', 'r', 'd'};
  static const char t1_Display[5]{'f', 'i', 'n', 'a', 'l'};
  int idx;
  int idxEndIneq;
  int idxStartIneq;
  int idx_global;
  int nVar_tmp_tmp;
  options->InitDamping = 0.01;
  for (idx_global = 0; idx_global < 7; idx_global++) {
    options->FiniteDifferenceType[idx_global] =
        t1_FiniteDifferenceType[idx_global];
  }
  options->SpecifyObjectiveGradient = false;
  options->ScaleProblem = false;
  options->SpecifyConstraintGradient = false;
  options->NonFiniteSupport = true;
  options->IterDisplaySQP = false;
  options->FiniteDifferenceStepSize = -1.0;
  options->MaxFunctionEvaluations = -1.0;
  options->IterDisplayQP = false;
  options->PricingTolerance = 0.0;
  for (idx_global = 0; idx_global < 10; idx_global++) {
    options->Algorithm[idx_global] = cv[idx_global];
  }
  options->ConstraintTolerance = 1.0E-8;
  options->OptimalityTolerance = 0.999;
  options->MaxIterations = 35.0;
  options->FunctionTolerance = 1.0E+9;
  for (idx_global = 0; idx_global < 8; idx_global++) {
    options->SolverName[idx_global] = t1_SolverName[idx_global];
  }
  options->CheckGradients = false;
  options->Diagnostics[0] = 'o';
  options->Diagnostics[1] = 'f';
  options->Diagnostics[2] = 'f';
  options->DiffMaxChange = rtInf;
  options->DiffMinChange = 0.0;
  for (idx_global = 0; idx_global < 5; idx_global++) {
    options->Display[idx_global] = t1_Display[idx_global];
  }
  options->FunValCheck[0] = 'o';
  options->FunValCheck[1] = 'f';
  options->FunValCheck[2] = 'f';
  options->UseParallel = false;
  options->LinearSolver[0] = 'a';
  options->LinearSolver[1] = 'u';
  options->LinearSolver[2] = 't';
  options->LinearSolver[3] = 'o';
  options->SubproblemAlgorithm[0] = 'c';
  options->SubproblemAlgorithm[1] = 'g';
  nVar_tmp_tmp = workingset->nVar;
  solution->xstar[120] = solution->maxConstr + 1.0;
  idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
  idxEndIneq = workingset->nActiveConstr;
  for (idx_global = idxStartIneq; idx_global <= idxEndIneq; idx_global++) {
    workingset->isActiveConstr
        [(workingset->isActiveIdx[workingset->Wid[idx_global - 1] - 1] +
          workingset->Wlocalidx[idx_global - 1]) -
         2] = false;
  }
  workingset->nWConstr[2] = 0;
  workingset->nWConstr[3] = 0;
  workingset->nWConstr[4] = 0;
  workingset->nActiveConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  WorkingSet::setProblemType(workingset, 1);
  std::memset(&objective->grad[0], 0, 121U * sizeof(double));
  std::memset(&objective->Hx[0], 0, 120U * sizeof(double));
  objective->maxVar = 121;
  objective->beta = 0.0;
  objective->rho = 0.0;
  objective->prev_objtype = 3;
  objective->prev_nvar = 120;
  objective->prev_hasLinear = true;
  objective->objtype = 5;
  objective->nvar = 121;
  objective->gammaScalar = 1.0;
  objective->hasLinear = true;
  options->ObjectiveLimit = 1.0E-8 * runTimeOptions->ConstrRelTolFactor;
  solution->fstar = Objective::computeFval(
      objective, memspace->workspace_double, H, f, solution->xstar);
  solution->state = 5;
  iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
          objective, options->ObjectiveLimit, 1.4901161193847657E-10,
          runTimeOptions->ConstrRelTolFactor, runTimeOptions->ProbRelTolFactor,
          runTimeOptions->RemainFeasible);
  if (workingset
          ->isActiveConstr[(workingset->isActiveIdx[3] + workingset->sizes[3]) -
                           2]) {
    boolean_T exitg1;
    idx = workingset->sizes[0] + 80;
    exitg1 = false;
    while ((!exitg1) && (idx + 1 <= workingset->nActiveConstr)) {
      if ((workingset->Wid[idx] == 4) &&
          (workingset->Wlocalidx[idx] == workingset->sizes[3])) {
        idxEndIneq = workingset->Wid[idx] - 1;
        workingset->isActiveConstr
            [(workingset->isActiveIdx[workingset->Wid[idx] - 1] +
              workingset->Wlocalidx[idx]) -
             2] = false;
        workingset->Wid[idx] = workingset->Wid[workingset->nActiveConstr - 1];
        workingset->Wlocalidx[idx] =
            workingset->Wlocalidx[workingset->nActiveConstr - 1];
        idx_global = workingset->nVar;
        for (idxStartIneq = 0; idxStartIneq < idx_global; idxStartIneq++) {
          workingset->ATwset[idxStartIneq + 121 * idx] =
              workingset->ATwset[idxStartIneq +
                                 121 * (workingset->nActiveConstr - 1)];
        }
        workingset->bwset[idx] =
            workingset->bwset[workingset->nActiveConstr - 1];
        workingset->nActiveConstr--;
        workingset->nWConstr[idxEndIneq]--;
        exitg1 = true;
      } else {
        idx++;
      }
    }
  }
  idxStartIneq = workingset->nActiveConstr - 1;
  while ((idxStartIneq + 1 > workingset->sizes[0] + 80) &&
         (idxStartIneq + 1 > nVar_tmp_tmp)) {
    idxEndIneq = workingset->Wid[idxStartIneq] - 1;
    workingset->isActiveConstr
        [(workingset->isActiveIdx[workingset->Wid[idxStartIneq] - 1] +
          workingset->Wlocalidx[idxStartIneq]) -
         2] = false;
    workingset->Wid[idxStartIneq] =
        workingset->Wid[workingset->nActiveConstr - 1];
    workingset->Wlocalidx[idxStartIneq] =
        workingset->Wlocalidx[workingset->nActiveConstr - 1];
    idx_global = workingset->nVar;
    for (idx = 0; idx < idx_global; idx++) {
      workingset->ATwset[idx + 121 * idxStartIneq] =
          workingset->ATwset[idx + 121 * (workingset->nActiveConstr - 1)];
    }
    workingset->bwset[idxStartIneq] =
        workingset->bwset[workingset->nActiveConstr - 1];
    workingset->nActiveConstr--;
    workingset->nWConstr[idxEndIneq]--;
    idxStartIneq--;
  }
  solution->maxConstr = solution->xstar[120];
  WorkingSet::setProblemType(workingset, 3);
  objective->objtype = objective->prev_objtype;
  objective->nvar = objective->prev_nvar;
  objective->hasLinear = objective->prev_hasLinear;
  options->ObjectiveLimit = -1.0E+20;
  options->StepTolerance = 0.001;
}

void phaseone(const double H[14400], const double f[120], struct_T *solution,
              e_struct_T *memspace, g_struct_T *workingset,
              f_struct_T *qrmanager, c_struct_T *cholmanager,
              b_struct_T *objective, h_struct_T *options,
              const d_struct_T *runTimeOptions)
{
  int PHASEONE;
  int PROBTYPE_ORIG;
  int idxEndIneq;
  int idxStartIneq;
  int idx_global;
  int nVar_tmp_tmp;
  PROBTYPE_ORIG = workingset->probType;
  nVar_tmp_tmp = workingset->nVar;
  solution->xstar[120] = solution->maxConstr + 1.0;
  if (workingset->probType == 3) {
    PHASEONE = 1;
  } else {
    PHASEONE = 4;
  }
  idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
  idxEndIneq = workingset->nActiveConstr;
  for (idx_global = idxStartIneq; idx_global <= idxEndIneq; idx_global++) {
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
  objective->nvar = 121;
  objective->gammaScalar = 1.0;
  objective->hasLinear = true;
  options->ObjectiveLimit = 1.0E-8 * runTimeOptions->ConstrRelTolFactor;
  options->StepTolerance = 1.4901161193847657E-10;
  solution->fstar = Objective::computeFval(
      objective, memspace->workspace_double, H, f, solution->xstar);
  solution->state = 5;
  iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
          objective, options->ObjectiveLimit, options->StepTolerance,
          runTimeOptions->ConstrRelTolFactor, runTimeOptions->ProbRelTolFactor,
          runTimeOptions->RemainFeasible);
  if (workingset
          ->isActiveConstr[(workingset->isActiveIdx[3] + workingset->sizes[3]) -
                           2]) {
    boolean_T exitg1;
    idx_global = workingset->sizes[0] + 80;
    exitg1 = false;
    while ((!exitg1) && (idx_global + 1 <= workingset->nActiveConstr)) {
      if ((workingset->Wid[idx_global] == 4) &&
          (workingset->Wlocalidx[idx_global] == workingset->sizes[3])) {
        idxStartIneq = workingset->Wid[idx_global] - 1;
        workingset->isActiveConstr
            [(workingset->isActiveIdx[workingset->Wid[idx_global] - 1] +
              workingset->Wlocalidx[idx_global]) -
             2] = false;
        workingset->Wid[idx_global] =
            workingset->Wid[workingset->nActiveConstr - 1];
        workingset->Wlocalidx[idx_global] =
            workingset->Wlocalidx[workingset->nActiveConstr - 1];
        idxEndIneq = workingset->nVar;
        for (PHASEONE = 0; PHASEONE < idxEndIneq; PHASEONE++) {
          workingset->ATwset[PHASEONE + 121 * idx_global] =
              workingset
                  ->ATwset[PHASEONE + 121 * (workingset->nActiveConstr - 1)];
        }
        workingset->bwset[idx_global] =
            workingset->bwset[workingset->nActiveConstr - 1];
        workingset->nActiveConstr--;
        workingset->nWConstr[idxStartIneq]--;
        exitg1 = true;
      } else {
        idx_global++;
      }
    }
  }
  PHASEONE = workingset->nActiveConstr - 1;
  while ((PHASEONE + 1 > workingset->sizes[0] + 80) &&
         (PHASEONE + 1 > nVar_tmp_tmp)) {
    idxStartIneq = workingset->Wid[PHASEONE] - 1;
    workingset->isActiveConstr
        [(workingset->isActiveIdx[workingset->Wid[PHASEONE] - 1] +
          workingset->Wlocalidx[PHASEONE]) -
         2] = false;
    workingset->Wid[PHASEONE] = workingset->Wid[workingset->nActiveConstr - 1];
    workingset->Wlocalidx[PHASEONE] =
        workingset->Wlocalidx[workingset->nActiveConstr - 1];
    idxEndIneq = workingset->nVar;
    for (idx_global = 0; idx_global < idxEndIneq; idx_global++) {
      workingset->ATwset[idx_global + 121 * PHASEONE] =
          workingset
              ->ATwset[idx_global + 121 * (workingset->nActiveConstr - 1)];
    }
    workingset->bwset[PHASEONE] =
        workingset->bwset[workingset->nActiveConstr - 1];
    workingset->nActiveConstr--;
    workingset->nWConstr[idxStartIneq]--;
    PHASEONE--;
  }
  solution->maxConstr = solution->xstar[120];
  WorkingSet::setProblemType(workingset, PROBTYPE_ORIG);
  objective->objtype = objective->prev_objtype;
  objective->nvar = objective->prev_nvar;
  objective->hasLinear = objective->prev_hasLinear;
  options->ObjectiveLimit = -1.0E+20;
  options->StepTolerance = 0.001;
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (phaseone.cpp)
