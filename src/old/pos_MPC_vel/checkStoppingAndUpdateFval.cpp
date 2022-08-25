//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// checkStoppingAndUpdateFval.cpp
//
// Code generation for function 'checkStoppingAndUpdateFval'
//

// Include files
#include "checkStoppingAndUpdateFval.h"
#include "PositionMPCStepFunction_internal_types.h"
#include "computeFval_ReuseHx.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "rt_nonfinite.h"
#include <algorithm>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace stopping {
void checkStoppingAndUpdateFval(int *activeSetChangeID, const double f[160],
                                struct_T *solution, f_struct_T *memspace,
                                const b_struct_T *objective,
                                d_struct_T *workingset, g_struct_T *qrmanager,
                                double options_ObjectiveLimit,
                                double runTimeOptions_ConstrRelTolFactor,
                                boolean_T updateFval)
{
  int nVar;
  solution->iterations++;
  nVar = objective->nvar - 1;
  if ((solution->iterations >= 35) &&
      ((solution->state != 1) || (objective->objtype == 5))) {
    solution->state = 0;
  }
  if (solution->iterations - solution->iterations / 50 * 50 == 0) {
    solution->maxConstr =
        WorkingSet::b_maxConstraintViolation(workingset, solution->xstar);
    if (solution->maxConstr > 1.0E-8 * runTimeOptions_ConstrRelTolFactor) {
      double constrViolation_new;
      boolean_T nonDegenerateWset;
      if (0 <= nVar) {
        std::copy(&solution->xstar[0], &solution->xstar[nVar + 1],
                  &solution->searchDir[0]);
      }
      nonDegenerateWset = initialize::feasibleX0ForWorkingSet(
          memspace->workspace_double, solution->searchDir, workingset,
          qrmanager);
      if ((!nonDegenerateWset) && (solution->state != 0)) {
        solution->state = -2;
      }
      *activeSetChangeID = 0;
      constrViolation_new =
          WorkingSet::b_maxConstraintViolation(workingset, solution->searchDir);
      if (constrViolation_new < solution->maxConstr) {
        if (0 <= nVar) {
          std::copy(&solution->searchDir[0], &solution->searchDir[nVar + 1],
                    &solution->xstar[0]);
        }
        solution->maxConstr = constrViolation_new;
      }
    }
  }
  if ((options_ObjectiveLimit > rtMinusInf) && updateFval) {
    solution->fstar = Objective::computeFval_ReuseHx(
        objective, memspace->workspace_double, f, solution->xstar);
    if ((solution->fstar < options_ObjectiveLimit) &&
        ((solution->state != 0) || (objective->objtype != 5))) {
      solution->state = 2;
    }
  }
}

} // namespace stopping
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (checkStoppingAndUpdateFval.cpp)
