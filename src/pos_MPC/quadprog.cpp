//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// quadprog.cpp
//
// Code generation for function 'quadprog'
//

// Include files
#include "quadprog.h"
#include "MPCStepFunction_internal_types.h"
#include "driver.h"
#include "loadProblem.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
namespace pos_MPC {
namespace coder {
void quadprog(const double H[32400], const double f[180],
              const double Aeq[21600], const double b_beq[120],
              const double lb[180], const double ub[180], const double x0[180],
              double x[180])
{
  static c_struct_T CholRegManager;
  static e_struct_T memspace;
  static f_struct_T obj;
  static g_struct_T WorkingSet;
  b_struct_T unusedExpr;
  d_struct_T expl_temp;
  struct_T solution;
  double H_infnrm;
  double colSum;
  double f_infnrm;
  double tol;
  int b_i;
  int i;
  int iATw0;
  int idx;
  int idx_global;
  int idx_local;
  int nWFixed;
  solution.fstar = 0.0;
  solution.firstorderopt = 0.0;
  std::memset(&solution.lambda[0], 0, 481U * sizeof(double));
  solution.state = 0;
  solution.maxConstr = 0.0;
  solution.iterations = 0;
  std::memset(&solution.searchDir[0], 0, 181U * sizeof(double));
  std::copy(&x0[0], &x0[180], &solution.xstar[0]);
  CholRegManager.ldm = 300;
  CholRegManager.ndims = 0;
  CholRegManager.info = 0;
  CholRegManager.ConvexCheck = true;
  CholRegManager.regTol_ = 0.0;
  CholRegManager.scaleFactor = 100.0;
  WorkingSet.mConstr = 0;
  WorkingSet.mConstrOrig = 0;
  WorkingSet.mConstrMax = 481;
  WorkingSet.nVar = 180;
  WorkingSet.nVarOrig = 180;
  WorkingSet.nVarMax = 181;
  WorkingSet.ldA = 181;
  std::memset(&WorkingSet.Aeq[0], 0, 21720U * sizeof(double));
  std::memset(&WorkingSet.beq[0], 0, 120U * sizeof(double));
  std::memset(&WorkingSet.lb[0], 0, 181U * sizeof(double));
  std::memset(&WorkingSet.ub[0], 0, 181U * sizeof(double));
  WorkingSet.mEqRemoved = 0;
  std::memset(&WorkingSet.indexEqRemoved[0], 0, 120U * sizeof(int));
  std::memset(&WorkingSet.ATwset[0], 0, 87061U * sizeof(double));
  WorkingSet.nActiveConstr = 0;
  std::memset(&WorkingSet.bwset[0], 0, 481U * sizeof(double));
  std::memset(&WorkingSet.maxConstrWorkspace[0], 0, 481U * sizeof(double));
  for (i = 0; i < 5; i++) {
    WorkingSet.sizes[i] = 0;
    WorkingSet.sizesNormal[i] = 0;
    WorkingSet.sizesPhaseOne[i] = 0;
    WorkingSet.sizesRegularized[i] = 0;
    WorkingSet.sizesRegPhaseOne[i] = 0;
  }
  for (i = 0; i < 6; i++) {
    WorkingSet.isActiveIdx[i] = 0;
    WorkingSet.isActiveIdxNormal[i] = 0;
    WorkingSet.isActiveIdxPhaseOne[i] = 0;
    WorkingSet.isActiveIdxRegularized[i] = 0;
    WorkingSet.isActiveIdxRegPhaseOne[i] = 0;
  }
  std::memset(&WorkingSet.isActiveConstr[0], 0, 481U * sizeof(boolean_T));
  std::memset(&WorkingSet.Wid[0], 0, 481U * sizeof(int));
  std::memset(&WorkingSet.Wlocalidx[0], 0, 481U * sizeof(int));
  for (i = 0; i < 5; i++) {
    WorkingSet.nWConstr[i] = 0;
  }
  WorkingSet.probType = 3;
  WorkingSet.SLACK0 = 1.0E-5;
  std::memset(&WorkingSet.indexLB[0], 0, 181U * sizeof(int));
  std::memset(&WorkingSet.indexUB[0], 0, 181U * sizeof(int));
  std::memset(&WorkingSet.indexFixed[0], 0, 181U * sizeof(int));
  i = 0;
  idx_global = 0;
  iATw0 = 0;
  for (idx = 0; idx < 180; idx++) {
    boolean_T guard1{false};
    tol = lb[idx];
    guard1 = false;
    if ((!std::isinf(tol)) && (!std::isnan(tol))) {
      if (std::abs(tol - ub[idx]) < 1.0E-8) {
        iATw0++;
        WorkingSet.indexFixed[iATw0 - 1] = idx + 1;
      } else {
        i++;
        WorkingSet.indexLB[i - 1] = idx + 1;
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      tol = ub[idx];
      if ((!std::isinf(tol)) && (!std::isnan(tol))) {
        idx_global++;
        WorkingSet.indexUB[idx_global - 1] = idx + 1;
      }
    }
  }
  optim::coder::qpactiveset::WorkingSet::loadProblem(&WorkingSet, Aeq, b_beq, i,
                                                     lb, idx_global, ub, iATw0);
  optim::coder::qpactiveset::WorkingSet::setProblemType(&WorkingSet, 3);
  i = WorkingSet.isActiveIdx[2];
  for (idx = i; idx < 482; idx++) {
    WorkingSet.isActiveConstr[idx - 1] = false;
  }
  WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
  WorkingSet.nWConstr[1] = 120;
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = WorkingSet.nWConstr[0] + 120;
  nWFixed = WorkingSet.sizes[0];
  for (idx_local = 0; idx_local < nWFixed; idx_local++) {
    WorkingSet.Wid[idx_local] = 1;
    WorkingSet.Wlocalidx[idx_local] = idx_local + 1;
    WorkingSet.isActiveConstr[idx_local] = true;
    i = 181 * idx_local;
    b_i = WorkingSet.indexFixed[idx_local];
    if (0 <= b_i - 2) {
      std::memset(&WorkingSet.ATwset[i], 0,
                  (((b_i + i) - i) + -1) * sizeof(double));
    }
    WorkingSet.ATwset[(WorkingSet.indexFixed[idx_local] + i) - 1] = 1.0;
    b_i = WorkingSet.indexFixed[idx_local] + 1;
    idx_global = WorkingSet.nVar;
    if (b_i <= idx_global) {
      std::memset(&WorkingSet.ATwset[(b_i + i) + -1], 0,
                  ((((idx_global + i) - b_i) - i) + 1) * sizeof(double));
    }
    WorkingSet.bwset[idx_local] =
        WorkingSet.ub[WorkingSet.indexFixed[idx_local] - 1];
  }
  WorkingSet.SLACK0 = 0.0;
  tol = 1.0;
  b_i = WorkingSet.nVar - 1;
  for (idx_local = 0; idx_local < 120; idx_local++) {
    idx_global = nWFixed + idx_local;
    WorkingSet.Wid[idx_global] = 2;
    WorkingSet.Wlocalidx[idx_global] = idx_local + 1;
    WorkingSet.isActiveConstr[idx_global] = true;
    idx = 181 * idx_local;
    iATw0 = 181 * idx_global;
    for (i = 0; i <= b_i; i++) {
      WorkingSet.ATwset[iATw0 + i] = WorkingSet.Aeq[idx + i];
    }
    WorkingSet.bwset[idx_global] = WorkingSet.beq[idx_local];
    colSum = 0.0;
    for (idx_global = 0; idx_global < 180; idx_global++) {
      colSum += std::abs(WorkingSet.Aeq[idx_global + idx]);
    }
    tol = std::fmax(tol, colSum);
  }
  H_infnrm = 0.0;
  f_infnrm = 0.0;
  for (i = 0; i < 180; i++) {
    colSum = 0.0;
    for (idx_global = 0; idx_global < 180; idx_global++) {
      colSum += std::abs(H[idx_global + 180 * i]);
    }
    H_infnrm = std::fmax(H_infnrm, colSum);
    f_infnrm = std::fmax(f_infnrm, std::abs(f[i]));
  }
  obj.ldq = 300;
  std::memset(&obj.Q[0], 0, 90000U * sizeof(double));
  std::memset(&obj.jpvt[0], 0, 481U * sizeof(int));
  obj.mrows = 0;
  obj.ncols = 0;
  obj.minRowCol = 0;
  obj.usedPivoting = false;
  expl_temp.RemainFeasible = false;
  expl_temp.ProbRelTolFactor = std::fmax(std::fmax(tol, f_infnrm), H_infnrm);
  expl_temp.ConstrRelTolFactor = tol;
  expl_temp.MaxIterations = 100;
  optim::coder::qpactiveset::driver(H, f, &solution, &memspace, &WorkingSet,
                                    &obj, &CholRegManager, expl_temp,
                                    &unusedExpr);
  std::copy(&solution.xstar[0], &solution.xstar[180], &x[0]);
}

} // namespace coder
} // namespace pos_MPC

// End of code generation (quadprog.cpp)
