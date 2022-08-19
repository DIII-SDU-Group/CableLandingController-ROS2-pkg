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
#include "PositionMPCStepFunction_internal_types.h"
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
void quadprog(const double H[25600], const double f[160],
              const double Aineq[25600], const double bineq[160],
              const double Aeq[12800], const double b_beq[80],
              const double lb[160], const double ub[160], const double x0[160],
              double x[160])
{
  static c_struct_T CholRegManager;
  static d_struct_T WorkingSet;
  static f_struct_T memspace;
  static g_struct_T obj;
  b_struct_T unusedExpr;
  e_struct_T expl_temp;
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
  std::memset(&solution.lambda[0], 0, 561U * sizeof(double));
  solution.state = 0;
  solution.maxConstr = 0.0;
  solution.iterations = 0;
  std::memset(&solution.searchDir[0], 0, 161U * sizeof(double));
  std::copy(&x0[0], &x0[160], &solution.xstar[0]);
  CholRegManager.ldm = 240;
  CholRegManager.ndims = 0;
  CholRegManager.info = 0;
  CholRegManager.ConvexCheck = true;
  CholRegManager.regTol_ = 0.0;
  CholRegManager.scaleFactor = 100.0;
  WorkingSet.mConstr = 0;
  WorkingSet.mConstrOrig = 0;
  WorkingSet.mConstrMax = 561;
  WorkingSet.nVar = 160;
  WorkingSet.nVarOrig = 160;
  WorkingSet.nVarMax = 161;
  WorkingSet.ldA = 161;
  std::memset(&WorkingSet.Aineq[0], 0, 25760U * sizeof(double));
  std::memset(&WorkingSet.bineq[0], 0, 160U * sizeof(double));
  std::memset(&WorkingSet.Aeq[0], 0, 12880U * sizeof(double));
  std::memset(&WorkingSet.beq[0], 0, 80U * sizeof(double));
  std::memset(&WorkingSet.lb[0], 0, 161U * sizeof(double));
  std::memset(&WorkingSet.ub[0], 0, 161U * sizeof(double));
  WorkingSet.mEqRemoved = 0;
  std::memset(&WorkingSet.indexEqRemoved[0], 0, 80U * sizeof(int));
  std::memset(&WorkingSet.ATwset[0], 0, 90321U * sizeof(double));
  WorkingSet.nActiveConstr = 0;
  std::memset(&WorkingSet.bwset[0], 0, 561U * sizeof(double));
  std::memset(&WorkingSet.maxConstrWorkspace[0], 0, 561U * sizeof(double));
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
  std::memset(&WorkingSet.isActiveConstr[0], 0, 561U * sizeof(boolean_T));
  std::memset(&WorkingSet.Wid[0], 0, 561U * sizeof(int));
  std::memset(&WorkingSet.Wlocalidx[0], 0, 561U * sizeof(int));
  for (i = 0; i < 5; i++) {
    WorkingSet.nWConstr[i] = 0;
  }
  WorkingSet.probType = 3;
  WorkingSet.SLACK0 = 1.0E-5;
  std::memset(&WorkingSet.indexLB[0], 0, 161U * sizeof(int));
  std::memset(&WorkingSet.indexUB[0], 0, 161U * sizeof(int));
  std::memset(&WorkingSet.indexFixed[0], 0, 161U * sizeof(int));
  i = 0;
  idx_global = 0;
  iATw0 = 0;
  for (idx = 0; idx < 160; idx++) {
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
  optim::coder::qpactiveset::WorkingSet::loadProblem(
      &WorkingSet, Aineq, bineq, Aeq, b_beq, i, lb, idx_global, ub, iATw0);
  optim::coder::qpactiveset::WorkingSet::setProblemType(&WorkingSet, 3);
  i = WorkingSet.isActiveIdx[2];
  for (idx = i; idx < 562; idx++) {
    WorkingSet.isActiveConstr[idx - 1] = false;
  }
  WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
  WorkingSet.nWConstr[1] = 80;
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = WorkingSet.nWConstr[0] + 80;
  nWFixed = WorkingSet.sizes[0];
  for (idx_local = 0; idx_local < nWFixed; idx_local++) {
    WorkingSet.Wid[idx_local] = 1;
    WorkingSet.Wlocalidx[idx_local] = idx_local + 1;
    WorkingSet.isActiveConstr[idx_local] = true;
    i = 161 * idx_local;
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
  for (idx_local = 0; idx_local < 80; idx_local++) {
    int iAeq0_tmp;
    idx_global = nWFixed + idx_local;
    WorkingSet.Wid[idx_global] = 2;
    WorkingSet.Wlocalidx[idx_global] = idx_local + 1;
    WorkingSet.isActiveConstr[idx_global] = true;
    iAeq0_tmp = 161 * idx_local;
    iATw0 = 161 * idx_global;
    for (i = 0; i <= b_i; i++) {
      WorkingSet.ATwset[iATw0 + i] = WorkingSet.Aeq[iAeq0_tmp + i];
    }
    WorkingSet.bwset[idx_global] = WorkingSet.beq[idx_local];
    colSum = 0.0;
    for (idx = 0; idx < 160; idx++) {
      colSum += std::abs(WorkingSet.Aeq[idx + iAeq0_tmp]);
    }
    tol = std::fmax(tol, colSum);
  }
  H_infnrm = 0.0;
  f_infnrm = 0.0;
  for (i = 0; i < 160; i++) {
    double b_colSum;
    colSum = 0.0;
    idx_global = 161 * i;
    b_colSum = 0.0;
    for (idx = 0; idx < 160; idx++) {
      colSum += std::abs(WorkingSet.Aineq[idx + idx_global]);
      b_colSum += std::abs(H[idx + 160 * i]);
    }
    tol = std::fmax(tol, colSum);
    H_infnrm = std::fmax(H_infnrm, b_colSum);
    f_infnrm = std::fmax(f_infnrm, std::abs(f[i]));
  }
  obj.ldq = 240;
  std::memset(&obj.Q[0], 0, 57600U * sizeof(double));
  std::memset(&obj.jpvt[0], 0, 561U * sizeof(int));
  obj.mrows = 0;
  obj.ncols = 0;
  obj.minRowCol = 0;
  obj.usedPivoting = false;
  expl_temp.RemainFeasible = false;
  expl_temp.ProbRelTolFactor = std::fmax(std::fmax(tol, f_infnrm), H_infnrm);
  expl_temp.ConstrRelTolFactor = tol;
  expl_temp.MaxIterations = 10;
  optim::coder::qpactiveset::driver(H, f, &solution, &memspace, &WorkingSet,
                                    &obj, &CholRegManager, expl_temp,
                                    &unusedExpr);
  std::copy(&solution.xstar[0], &solution.xstar[160], &x[0]);
}

} // namespace coder
} // namespace pos_MPC

// End of code generation (quadprog.cpp)
