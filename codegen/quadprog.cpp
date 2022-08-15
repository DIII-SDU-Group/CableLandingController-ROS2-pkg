//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: quadprog.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "quadprog.h"
#include "compressBounds.h"
#include "driver.h"
#include "loadProblem.h"
#include "mpc_fun_internal_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &H
//                const ::coder::array<double, 1U> &f
//                const ::coder::array<double, 2U> &Aineq
//                const ::coder::array<double, 1U> &bineq
//                const ::coder::array<double, 2U> &Aeq
//                const ::coder::array<double, 1U> &beq
//                const ::coder::array<double, 1U> &lb
//                const ::coder::array<double, 1U> &ub
//                const ::coder::array<double, 1U> &x0
//                const h_struct_T *b_options
//                ::coder::array<double, 1U> &x
// Return Type  : void
//
namespace coder {
void quadprog(const ::coder::array<double, 2U> &H,
              const ::coder::array<double, 1U> &f,
              const ::coder::array<double, 2U> &Aineq,
              const ::coder::array<double, 1U> &bineq,
              const ::coder::array<double, 2U> &Aeq,
              const ::coder::array<double, 1U> &beq,
              const ::coder::array<double, 1U> &lb,
              const ::coder::array<double, 1U> &ub,
              const ::coder::array<double, 1U> &x0, const h_struct_T *b_options,
              ::coder::array<double, 1U> &x)
{
  b_struct_T solution;
  c_struct_T QPObjective;
  d_struct_T QRManager;
  e_struct_T CholRegManager;
  f_struct_T WorkingSet;
  g_struct_T memspace;
  h_struct_T c_options;
  struct_T expl_temp;
  double H_infnrm;
  double colSum;
  double tol;
  int QRRowBound;
  int b_i;
  int i;
  int iAeq0;
  int idxFillStart;
  int idx_local;
  int mConstrMax;
  int mFixed;
  int mLB;
  int mUB;
  int maxDims;
  int nVar;
  int nWFixed;
  nVar = x0.size(0) - 1;
  mConstrMax = (((bineq.size(0) + beq.size(0)) + lb.size(0)) + ub.size(0)) + 1;
  solution.xstar.set_size(x0.size(0) + 1);
  solution.fstar = 0.0;
  solution.firstorderopt = 0.0;
  solution.lambda.set_size(mConstrMax);
  for (i = 0; i < mConstrMax; i++) {
    solution.lambda[i] = 0.0;
  }
  solution.state = 0;
  solution.maxConstr = 0.0;
  solution.iterations = 0;
  solution.searchDir.set_size(x0.size(0) + 1);
  idxFillStart = x0.size(0);
  for (i = 0; i <= idxFillStart; i++) {
    solution.searchDir[i] = 0.0;
  }
  for (QRRowBound = 0; QRRowBound <= nVar; QRRowBound++) {
    solution.xstar[QRRowBound] = x0[QRRowBound];
  }
  QPObjective.grad.set_size(x0.size(0) + 1);
  QPObjective.Hx.set_size(x0.size(0));
  QPObjective.maxVar = x0.size(0) + 1;
  QPObjective.beta = 0.0;
  QPObjective.rho = 0.0;
  QPObjective.prev_objtype = 3;
  QPObjective.prev_nvar = 0;
  QPObjective.prev_hasLinear = false;
  QPObjective.gammaScalar = 0.0;
  QPObjective.hasLinear = (f.size(0) != 0);
  QPObjective.nvar = x0.size(0);
  QPObjective.objtype = 3;
  maxDims = x0.size(0) + 1;
  if (maxDims <= mConstrMax) {
    maxDims = mConstrMax;
  }
  idxFillStart = beq.size(0);
  if (1 > idxFillStart) {
    idxFillStart = 1;
  }
  QRRowBound = x0.size(0) + idxFillStart;
  QRManager.ldq = QRRowBound;
  QRManager.QR.set_size(QRRowBound, maxDims);
  QRManager.Q.set_size(QRRowBound, QRRowBound);
  iAeq0 = QRRowBound * QRRowBound;
  for (i = 0; i < iAeq0; i++) {
    QRManager.Q[i] = 0.0;
  }
  QRManager.jpvt.set_size(maxDims);
  for (i = 0; i < maxDims; i++) {
    QRManager.jpvt[i] = 0;
  }
  QRManager.mrows = 0;
  QRManager.ncols = 0;
  if (QRRowBound < maxDims) {
    idxFillStart = QRRowBound;
  } else {
    idxFillStart = maxDims;
  }
  QRManager.tau.set_size(idxFillStart);
  QRManager.minRowCol = 0;
  QRManager.usedPivoting = false;
  CholRegManager.FMat.set_size(iAeq0);
  CholRegManager.ldm = QRRowBound;
  CholRegManager.ndims = 0;
  CholRegManager.info = 0;
  CholRegManager.ConvexCheck = true;
  CholRegManager.regTol_ = 0.0;
  CholRegManager.workspace_.set_size(48 * QRRowBound);
  CholRegManager.workspace2_.set_size(48 * QRRowBound);
  CholRegManager.scaleFactor = 100.0;
  WorkingSet.mConstr = 0;
  WorkingSet.mConstrOrig = 0;
  WorkingSet.mConstrMax = mConstrMax;
  WorkingSet.nVar = x0.size(0);
  WorkingSet.nVarOrig = x0.size(0);
  WorkingSet.nVarMax = x0.size(0) + 1;
  WorkingSet.ldA = x0.size(0) + 1;
  WorkingSet.Aineq.set_size(bineq.size(0) * (x0.size(0) + 1));
  WorkingSet.bineq.set_size(bineq.size(0));
  WorkingSet.Aeq.set_size(beq.size(0) * (x0.size(0) + 1));
  WorkingSet.beq.set_size(beq.size(0));
  WorkingSet.lb.set_size(x0.size(0) + 1);
  WorkingSet.ub.set_size(x0.size(0) + 1);
  WorkingSet.indexLB.set_size(x0.size(0) + 1);
  WorkingSet.indexUB.set_size(x0.size(0) + 1);
  WorkingSet.indexFixed.set_size(x0.size(0) + 1);
  WorkingSet.mEqRemoved = 0;
  WorkingSet.indexEqRemoved.set_size(beq.size(0));
  WorkingSet.ATwset.set_size((x0.size(0) + 1) * mConstrMax);
  WorkingSet.bwset.set_size(mConstrMax);
  WorkingSet.nActiveConstr = 0;
  WorkingSet.maxConstrWorkspace.set_size(mConstrMax);
  for (b_i = 0; b_i < 5; b_i++) {
    WorkingSet.sizes[b_i] = 0;
    WorkingSet.sizesNormal[b_i] = 0;
    WorkingSet.sizesPhaseOne[b_i] = 0;
    WorkingSet.sizesRegularized[b_i] = 0;
    WorkingSet.sizesRegPhaseOne[b_i] = 0;
  }
  for (b_i = 0; b_i < 6; b_i++) {
    WorkingSet.isActiveIdx[b_i] = 0;
    WorkingSet.isActiveIdxNormal[b_i] = 0;
    WorkingSet.isActiveIdxPhaseOne[b_i] = 0;
    WorkingSet.isActiveIdxRegularized[b_i] = 0;
    WorkingSet.isActiveIdxRegPhaseOne[b_i] = 0;
  }
  WorkingSet.isActiveConstr.set_size(mConstrMax);
  WorkingSet.Wid.set_size(mConstrMax);
  WorkingSet.Wlocalidx.set_size(mConstrMax);
  for (b_i = 0; b_i < 5; b_i++) {
    WorkingSet.nWConstr[b_i] = 0;
  }
  WorkingSet.probType = 3;
  WorkingSet.SLACK0 = 1.0E-5;
  optim::coder::qpactiveset::initialize::compressBounds(
      x0.size(0), WorkingSet.indexLB, WorkingSet.indexUB, WorkingSet.indexFixed,
      lb, ub, &mLB, &mUB, &mFixed);
  optim::coder::qpactiveset::WorkingSet::loadProblem(
      &WorkingSet, bineq.size(0), bineq.size(0), Aineq, bineq, beq.size(0),
      beq.size(0), Aeq, beq, mLB, lb, mUB, ub, mFixed, mConstrMax);
  optim::coder::qpactiveset::WorkingSet::setProblemType(&WorkingSet, 3);
  idxFillStart = WorkingSet.isActiveIdx[2];
  i = WorkingSet.mConstrMax;
  for (QRRowBound = idxFillStart; QRRowBound <= i; QRRowBound++) {
    WorkingSet.isActiveConstr[QRRowBound - 1] = false;
  }
  WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
  WorkingSet.nWConstr[1] = WorkingSet.sizes[1];
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = WorkingSet.nWConstr[0] + WorkingSet.nWConstr[1];
  nWFixed = WorkingSet.sizes[0];
  for (idx_local = 0; idx_local < nWFixed; idx_local++) {
    WorkingSet.Wid[idx_local] = 1;
    WorkingSet.Wlocalidx[idx_local] = idx_local + 1;
    WorkingSet.isActiveConstr[idx_local] = true;
    QRRowBound = WorkingSet.ldA * idx_local;
    i = WorkingSet.indexFixed[idx_local];
    for (b_i = 0; b_i <= i - 2; b_i++) {
      WorkingSet.ATwset[b_i + QRRowBound] = 0.0;
    }
    WorkingSet.ATwset[(WorkingSet.indexFixed[idx_local] + QRRowBound) - 1] =
        1.0;
    i = WorkingSet.indexFixed[idx_local] + 1;
    idxFillStart = WorkingSet.nVar;
    for (b_i = i; b_i <= idxFillStart; b_i++) {
      WorkingSet.ATwset[(b_i + QRRowBound) - 1] = 0.0;
    }
    WorkingSet.bwset[idx_local] =
        WorkingSet.ub[WorkingSet.indexFixed[idx_local] - 1];
  }
  QRRowBound = WorkingSet.sizes[1];
  for (idx_local = 0; idx_local < QRRowBound; idx_local++) {
    idxFillStart = nWFixed + idx_local;
    WorkingSet.Wid[idxFillStart] = 2;
    WorkingSet.Wlocalidx[idxFillStart] = idx_local + 1;
    WorkingSet.isActiveConstr[idxFillStart] = true;
    iAeq0 = WorkingSet.ldA * idx_local;
    mConstrMax = WorkingSet.ldA * idxFillStart;
    i = WorkingSet.nVar - 1;
    for (b_i = 0; b_i <= i; b_i++) {
      WorkingSet.ATwset[mConstrMax + b_i] = WorkingSet.Aeq[iAeq0 + b_i];
    }
    WorkingSet.bwset[idxFillStart] = WorkingSet.beq[idx_local];
  }
  WorkingSet.SLACK0 = 0.0;
  if (2 < x0.size(0) + 1) {
    i = x0.size(0) + 1;
  } else {
    i = 2;
  }
  memspace.workspace_double.set_size(maxDims, i);
  memspace.workspace_int.set_size(maxDims);
  memspace.workspace_sort.set_size(maxDims);
  QRRowBound = WorkingSet.nVarOrig - 1;
  tol = 1.0;
  i = WorkingSet.sizes[1];
  for (mConstrMax = 0; mConstrMax < i; mConstrMax++) {
    colSum = 0.0;
    iAeq0 = WorkingSet.ldA * mConstrMax;
    for (nWFixed = 0; nWFixed <= QRRowBound; nWFixed++) {
      colSum += std::abs(WorkingSet.Aeq[nWFixed + iAeq0]);
    }
    tol = std::fmax(tol, colSum);
  }
  i = WorkingSet.sizes[2];
  for (mConstrMax = 0; mConstrMax < i; mConstrMax++) {
    colSum = 0.0;
    iAeq0 = WorkingSet.ldA * mConstrMax;
    for (nWFixed = 0; nWFixed <= QRRowBound; nWFixed++) {
      colSum += std::abs(WorkingSet.Aineq[nWFixed + iAeq0]);
    }
    tol = std::fmax(tol, colSum);
  }
  H_infnrm = 0.0;
  if (f.size(0) != 0) {
    i = H.size(1);
    for (mConstrMax = 0; mConstrMax < i; mConstrMax++) {
      colSum = 0.0;
      idxFillStart = H.size(0);
      for (nWFixed = 0; nWFixed < idxFillStart; nWFixed++) {
        colSum += std::abs(H[nWFixed + H.size(0) * mConstrMax]);
      }
      H_infnrm = std::fmax(H_infnrm, colSum);
    }
  } else {
    i = H.size(1);
    for (mConstrMax = 0; mConstrMax < i; mConstrMax++) {
      colSum = 0.0;
      idxFillStart = H.size(0);
      for (nWFixed = 0; nWFixed < idxFillStart; nWFixed++) {
        colSum += std::abs(H[nWFixed + H.size(0) * mConstrMax]);
      }
      H_infnrm = std::fmax(H_infnrm, colSum);
    }
  }
  expl_temp.RemainFeasible = false;
  expl_temp.ProbRelTolFactor = std::fmax(tol, H_infnrm);
  expl_temp.ConstrRelTolFactor = tol;
  expl_temp.MaxIterations =
      10 *
      (((((x0.size(0) + mFixed) + beq.size(0)) + bineq.size(0)) + mLB) + mUB);
  c_options = *b_options;
  optim::coder::qpactiveset::driver(H, f, &solution, &memspace, &WorkingSet,
                                    &QRManager, &CholRegManager, &QPObjective,
                                    &c_options, expl_temp);
  x.set_size(x0.size(0));
  for (QRRowBound = 0; QRRowBound <= nVar; QRRowBound++) {
    x[QRRowBound] = solution.xstar[QRRowBound];
  }
}

} // namespace coder

//
// File trailer for quadprog.cpp
//
// [EOF]
//
