//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// feasibleratiotest.cpp
//
// Code generation for function 'feasibleratiotest'
//

// Include files
#include "feasibleratiotest.h"
#include "PositionMPCStepFunction_rtwutil.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void feasibleratiotest(
    const double solution_xstar[161], const double solution_searchDir[161],
    double workspace[90321], int workingset_nVar,
    const double workingset_Aineq[25760], const double workingset_bineq[160],
    const double workingset_lb[161], const double workingset_ub[161],
    const int workingset_indexLB[161], const int workingset_indexUB[161],
    const int workingset_sizes[5], const int workingset_isActiveIdx[6],
    const boolean_T workingset_isActiveConstr[561],
    const int workingset_nWConstr[5], boolean_T isPhaseOne, double *alpha,
    boolean_T *newBlocking, int *constrType, int *constrIdx)
{
  double c;
  double denomTol;
  double phaseOneCorrectionP;
  double phaseOneCorrectionX;
  double ratio;
  int i;
  int ia;
  int k;
  int totalUB;
  totalUB = workingset_sizes[4];
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  denomTol = 2.2204460492503131E-13 *
             internal::blas::xnrm2(workingset_nVar, solution_searchDir);
  if (workingset_nWConstr[2] < 160) {
    for (k = 0; k < 160; k++) {
      workspace[k] = workingset_bineq[k];
      workspace[k] = -workspace[k];
    }
    for (k = 0; k <= 25599; k += 161) {
      c = 0.0;
      i = k + workingset_nVar;
      for (ia = k + 1; ia <= i; ia++) {
        c += workingset_Aineq[ia - 1] * solution_xstar[(ia - k) - 1];
      }
      i = div_nde_s32_floor(k, 161);
      workspace[i] += c;
    }
    std::memset(&workspace[561], 0, 160U * sizeof(double));
    for (k = 0; k <= 25599; k += 161) {
      c = 0.0;
      i = k + workingset_nVar;
      for (ia = k + 1; ia <= i; ia++) {
        c += workingset_Aineq[ia - 1] * solution_searchDir[(ia - k) - 1];
      }
      i = div_nde_s32_floor(k, 161) + 561;
      workspace[i] += c;
    }
    for (ia = 0; ia < 160; ia++) {
      c = workspace[ia + 561];
      if ((c > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[2] + ia) - 1])) {
        c = std::fmin(std::abs(workspace[ia]), 1.0E-8 - workspace[ia]) / c;
        if (c < *alpha) {
          *alpha = c;
          *constrType = 3;
          *constrIdx = ia + 1;
          *newBlocking = true;
        }
      }
    }
  }
  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    phaseOneCorrectionX =
        static_cast<double>(isPhaseOne) * solution_xstar[workingset_nVar - 1];
    phaseOneCorrectionP = static_cast<double>(isPhaseOne) *
                          solution_searchDir[workingset_nVar - 1];
    i = workingset_sizes[3];
    for (ia = 0; ia <= i - 2; ia++) {
      k = workingset_indexLB[ia];
      c = -solution_searchDir[k - 1] - phaseOneCorrectionP;
      if ((c > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[3] + ia) - 1])) {
        ratio = (-solution_xstar[k - 1] - workingset_lb[k - 1]) -
                phaseOneCorrectionX;
        c = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / c;
        if (c < *alpha) {
          *alpha = c;
          *constrType = 4;
          *constrIdx = ia + 1;
          *newBlocking = true;
        }
      }
    }
    i = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    c = -solution_searchDir[i];
    if ((c > denomTol) &&
        (!workingset_isActiveConstr
             [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio = -solution_xstar[i] - workingset_lb[i];
      c = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / c;
      if (c < *alpha) {
        *alpha = c;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
    }
  }
  if (workingset_nWConstr[4] < workingset_sizes[4]) {
    phaseOneCorrectionX =
        static_cast<double>(isPhaseOne) * solution_xstar[workingset_nVar - 1];
    phaseOneCorrectionP = static_cast<double>(isPhaseOne) *
                          solution_searchDir[workingset_nVar - 1];
    for (ia = 0; ia < totalUB; ia++) {
      i = workingset_indexUB[ia];
      c = solution_searchDir[i - 1] - phaseOneCorrectionP;
      if ((c > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[4] + ia) - 1])) {
        ratio = (solution_xstar[i - 1] - workingset_ub[i - 1]) -
                phaseOneCorrectionX;
        c = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / c;
        if (c < *alpha) {
          *alpha = c;
          *constrType = 5;
          *constrIdx = ia + 1;
          *newBlocking = true;
        }
      }
    }
  }
  if (!isPhaseOne) {
    if ((*newBlocking) && (*alpha > 1.0)) {
      *newBlocking = false;
    }
    *alpha = std::fmin(*alpha, 1.0);
  }
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (feasibleratiotest.cpp)
