//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: feasibleratiotest.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "feasibleratiotest.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xnrm2.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 1U> &solution_xstar
//                const ::coder::array<double, 1U> &solution_searchDir
//                ::coder::array<double, 2U> &workspace
//                int workingset_nVar
//                int workingset_ldA
//                const ::coder::array<double, 1U> &workingset_Aineq
//                const ::coder::array<double, 1U> &workingset_bineq
//                const ::coder::array<double, 1U> &workingset_lb
//                const ::coder::array<double, 1U> &workingset_ub
//                const ::coder::array<int, 1U> &workingset_indexLB
//                const ::coder::array<int, 1U> &workingset_indexUB
//                const int workingset_sizes[5]
//                const int workingset_isActiveIdx[6]
//                const ::coder::array<boolean_T, 1U> &workingset_isActiveConstr
//                const int workingset_nWConstr[5]
//                boolean_T isPhaseOne
//                double *alpha
//                boolean_T *newBlocking
//                int *constrType
//                int *constrIdx
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void feasibleratiotest(
    const ::coder::array<double, 1U> &solution_xstar,
    const ::coder::array<double, 1U> &solution_searchDir,
    ::coder::array<double, 2U> &workspace, int workingset_nVar,
    int workingset_ldA, const ::coder::array<double, 1U> &workingset_Aineq,
    const ::coder::array<double, 1U> &workingset_bineq,
    const ::coder::array<double, 1U> &workingset_lb,
    const ::coder::array<double, 1U> &workingset_ub,
    const ::coder::array<int, 1U> &workingset_indexLB,
    const ::coder::array<int, 1U> &workingset_indexUB,
    const int workingset_sizes[5], const int workingset_isActiveIdx[6],
    const ::coder::array<boolean_T, 1U> &workingset_isActiveConstr,
    const int workingset_nWConstr[5], boolean_T isPhaseOne, double *alpha,
    boolean_T *newBlocking, int *constrType, int *constrIdx)
{
  double alphaTemp;
  double denomTol;
  double phaseOneCorrectionP;
  double phaseOneCorrectionX;
  double ratio;
  int idx;
  int ldw;
  int totalIneq;
  int totalUB;
  totalIneq = workingset_sizes[2] - 1;
  totalUB = workingset_sizes[4];
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  denomTol = 2.2204460492503131E-13 *
             internal::blas::xnrm2(workingset_nVar, solution_searchDir);
  if (workingset_nWConstr[2] < workingset_sizes[2]) {
    for (ldw = 0; ldw <= totalIneq; ldw++) {
      workspace[ldw] = workingset_bineq[ldw];
    }
    internal::blas::xgemv(workingset_nVar, workingset_sizes[2],
                          workingset_Aineq, workingset_ldA, solution_xstar,
                          workspace);
    ldw = workspace.size(0);
    internal::blas::xgemv(workingset_nVar, workingset_sizes[2],
                          workingset_Aineq, workingset_ldA, solution_searchDir,
                          workspace, workspace.size(0) + 1);
    for (idx = 0; idx <= totalIneq; idx++) {
      if ((workspace[ldw + idx] > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[2] + idx) - 1])) {
        alphaTemp =
            std::fmin(std::abs(workspace[idx]), 1.0E-8 - workspace[idx]) /
            workspace[ldw + idx];
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 3;
          *constrIdx = idx + 1;
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
    ldw = workingset_sizes[3];
    for (idx = 0; idx <= ldw - 2; idx++) {
      alphaTemp = -solution_searchDir[workingset_indexLB[idx] - 1] -
                  phaseOneCorrectionP;
      if ((alphaTemp > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[3] + idx) - 1])) {
        ratio = (-solution_xstar[workingset_indexLB[idx] - 1] -
                 workingset_lb[workingset_indexLB[idx] - 1]) -
                phaseOneCorrectionX;
        alphaTemp = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / alphaTemp;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }
    ldw = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    alphaTemp = -solution_searchDir[ldw];
    if ((alphaTemp > denomTol) &&
        (!workingset_isActiveConstr
             [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio = -solution_xstar[ldw] - workingset_lb[ldw];
      alphaTemp = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / alphaTemp;
      if (alphaTemp < *alpha) {
        *alpha = alphaTemp;
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
    for (idx = 0; idx < totalUB; idx++) {
      alphaTemp =
          solution_searchDir[workingset_indexUB[idx] - 1] - phaseOneCorrectionP;
      if ((alphaTemp > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[4] + idx) - 1])) {
        ratio = (solution_xstar[workingset_indexUB[idx] - 1] -
                 workingset_ub[workingset_indexUB[idx] - 1]) -
                phaseOneCorrectionX;
        alphaTemp = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / alphaTemp;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 5;
          *constrIdx = idx + 1;
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

//
// File trailer for feasibleratiotest.cpp
//
// [EOF]
//
