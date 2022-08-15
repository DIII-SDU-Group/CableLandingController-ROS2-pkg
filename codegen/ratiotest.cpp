//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ratiotest.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "ratiotest.h"
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
//                double *toldelta
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
void ratiotest(const ::coder::array<double, 1U> &solution_xstar,
               const ::coder::array<double, 1U> &solution_searchDir,
               ::coder::array<double, 2U> &workspace, int workingset_nVar,
               int workingset_ldA,
               const ::coder::array<double, 1U> &workingset_Aineq,
               const ::coder::array<double, 1U> &workingset_bineq,
               const ::coder::array<double, 1U> &workingset_lb,
               const ::coder::array<double, 1U> &workingset_ub,
               const ::coder::array<int, 1U> &workingset_indexLB,
               const ::coder::array<int, 1U> &workingset_indexUB,
               const int workingset_sizes[5],
               const int workingset_isActiveIdx[6],
               const ::coder::array<boolean_T, 1U> &workingset_isActiveConstr,
               const int workingset_nWConstr[5], double *toldelta,
               double *alpha, boolean_T *newBlocking, int *constrType,
               int *constrIdx)
{
  double alphaTemp;
  double denomTol;
  double p_max;
  double phaseOneCorrectionP;
  double phaseOneCorrectionX;
  double pk_corrected;
  double ratio_tmp;
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
  p_max = 0.0;
  denomTol = 2.2204460492503131E-13 *
             internal::blas::xnrm2(workingset_nVar, solution_searchDir);
  if (workingset_nWConstr[2] < workingset_sizes[2]) {
    for (ldw = 0; ldw <= totalIneq; ldw++) {
      workspace[ldw] = workingset_bineq[ldw];
    }
    internal::blas::xgemv(workingset_nVar, workingset_sizes[2],
                          workingset_Aineq, workingset_ldA, solution_xstar,
                          workspace);
    ldw = workspace.size(0) - 1;
    internal::blas::xgemv(workingset_nVar, workingset_sizes[2],
                          workingset_Aineq, workingset_ldA, solution_searchDir,
                          workspace, workspace.size(0) + 1);
    for (idx = 0; idx <= totalIneq; idx++) {
      if ((workspace[(ldw + idx) + 1] > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[2] + idx) - 1])) {
        alphaTemp = std::fmin(std::abs(workspace[idx] - *toldelta),
                              (1.0E-8 - workspace[idx]) + *toldelta) /
                    workspace[(ldw + idx) + 1];
        if ((alphaTemp <= *alpha) &&
            (std::abs(workspace[(ldw + idx) + 1]) > p_max)) {
          *alpha = alphaTemp;
          *constrType = 3;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
        alphaTemp =
            std::fmin(std::abs(workspace[idx]), 1.0E-8 - workspace[idx]) /
            workspace[(ldw + idx) + 1];
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 3;
          *constrIdx = idx + 1;
          *newBlocking = true;
          p_max = std::abs(workspace[(ldw + idx) + 1]);
        }
      }
    }
  }
  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    phaseOneCorrectionX = 0.0 * solution_xstar[workingset_nVar - 1];
    phaseOneCorrectionP = 0.0 * solution_searchDir[workingset_nVar - 1];
    ldw = workingset_sizes[3];
    for (idx = 0; idx <= ldw - 2; idx++) {
      pk_corrected = -solution_searchDir[workingset_indexLB[idx] - 1] -
                     phaseOneCorrectionP;
      if ((pk_corrected > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[3] + idx) - 1])) {
        ratio_tmp = -solution_xstar[workingset_indexLB[idx] - 1] -
                    workingset_lb[workingset_indexLB[idx] - 1];
        alphaTemp = (ratio_tmp - *toldelta) - phaseOneCorrectionX;
        alphaTemp =
            std::fmin(std::abs(alphaTemp), 1.0E-8 - alphaTemp) / pk_corrected;
        if ((alphaTemp <= *alpha) && (std::abs(pk_corrected) > p_max)) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
        alphaTemp = ratio_tmp - phaseOneCorrectionX;
        alphaTemp =
            std::fmin(std::abs(alphaTemp), 1.0E-8 - alphaTemp) / pk_corrected;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
          p_max = std::abs(pk_corrected);
        }
      }
    }
    ldw = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    phaseOneCorrectionX = solution_searchDir[ldw];
    if ((-phaseOneCorrectionX > denomTol) &&
        (!workingset_isActiveConstr
             [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio_tmp = -solution_xstar[ldw] - workingset_lb[ldw];
      alphaTemp = ratio_tmp - *toldelta;
      alphaTemp = std::fmin(std::abs(alphaTemp), 1.0E-8 - alphaTemp) /
                  -phaseOneCorrectionX;
      if ((alphaTemp <= *alpha) && (std::abs(phaseOneCorrectionX) > p_max)) {
        *alpha = alphaTemp;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
      alphaTemp = std::fmin(std::abs(ratio_tmp), 1.0E-8 - ratio_tmp) /
                  -phaseOneCorrectionX;
      if (alphaTemp < *alpha) {
        *alpha = alphaTemp;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
        p_max = std::abs(solution_searchDir[ldw]);
      }
    }
  }
  if (workingset_nWConstr[4] < workingset_sizes[4]) {
    phaseOneCorrectionX = 0.0 * solution_xstar[workingset_nVar - 1];
    phaseOneCorrectionP = 0.0 * solution_searchDir[workingset_nVar - 1];
    for (idx = 0; idx < totalUB; idx++) {
      pk_corrected =
          solution_searchDir[workingset_indexUB[idx] - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[4] + idx) - 1])) {
        ratio_tmp = solution_xstar[workingset_indexUB[idx] - 1] -
                    workingset_ub[workingset_indexUB[idx] - 1];
        alphaTemp = (ratio_tmp - *toldelta) - phaseOneCorrectionX;
        alphaTemp =
            std::fmin(std::abs(alphaTemp), 1.0E-8 - alphaTemp) / pk_corrected;
        if ((alphaTemp <= *alpha) && (std::abs(pk_corrected) > p_max)) {
          *alpha = alphaTemp;
          *constrType = 5;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
        alphaTemp = ratio_tmp - phaseOneCorrectionX;
        alphaTemp =
            std::fmin(std::abs(alphaTemp), 1.0E-8 - alphaTemp) / pk_corrected;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 5;
          *constrIdx = idx + 1;
          *newBlocking = true;
          p_max = std::abs(pk_corrected);
        }
      }
    }
  }
  *toldelta += 6.608625846508183E-7;
  if (p_max > 0.0) {
    *alpha = std::fmax(*alpha, 6.608625846508183E-7 / p_max);
  }
  if ((*newBlocking) && (*alpha > 1.0)) {
    *newBlocking = false;
  }
  *alpha = std::fmin(*alpha, 1.0);
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for ratiotest.cpp
//
// [EOF]
//
