//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ratiotest.cpp
//
// Code generation for function 'ratiotest'
//

// Include files
#include "ratiotest.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include <cmath>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void ratiotest(const double solution_xstar[181],
               const double solution_searchDir[181], int workingset_nVar,
               const double workingset_lb[181], const double workingset_ub[181],
               const int workingset_indexLB[181],
               const int workingset_indexUB[181], const int workingset_sizes[5],
               const int workingset_isActiveIdx[6],
               const boolean_T workingset_isActiveConstr[481],
               const int workingset_nWConstr[5], double *toldelta,
               double *alpha, boolean_T *newBlocking, int *constrType,
               int *constrIdx)
{
  double denomTol;
  double p_max;
  double phaseOneCorrectionP;
  double phaseOneCorrectionX;
  double pk_corrected;
  double ratio;
  double ratio_tmp;
  int i;
  int idx;
  int totalUB;
  totalUB = workingset_sizes[4];
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  p_max = 0.0;
  denomTol = 2.2204460492503131E-13 *
             internal::blas::xnrm2(workingset_nVar, solution_searchDir);
  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    phaseOneCorrectionX = 0.0 * solution_xstar[workingset_nVar - 1];
    phaseOneCorrectionP = 0.0 * solution_searchDir[workingset_nVar - 1];
    i = workingset_sizes[3];
    for (idx = 0; idx <= i - 2; idx++) {
      int i1;
      i1 = workingset_indexLB[idx];
      pk_corrected = -solution_searchDir[i1 - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[3] + idx) - 1])) {
        ratio_tmp = -solution_xstar[i1 - 1] - workingset_lb[i1 - 1];
        ratio = (ratio_tmp - *toldelta) - phaseOneCorrectionX;
        ratio = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / pk_corrected;
        if ((ratio <= *alpha) && (std::abs(pk_corrected) > p_max)) {
          *alpha = ratio;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
        ratio = ratio_tmp - phaseOneCorrectionX;
        ratio = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / pk_corrected;
        if (ratio < *alpha) {
          *alpha = ratio;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
          p_max = std::abs(pk_corrected);
        }
      }
    }
    i = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    phaseOneCorrectionX = solution_searchDir[i];
    if ((-phaseOneCorrectionX > denomTol) &&
        (!workingset_isActiveConstr
             [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio_tmp = -solution_xstar[i] - workingset_lb[i];
      ratio = ratio_tmp - *toldelta;
      ratio = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / -phaseOneCorrectionX;
      if ((ratio <= *alpha) && (std::abs(phaseOneCorrectionX) > p_max)) {
        *alpha = ratio;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
      ratio = std::fmin(std::abs(ratio_tmp), 1.0E-8 - ratio_tmp) /
              -phaseOneCorrectionX;
      if (ratio < *alpha) {
        *alpha = ratio;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
        p_max = std::abs(solution_searchDir[i]);
      }
    }
  }
  if (workingset_nWConstr[4] < workingset_sizes[4]) {
    phaseOneCorrectionX = 0.0 * solution_xstar[workingset_nVar - 1];
    phaseOneCorrectionP = 0.0 * solution_searchDir[workingset_nVar - 1];
    for (idx = 0; idx < totalUB; idx++) {
      i = workingset_indexUB[idx];
      pk_corrected = solution_searchDir[i - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[4] + idx) - 1])) {
        ratio_tmp = solution_xstar[i - 1] - workingset_ub[i - 1];
        ratio = (ratio_tmp - *toldelta) - phaseOneCorrectionX;
        ratio = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / pk_corrected;
        if ((ratio <= *alpha) && (std::abs(pk_corrected) > p_max)) {
          *alpha = ratio;
          *constrType = 5;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
        ratio = ratio_tmp - phaseOneCorrectionX;
        ratio = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / pk_corrected;
        if (ratio < *alpha) {
          *alpha = ratio;
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
} // namespace pos_MPC

// End of code generation (ratiotest.cpp)