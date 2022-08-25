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
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include <cmath>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void feasibleratiotest(
    const double solution_xstar[121], const double solution_searchDir[121],
    int workingset_nVar, const double workingset_lb[121],
    const double workingset_ub[121], const int workingset_indexLB[121],
    const int workingset_indexUB[121], const int workingset_sizes[5],
    const int workingset_isActiveIdx[6],
    const boolean_T workingset_isActiveConstr[321],
    const int workingset_nWConstr[5], boolean_T isPhaseOne, double *alpha,
    boolean_T *newBlocking, int *constrType, int *constrIdx)
{
  double denomTol;
  double phaseOneCorrectionP;
  double phaseOneCorrectionX;
  double pk_corrected;
  double ratio;
  int i;
  int idx;
  int totalUB;
  totalUB = workingset_sizes[4];
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  denomTol = 2.2204460492503131E-13 *
             internal::blas::xnrm2(workingset_nVar, solution_searchDir);
  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    phaseOneCorrectionX =
        static_cast<double>(isPhaseOne) * solution_xstar[workingset_nVar - 1];
    phaseOneCorrectionP = static_cast<double>(isPhaseOne) *
                          solution_searchDir[workingset_nVar - 1];
    i = workingset_sizes[3];
    for (idx = 0; idx <= i - 2; idx++) {
      int i1;
      i1 = workingset_indexLB[idx];
      pk_corrected = -solution_searchDir[i1 - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[3] + idx) - 1])) {
        ratio = (-solution_xstar[i1 - 1] - workingset_lb[i1 - 1]) -
                phaseOneCorrectionX;
        pk_corrected =
            std::fmin(std::abs(ratio), 1.0E-8 - ratio) / pk_corrected;
        if (pk_corrected < *alpha) {
          *alpha = pk_corrected;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }
    i = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    pk_corrected = -solution_searchDir[i];
    if ((pk_corrected > denomTol) &&
        (!workingset_isActiveConstr
             [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio = -solution_xstar[i] - workingset_lb[i];
      pk_corrected = std::fmin(std::abs(ratio), 1.0E-8 - ratio) / pk_corrected;
      if (pk_corrected < *alpha) {
        *alpha = pk_corrected;
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
      i = workingset_indexUB[idx];
      pk_corrected = solution_searchDir[i - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[4] + idx) - 1])) {
        ratio = (solution_xstar[i - 1] - workingset_ub[i - 1]) -
                phaseOneCorrectionX;
        pk_corrected =
            std::fmin(std::abs(ratio), 1.0E-8 - ratio) / pk_corrected;
        if (pk_corrected < *alpha) {
          *alpha = pk_corrected;
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
} // namespace pos_MPC

// End of code generation (feasibleratiotest.cpp)
