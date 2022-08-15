//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ratiotest.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef RATIOTEST_H
#define RATIOTEST_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
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
               int *constrIdx);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for ratiotest.h
//
// [EOF]
//
