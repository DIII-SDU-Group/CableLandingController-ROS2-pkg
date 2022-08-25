//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ratiotest.h
//
// Code generation for function 'ratiotest'
//

#ifndef RATIOTEST_H
#define RATIOTEST_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void ratiotest(const double solution_xstar[121],
               const double solution_searchDir[121], int workingset_nVar,
               const double workingset_lb[121], const double workingset_ub[121],
               const int workingset_indexLB[121],
               const int workingset_indexUB[121], const int workingset_sizes[5],
               const int workingset_isActiveIdx[6],
               const boolean_T workingset_isActiveConstr[321],
               const int workingset_nWConstr[5], double *toldelta,
               double *alpha, boolean_T *newBlocking, int *constrType,
               int *constrIdx);

}
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (ratiotest.h)
