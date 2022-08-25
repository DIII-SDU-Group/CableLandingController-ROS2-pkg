//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// feasibleratiotest.h
//
// Code generation for function 'feasibleratiotest'
//

#ifndef FEASIBLERATIOTEST_H
#define FEASIBLERATIOTEST_H

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
void feasibleratiotest(
    const double solution_xstar[161], const double solution_searchDir[161],
    double workspace[90321], int workingset_nVar,
    const double workingset_Aineq[25760], const double workingset_bineq[160],
    const double workingset_lb[161], const double workingset_ub[161],
    const int workingset_indexLB[161], const int workingset_indexUB[161],
    const int workingset_sizes[5], const int workingset_isActiveIdx[6],
    const boolean_T workingset_isActiveConstr[561],
    const int workingset_nWConstr[5], boolean_T isPhaseOne, double *alpha,
    boolean_T *newBlocking, int *constrType, int *constrIdx);

}
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (feasibleratiotest.h)
