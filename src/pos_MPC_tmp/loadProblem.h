//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// loadProblem.h
//
// Code generation for function 'loadProblem'
//

#ifndef LOADPROBLEM_H
#define LOADPROBLEM_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace pos_MPC {
struct d_struct_T;

}

// Function Declarations
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void loadProblem(d_struct_T *obj, const double Aineq[25600],
                 const double bineq[160], const double Aeq[12800],
                 const double b_beq[80], int mLB, const double lb[160], int mUB,
                 const double ub[160], int mFixed);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (loadProblem.h)
