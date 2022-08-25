//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeFval_ReuseHx.h
//
// Code generation for function 'computeFval_ReuseHx'
//

#ifndef COMPUTEFVAL_REUSEHX_H
#define COMPUTEFVAL_REUSEHX_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace pos_MPC {
struct b_struct_T;

}

// Function Declarations
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
double computeFval_ReuseHx(const b_struct_T *obj, double workspace[87061],
                           const double f[180], const double x[181]);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (computeFval_ReuseHx.h)
