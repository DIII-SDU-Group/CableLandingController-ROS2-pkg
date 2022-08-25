//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeFval.h
//
// Code generation for function 'computeFval'
//

#ifndef COMPUTEFVAL_H
#define COMPUTEFVAL_H

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
double computeFval(const b_struct_T *obj, double workspace[38841],
                   const double H[14400], const double f[120],
                   const double x[121]);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (computeFval.h)
