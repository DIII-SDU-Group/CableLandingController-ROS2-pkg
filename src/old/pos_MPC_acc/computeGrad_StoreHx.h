//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeGrad_StoreHx.h
//
// Code generation for function 'computeGrad_StoreHx'
//

#ifndef COMPUTEGRAD_STOREHX_H
#define COMPUTEGRAD_STOREHX_H

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
void computeGrad_StoreHx(b_struct_T *obj, const double H[14400],
                         const double f[120], const double x[121]);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (computeGrad_StoreHx.h)
