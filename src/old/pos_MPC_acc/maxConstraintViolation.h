//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// maxConstraintViolation.h
//
// Code generation for function 'maxConstraintViolation'
//

#ifndef MAXCONSTRAINTVIOLATION_H
#define MAXCONSTRAINTVIOLATION_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace pos_MPC {
struct g_struct_T;

}

// Function Declarations
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
double maxConstraintViolation(g_struct_T *obj, const double x[121]);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (maxConstraintViolation.h)
