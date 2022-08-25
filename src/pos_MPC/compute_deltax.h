//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// compute_deltax.h
//
// Code generation for function 'compute_deltax'
//

#ifndef COMPUTE_DELTAX_H
#define COMPUTE_DELTAX_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace pos_MPC {
struct struct_T;

struct e_struct_T;

struct f_struct_T;

struct c_struct_T;

struct b_struct_T;

} // namespace pos_MPC

// Function Declarations
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void compute_deltax(const double H[32400], struct_T *solution,
                    e_struct_T *memspace, const f_struct_T *qrmanager,
                    c_struct_T *cholmanager, const b_struct_T *objective);

}
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (compute_deltax.h)
