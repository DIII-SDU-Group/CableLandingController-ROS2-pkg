//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: compute_deltax.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef COMPUTE_DELTAX_H
#define COMPUTE_DELTAX_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct b_struct_T;

struct g_struct_T;

struct d_struct_T;

struct e_struct_T;

struct c_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void compute_deltax(const ::coder::array<double, 2U> &H, b_struct_T *solution,
                    g_struct_T *memspace, const d_struct_T *qrmanager,
                    e_struct_T *cholmanager, const c_struct_T *objective);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for compute_deltax.h
//
// [EOF]
//
