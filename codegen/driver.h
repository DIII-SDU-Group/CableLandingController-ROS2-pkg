//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: driver.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef DRIVER_H
#define DRIVER_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct b_struct_T;

struct g_struct_T;

struct f_struct_T;

struct d_struct_T;

struct e_struct_T;

struct c_struct_T;

struct h_struct_T;

struct struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void driver(const ::coder::array<double, 2U> &H,
            const ::coder::array<double, 1U> &f, b_struct_T *solution,
            g_struct_T *memspace, f_struct_T *workingset, d_struct_T *qrmanager,
            e_struct_T *cholmanager, c_struct_T *objective,
            h_struct_T *b_options, struct_T runTimeOptions);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for driver.h
//
// [EOF]
//
