//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// driver.h
//
// Code generation for function 'driver'
//

#ifndef DRIVER_H
#define DRIVER_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace pos_MPC {
struct struct_T;

struct e_struct_T;

struct g_struct_T;

struct f_struct_T;

struct c_struct_T;

struct d_struct_T;

struct b_struct_T;

} // namespace pos_MPC

// Function Declarations
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void driver(const double H[32400], const double f[180], struct_T *solution,
            e_struct_T *memspace, g_struct_T *workingset, f_struct_T *qrmanager,
            c_struct_T *cholmanager, d_struct_T runTimeOptions,
            b_struct_T *objective);

}
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (driver.h)
