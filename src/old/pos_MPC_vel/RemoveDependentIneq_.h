//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// RemoveDependentIneq_.h
//
// Code generation for function 'RemoveDependentIneq_'
//

#ifndef REMOVEDEPENDENTINEQ__H
#define REMOVEDEPENDENTINEQ__H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace pos_MPC {
struct d_struct_T;

struct g_struct_T;

struct f_struct_T;

} // namespace pos_MPC

// Function Declarations
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
void RemoveDependentIneq_(d_struct_T *workingset, g_struct_T *qrmanager,
                          f_struct_T *memspace, double tolfactor);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (RemoveDependentIneq_.h)
