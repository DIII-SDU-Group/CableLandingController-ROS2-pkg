//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// RemoveDependentEq_.h
//
// Code generation for function 'RemoveDependentEq_'
//

#ifndef REMOVEDEPENDENTEQ__H
#define REMOVEDEPENDENTEQ__H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace pos_MPC {
struct e_struct_T;

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
int RemoveDependentEq_(e_struct_T *memspace, g_struct_T *workingset,
                       f_struct_T *qrmanager);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (RemoveDependentEq_.h)
