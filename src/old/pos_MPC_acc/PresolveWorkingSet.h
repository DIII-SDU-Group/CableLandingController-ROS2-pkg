//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// PresolveWorkingSet.h
//
// Code generation for function 'PresolveWorkingSet'
//

#ifndef PRESOLVEWORKINGSET_H
#define PRESOLVEWORKINGSET_H

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

} // namespace pos_MPC

// Function Declarations
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
void PresolveWorkingSet(struct_T *solution, e_struct_T *memspace,
                        g_struct_T *workingset, f_struct_T *qrmanager);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (PresolveWorkingSet.h)
