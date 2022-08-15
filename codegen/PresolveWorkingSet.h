//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: PresolveWorkingSet.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef PRESOLVEWORKINGSET_H
#define PRESOLVEWORKINGSET_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct b_struct_T;

struct g_struct_T;

struct f_struct_T;

struct d_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
void PresolveWorkingSet(b_struct_T *solution, g_struct_T *memspace,
                        f_struct_T *workingset, d_struct_T *qrmanager);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for PresolveWorkingSet.h
//
// [EOF]
//
