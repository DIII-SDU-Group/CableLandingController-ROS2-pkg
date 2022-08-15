//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: feasibleX0ForWorkingSet.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef FEASIBLEX0FORWORKINGSET_H
#define FEASIBLEX0FORWORKINGSET_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct f_struct_T;

struct d_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
boolean_T feasibleX0ForWorkingSet(::coder::array<double, 2U> &workspace,
                                  ::coder::array<double, 1U> &xCurrent,
                                  f_struct_T *workingset,
                                  d_struct_T *qrmanager);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for feasibleX0ForWorkingSet.h
//
// [EOF]
//
