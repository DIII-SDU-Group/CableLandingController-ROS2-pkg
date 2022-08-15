//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: linearForm_.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef LINEARFORM__H
#define LINEARFORM__H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
void linearForm_(boolean_T obj_hasLinear, int obj_nvar,
                 ::coder::array<double, 2U> &workspace,
                 const ::coder::array<double, 2U> &H,
                 const ::coder::array<double, 1U> &x);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for linearForm_.h
//
// [EOF]
//
