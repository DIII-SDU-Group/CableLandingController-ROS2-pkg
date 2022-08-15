//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: factorQR.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef FACTORQR_H
#define FACTORQR_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct d_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace QRManager {
void factorQR(d_struct_T *obj, const ::coder::array<double, 1U> &b_A, int mrows,
              int ncols, int ldA);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for factorQR.h
//
// [EOF]
//
