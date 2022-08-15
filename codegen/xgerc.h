//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgerc.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef XGERC_H
#define XGERC_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
void xgerc(int m, int n, double alpha1, int ix0,
           const ::coder::array<double, 1U> &y, ::coder::array<double, 2U> &b_A,
           int ia0, int lda);

}
} // namespace internal
} // namespace coder

#endif
//
// File trailer for xgerc.h
//
// [EOF]
//
