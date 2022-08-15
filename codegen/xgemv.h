//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemv.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef XGEMV_H
#define XGEMV_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
void xgemv(int m, int n, const ::coder::array<double, 1U> &b_A, int lda,
           const ::coder::array<double, 1U> &x, ::coder::array<double, 2U> &y);

void xgemv(int m, int n, const ::coder::array<double, 1U> &b_A, int lda,
           const ::coder::array<double, 1U> &x, ::coder::array<double, 2U> &y,
           int iy0);

void xgemv(int m, int n, const ::coder::array<double, 1U> &b_A, int lda,
           const ::coder::array<double, 2U> &x, ::coder::array<double, 1U> &y);

void xgemv(int m, int n, const ::coder::array<double, 1U> &b_A, int lda,
           const ::coder::array<double, 2U> &x, int ix0,
           ::coder::array<double, 1U> &y);

void xgemv(int m, int n, const ::coder::array<double, 1U> &b_A, int lda,
           const ::coder::array<double, 1U> &x, ::coder::array<double, 1U> &y);

} // namespace blas
} // namespace internal
} // namespace coder

#endif
//
// File trailer for xgemv.h
//
// [EOF]
//
