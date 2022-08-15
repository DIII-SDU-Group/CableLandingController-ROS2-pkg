//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemv.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "xgemv.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : int m
//                int n
//                const ::coder::array<double, 1U> &b_A
//                int lda
//                const ::coder::array<double, 1U> &x
//                ::coder::array<double, 2U> &y
// Return Type  : void
//
namespace coder {
namespace internal {
namespace blas {
void xgemv(int m, int n, const ::coder::array<double, 1U> &b_A, int lda,
           const ::coder::array<double, 1U> &x, ::coder::array<double, 2U> &y)
{
  if (n != 0) {
    int i;
    int iy;
    for (iy = 0; iy < n; iy++) {
      y[iy] = -y[iy];
    }
    iy = 0;
    i = lda * (n - 1) + 1;
    for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        c += b_A[ia - 1] * x[ia - iac];
      }
      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

//
// Arguments    : int m
//                int n
//                const ::coder::array<double, 1U> &b_A
//                int lda
//                const ::coder::array<double, 1U> &x
//                ::coder::array<double, 2U> &y
//                int iy0
// Return Type  : void
//
void xgemv(int m, int n, const ::coder::array<double, 1U> &b_A, int lda,
           const ::coder::array<double, 1U> &x, ::coder::array<double, 2U> &y,
           int iy0)
{
  if (n != 0) {
    int iy;
    int iyend;
    iyend = (iy0 + n) - 1;
    for (iy = iy0; iy <= iyend; iy++) {
      y[iy - 1] = 0.0;
    }
    iy = iy0 - 1;
    iyend = lda * (n - 1) + 1;
    for (int iac{1}; lda < 0 ? iac >= iyend : iac <= iyend; iac += lda) {
      double c;
      int i;
      c = 0.0;
      i = (iac + m) - 1;
      for (int ia{iac}; ia <= i; ia++) {
        c += b_A[ia - 1] * x[ia - iac];
      }
      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

//
// Arguments    : int m
//                int n
//                const ::coder::array<double, 1U> &b_A
//                int lda
//                const ::coder::array<double, 2U> &x
//                ::coder::array<double, 1U> &y
// Return Type  : void
//
void xgemv(int m, int n, const ::coder::array<double, 1U> &b_A, int lda,
           const ::coder::array<double, 2U> &x, ::coder::array<double, 1U> &y)
{
  if (n != 0) {
    int i;
    int iy;
    for (iy = 0; iy < n; iy++) {
      y[iy] = -y[iy];
    }
    iy = 0;
    i = lda * (n - 1) + 1;
    for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        c += b_A[ia - 1] * x[ia - iac];
      }
      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

//
// Arguments    : int m
//                int n
//                const ::coder::array<double, 1U> &b_A
//                int lda
//                const ::coder::array<double, 2U> &x
//                int ix0
//                ::coder::array<double, 1U> &y
// Return Type  : void
//
void xgemv(int m, int n, const ::coder::array<double, 1U> &b_A, int lda,
           const ::coder::array<double, 2U> &x, int ix0,
           ::coder::array<double, 1U> &y)
{
  if (n != 0) {
    int i;
    int iy;
    for (iy = 0; iy < n; iy++) {
      y[iy] = -y[iy];
    }
    iy = 0;
    i = lda * (n - 1) + 1;
    for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        c += b_A[ia - 1] * x[((ix0 + ia) - iac) - 1];
      }
      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

//
// Arguments    : int m
//                int n
//                const ::coder::array<double, 1U> &b_A
//                int lda
//                const ::coder::array<double, 1U> &x
//                ::coder::array<double, 1U> &y
// Return Type  : void
//
void xgemv(int m, int n, const ::coder::array<double, 1U> &b_A, int lda,
           const ::coder::array<double, 1U> &x, ::coder::array<double, 1U> &y)
{
  if (n != 0) {
    int i;
    int iy;
    for (iy = 0; iy < n; iy++) {
      y[iy] = -y[iy];
    }
    iy = 0;
    i = lda * (n - 1) + 1;
    for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        c += b_A[ia - 1] * x[ia - iac];
      }
      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for xgemv.cpp
//
// [EOF]
//
