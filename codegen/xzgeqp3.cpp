//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzgeqp3.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "xzgeqp3.h"
#include "rt_nonfinite.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : ::coder::array<double, 2U> &b_A
//                int m
//                int n
//                int nfxd
//                ::coder::array<double, 1U> &tau
// Return Type  : void
//
namespace coder {
namespace internal {
namespace reflapack {
void qrf(::coder::array<double, 2U> &b_A, int m, int n, int nfxd,
         ::coder::array<double, 1U> &tau)
{
  array<double, 1U> work;
  double atmp;
  int ii;
  int lda;
  int mmi;
  lda = b_A.size(0);
  work.set_size(b_A.size(1));
  ii = b_A.size(1);
  for (mmi = 0; mmi < ii; mmi++) {
    work[mmi] = 0.0;
  }
  for (int i{0}; i < nfxd; i++) {
    ii = i * lda + i;
    mmi = m - i;
    if (i + 1 < m) {
      atmp = b_A[ii];
      tau[i] = xzlarfg(mmi, &atmp, b_A, ii + 2);
      b_A[ii] = atmp;
    } else {
      tau[i] = 0.0;
    }
    if (i + 1 < n) {
      atmp = b_A[ii];
      b_A[ii] = 1.0;
      xzlarf(mmi, (n - i) - 1, ii + 1, tau[i], b_A, (ii + lda) + 1, lda, work);
      b_A[ii] = atmp;
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xzgeqp3.cpp
//
// [EOF]
//
