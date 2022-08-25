//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzgeqp3.cpp
//
// Code generation for function 'xzgeqp3'
//

// Include files
#include "xzgeqp3.h"
#include "rt_nonfinite.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include <cstring>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace internal {
namespace reflapack {
void qrf(double b_A[64200], int m, int n, int nfxd, double tau[200])
{
  double work[321];
  double atmp;
  std::memset(&work[0], 0, 321U * sizeof(double));
  for (int i{0}; i < nfxd; i++) {
    int ii;
    int mmi;
    ii = i * 200 + i;
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
      xzlarf(mmi, (n - i) - 1, ii + 1, tau[i], b_A, ii + 201, work);
      b_A[ii] = atmp;
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder
} // namespace pos_MPC

// End of code generation (xzgeqp3.cpp)
