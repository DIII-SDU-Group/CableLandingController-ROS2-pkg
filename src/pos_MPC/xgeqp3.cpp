//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xgeqp3.cpp
//
// Code generation for function 'xgeqp3'
//

// Include files
#include "xgeqp3.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"
#include <cstring>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace internal {
namespace lapack {
void xgeqp3(double A[144300], int m, int n, int jpvt[481], double tau[300])
{
  int minmn;
  if (m < n) {
    minmn = m;
  } else {
    minmn = n;
  }
  std::memset(&tau[0], 0, 300U * sizeof(double));
  if (minmn < 1) {
    for (int j{0}; j < n; j++) {
      jpvt[j] = j + 1;
    }
  } else {
    int nfxd;
    nfxd = 0;
    for (int j{0}; j < n; j++) {
      if (jpvt[j] != 0) {
        nfxd++;
        if (j + 1 != nfxd) {
          int ix;
          int iy;
          ix = j * 300;
          iy = (nfxd - 1) * 300;
          for (int k{0}; k < m; k++) {
            double temp;
            int i;
            int temp_tmp;
            temp_tmp = ix + k;
            temp = A[temp_tmp];
            i = iy + k;
            A[temp_tmp] = A[i];
            A[i] = temp;
          }
          jpvt[j] = jpvt[nfxd - 1];
          jpvt[nfxd - 1] = j + 1;
        } else {
          jpvt[j] = j + 1;
        }
      } else {
        jpvt[j] = j + 1;
      }
    }
    if (nfxd >= minmn) {
      nfxd = minmn;
    }
    reflapack::qrf(A, m, n, nfxd, tau);
    if (nfxd < minmn) {
      reflapack::qrpf(A, m, n, nfxd, tau, jpvt);
    }
  }
}

} // namespace lapack
} // namespace internal
} // namespace coder
} // namespace pos_MPC

// End of code generation (xgeqp3.cpp)
