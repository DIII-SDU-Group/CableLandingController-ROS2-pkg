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
#include "xnrm2.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace internal {
namespace reflapack {
void qrf(double A[144300], int m, int n, int nfxd, double tau[300])
{
  double work[481];
  double atmp;
  std::memset(&work[0], 0, 481U * sizeof(double));
  for (int i{0}; i < nfxd; i++) {
    int ii;
    int mmi;
    ii = i * 300 + i;
    mmi = m - i;
    if (i + 1 < m) {
      atmp = A[ii];
      tau[i] = xzlarfg(mmi, &atmp, A, ii + 2);
      A[ii] = atmp;
    } else {
      tau[i] = 0.0;
    }
    if (i + 1 < n) {
      atmp = A[ii];
      A[ii] = 1.0;
      xzlarf(mmi, (n - i) - 1, ii + 1, tau[i], A, ii + 301, work);
      A[ii] = atmp;
    }
  }
}

void qrpf(double A[144300], int m, int n, int nfxd, double tau[300],
          int jpvt[481])
{
  double vn1[481];
  double vn2[481];
  double work[481];
  double smax;
  int i;
  int j;
  int minmn;
  if (m < n) {
    minmn = m;
  } else {
    minmn = n;
  }
  std::memset(&work[0], 0, 481U * sizeof(double));
  std::memset(&vn1[0], 0, 481U * sizeof(double));
  std::memset(&vn2[0], 0, 481U * sizeof(double));
  i = nfxd + 1;
  for (j = i; j <= n; j++) {
    smax = blas::xnrm2(m - nfxd, A, (nfxd + (j - 1) * 300) + 1);
    vn1[j - 1] = smax;
    vn2[j - 1] = smax;
  }
  for (int b_i{i}; b_i <= minmn; b_i++) {
    double s;
    int ii;
    int ii_tmp;
    int ip1;
    int ix;
    int k;
    int mmi;
    int nmi;
    int pvt;
    ip1 = b_i + 1;
    ii_tmp = (b_i - 1) * 300;
    ii = (ii_tmp + b_i) - 1;
    nmi = (n - b_i) + 1;
    mmi = m - b_i;
    if (nmi < 1) {
      ix = -2;
    } else {
      ix = -1;
      if (nmi > 1) {
        smax = std::abs(vn1[b_i - 1]);
        for (k = 2; k <= nmi; k++) {
          s = std::abs(vn1[(b_i + k) - 2]);
          if (s > smax) {
            ix = k - 2;
            smax = s;
          }
        }
      }
    }
    pvt = b_i + ix;
    if (pvt + 1 != b_i) {
      ix = pvt * 300;
      for (k = 0; k < m; k++) {
        int i1;
        j = ix + k;
        smax = A[j];
        i1 = ii_tmp + k;
        A[j] = A[i1];
        A[i1] = smax;
      }
      ix = jpvt[pvt];
      jpvt[pvt] = jpvt[b_i - 1];
      jpvt[b_i - 1] = ix;
      vn1[pvt] = vn1[b_i - 1];
      vn2[pvt] = vn2[b_i - 1];
    }
    if (b_i < m) {
      smax = A[ii];
      tau[b_i - 1] = xzlarfg(mmi + 1, &smax, A, ii + 2);
      A[ii] = smax;
    } else {
      tau[b_i - 1] = 0.0;
    }
    if (b_i < n) {
      smax = A[ii];
      A[ii] = 1.0;
      xzlarf(mmi + 1, nmi - 1, ii + 1, tau[b_i - 1], A, ii + 301, work);
      A[ii] = smax;
    }
    for (j = ip1; j <= n; j++) {
      ix = b_i + (j - 1) * 300;
      smax = vn1[j - 1];
      if (smax != 0.0) {
        double temp2;
        s = std::abs(A[ix - 1]) / smax;
        s = 1.0 - s * s;
        if (s < 0.0) {
          s = 0.0;
        }
        temp2 = smax / vn2[j - 1];
        temp2 = s * (temp2 * temp2);
        if (temp2 <= 1.4901161193847656E-8) {
          if (b_i < m) {
            smax = blas::xnrm2(mmi, A, ix + 1);
            vn1[j - 1] = smax;
            vn2[j - 1] = smax;
          } else {
            vn1[j - 1] = 0.0;
            vn2[j - 1] = 0.0;
          }
        } else {
          vn1[j - 1] = smax * std::sqrt(s);
        }
      }
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder
} // namespace pos_MPC

// End of code generation (xzgeqp3.cpp)
