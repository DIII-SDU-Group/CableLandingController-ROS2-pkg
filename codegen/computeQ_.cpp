//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeQ_.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "computeQ_.h"
#include "mpc_fun_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : d_struct_T *obj
//                int nrows
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace QRManager {
void computeQ_(d_struct_T *obj, int nrows)
{
  ::coder::array<double, 2U> *b_A;
  array<double, 1U> work;
  int i;
  int i1;
  int iQR0;
  int lastc;
  int lastv;
  int lda;
  int m;
  int n;
  i = obj->minRowCol;
  for (lastc = 0; lastc < i; lastc++) {
    iQR0 = obj->ldq * lastc + lastc;
    n = obj->mrows - lastc;
    for (lastv = 0; lastv <= n - 2; lastv++) {
      i1 = (iQR0 + lastv) + 1;
      obj->Q[i1] = obj->QR[i1];
    }
  }
  m = obj->mrows;
  lastv = obj->minRowCol;
  lda = obj->ldq;
  if (nrows >= 1) {
    int b_i;
    int ia;
    int itau;
    i = nrows - 1;
    for (n = lastv; n <= i; n++) {
      ia = n * lda;
      i1 = m - 1;
      for (b_i = 0; b_i <= i1; b_i++) {
        obj->Q[ia + b_i] = 0.0;
      }
      obj->Q[ia + n] = 1.0;
    }
    itau = obj->minRowCol - 1;
    iQR0 = obj->Q.size(1);
    work.set_size(iQR0);
    for (i = 0; i < iQR0; i++) {
      work[i] = 0.0;
    }
    for (b_i = obj->minRowCol; b_i >= 1; b_i--) {
      int iaii;
      iaii = b_i + (b_i - 1) * lda;
      if (b_i < nrows) {
        int jA;
        obj->Q[iaii - 1] = 1.0;
        jA = iaii + lda;
        if (obj->tau[itau] != 0.0) {
          boolean_T exitg2;
          lastv = (m - b_i) + 1;
          iQR0 = (iaii + m) - b_i;
          while ((lastv > 0) && (obj->Q[iQR0 - 1] == 0.0)) {
            lastv--;
            iQR0--;
          }
          lastc = nrows - b_i;
          exitg2 = false;
          while ((!exitg2) && (lastc > 0)) {
            int exitg1;
            iQR0 = jA + (lastc - 1) * lda;
            ia = iQR0;
            do {
              exitg1 = 0;
              if (ia <= (iQR0 + lastv) - 1) {
                if (obj->Q[ia - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  ia++;
                }
              } else {
                lastc--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);
            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = 0;
          lastc = 0;
        }
        if (lastv > 0) {
          double c;
          if (lastc != 0) {
            for (iQR0 = 0; iQR0 < lastc; iQR0++) {
              work[iQR0] = 0.0;
            }
            iQR0 = 0;
            i = jA + lda * (lastc - 1);
            for (n = jA; lda < 0 ? n >= i : n <= i; n += lda) {
              c = 0.0;
              i1 = (n + lastv) - 1;
              for (ia = n; ia <= i1; ia++) {
                c += obj->Q[ia - 1] * obj->Q[((iaii + ia) - n) - 1];
              }
              work[iQR0] = work[iQR0] + c;
              iQR0++;
            }
          }
          c = -obj->tau[itau];
          b_A = &obj->Q;
          if (!(c == 0.0)) {
            for (n = 0; n < lastc; n++) {
              if (work[n] != 0.0) {
                double temp;
                temp = work[n] * c;
                i = lastv + jA;
                for (iQR0 = jA; iQR0 < i; iQR0++) {
                  (*b_A)[iQR0 - 1] = (*b_A)[iQR0 - 1] +
                                     (*b_A)[((iaii + iQR0) - jA) - 1] * temp;
                }
              }
              jA += lda;
            }
          }
        }
      }
      if (b_i < m) {
        iQR0 = iaii + 1;
        i = (iaii + m) - b_i;
        for (lastv = iQR0; lastv <= i; lastv++) {
          obj->Q[lastv - 1] = -obj->tau[itau] * obj->Q[lastv - 1];
        }
      }
      obj->Q[iaii - 1] = 1.0 - obj->tau[itau];
      for (n = 0; n <= b_i - 2; n++) {
        obj->Q[(iaii - n) - 2] = 0.0;
      }
      itau--;
    }
  }
}

} // namespace QRManager
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeQ_.cpp
//
// [EOF]
//