//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// deleteColMoveEnd.cpp
//
// Code generation for function 'deleteColMoveEnd'
//

// Include files
#include "deleteColMoveEnd.h"
#include "MPCStepFunction_internal_types.h"
#include "rt_nonfinite.h"
#include "xrotg.h"

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace QRManager {
void deleteColMoveEnd(f_struct_T *obj, int idx)
{
  double c;
  double s;
  double temp_tmp;
  int i;
  if (obj->usedPivoting) {
    i = 1;
    while ((i <= obj->ncols) && (obj->jpvt[i - 1] != idx)) {
      i++;
    }
    idx = i;
  }
  if (idx >= obj->ncols) {
    obj->ncols--;
  } else {
    int b_i;
    int k;
    int u0;
    obj->jpvt[idx - 1] = obj->jpvt[obj->ncols - 1];
    b_i = obj->minRowCol;
    for (k = 0; k < b_i; k++) {
      obj->QR[k + 200 * (idx - 1)] = obj->QR[k + 200 * (obj->ncols - 1)];
    }
    obj->ncols--;
    u0 = obj->mrows;
    i = obj->ncols;
    if (u0 < i) {
      i = u0;
    }
    obj->minRowCol = i;
    if (idx < obj->mrows) {
      double c_temp_tmp;
      int QRk0;
      int b_k;
      int b_temp_tmp;
      int endIdx;
      int n;
      u0 = obj->mrows - 1;
      endIdx = obj->ncols;
      if (u0 < endIdx) {
        endIdx = u0;
      }
      k = endIdx;
      i = 200 * (idx - 1);
      while (k >= idx) {
        b_i = k + i;
        temp_tmp = obj->QR[b_i];
        internal::blas::xrotg(&obj->QR[(k + i) - 1], &temp_tmp, &c, &s);
        obj->QR[b_i] = temp_tmp;
        b_i = 200 * (k - 1);
        obj->QR[k + b_i] = 0.0;
        QRk0 = k + 200 * idx;
        n = obj->ncols - idx;
        if (n >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            b_temp_tmp = QRk0 + b_k * 200;
            temp_tmp = obj->QR[b_temp_tmp];
            c_temp_tmp = obj->QR[b_temp_tmp - 1];
            obj->QR[b_temp_tmp] = c * temp_tmp - s * c_temp_tmp;
            obj->QR[b_temp_tmp - 1] = c * c_temp_tmp + s * temp_tmp;
          }
        }
        n = obj->mrows;
        if (obj->mrows >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            b_temp_tmp = b_i + b_k;
            temp_tmp = obj->Q[b_temp_tmp + 200];
            c_temp_tmp = obj->Q[b_temp_tmp];
            obj->Q[b_temp_tmp + 200] = c * temp_tmp - s * c_temp_tmp;
            obj->Q[b_temp_tmp] = c * c_temp_tmp + s * temp_tmp;
          }
        }
        k--;
      }
      b_i = idx + 1;
      for (k = b_i; k <= endIdx; k++) {
        u0 = 200 * (k - 1);
        i = k + u0;
        temp_tmp = obj->QR[i];
        internal::blas::xrotg(&obj->QR[(k + u0) - 1], &temp_tmp, &c, &s);
        obj->QR[i] = temp_tmp;
        QRk0 = k * 201;
        n = obj->ncols - k;
        if (n >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            b_temp_tmp = QRk0 + b_k * 200;
            temp_tmp = obj->QR[b_temp_tmp];
            c_temp_tmp = obj->QR[b_temp_tmp - 1];
            obj->QR[b_temp_tmp] = c * temp_tmp - s * c_temp_tmp;
            obj->QR[b_temp_tmp - 1] = c * c_temp_tmp + s * temp_tmp;
          }
        }
        n = obj->mrows;
        if (obj->mrows >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            b_temp_tmp = u0 + b_k;
            temp_tmp = obj->Q[b_temp_tmp + 200];
            c_temp_tmp = obj->Q[b_temp_tmp];
            obj->Q[b_temp_tmp + 200] = c * temp_tmp - s * c_temp_tmp;
            obj->Q[b_temp_tmp] = c * c_temp_tmp + s * temp_tmp;
          }
        }
      }
    }
  }
}

} // namespace QRManager
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (deleteColMoveEnd.cpp)
