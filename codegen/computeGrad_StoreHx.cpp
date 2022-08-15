//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeGrad_StoreHx.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "computeGrad_StoreHx.h"
#include "mpc_fun_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : c_struct_T *obj
//                const ::coder::array<double, 2U> &H
//                const ::coder::array<double, 1U> &f
//                const ::coder::array<double, 1U> &x
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
void computeGrad_StoreHx(c_struct_T *obj, const ::coder::array<double, 2U> &H,
                         const ::coder::array<double, 1U> &f,
                         const ::coder::array<double, 1U> &x)
{
  switch (obj->objtype) {
  case 5: {
    int i;
    i = obj->nvar;
    for (int ix{0}; ix <= i - 2; ix++) {
      obj->grad[ix] = 0.0;
    }
    obj->grad[obj->nvar - 1] = obj->gammaScalar;
  } break;
  case 3: {
    int i;
    int ix;
    int lda;
    int m_tmp_tmp;
    m_tmp_tmp = obj->nvar - 1;
    lda = obj->nvar;
    if (obj->nvar != 0) {
      for (ix = 0; ix <= m_tmp_tmp; ix++) {
        obj->Hx[ix] = 0.0;
      }
      ix = 0;
      i = obj->nvar * (obj->nvar - 1) + 1;
      for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
        int i1;
        i1 = iac + m_tmp_tmp;
        for (int ia{iac}; ia <= i1; ia++) {
          int i2;
          i2 = ia - iac;
          obj->Hx[i2] = obj->Hx[i2] + H[ia - 1] * x[ix];
        }
        ix++;
      }
    }
    i = obj->nvar;
    for (ix = 0; ix < i; ix++) {
      obj->grad[ix] = obj->Hx[ix];
    }
    if (obj->hasLinear && (obj->nvar >= 1)) {
      i = obj->nvar - 1;
      for (ix = 0; ix <= i; ix++) {
        obj->grad[ix] = obj->grad[ix] + f[ix];
      }
    }
  } break;
  case 4: {
    int i;
    int ix;
    int lda;
    int m_tmp_tmp;
    int maxRegVar;
    maxRegVar = obj->maxVar - 1;
    m_tmp_tmp = obj->nvar - 1;
    lda = obj->nvar;
    if (obj->nvar != 0) {
      for (ix = 0; ix <= m_tmp_tmp; ix++) {
        obj->Hx[ix] = 0.0;
      }
      ix = 0;
      i = obj->nvar * (obj->nvar - 1) + 1;
      for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
        int i1;
        i1 = iac + m_tmp_tmp;
        for (int ia{iac}; ia <= i1; ia++) {
          int i2;
          i2 = ia - iac;
          obj->Hx[i2] = obj->Hx[i2] + H[ia - 1] * x[ix];
        }
        ix++;
      }
    }
    i = obj->nvar + 1;
    for (ix = i; ix <= maxRegVar; ix++) {
      obj->Hx[ix - 1] = 0.0 * x[ix - 1];
    }
    for (ix = 0; ix < maxRegVar; ix++) {
      obj->grad[ix] = obj->Hx[ix];
    }
    if (obj->hasLinear && (obj->nvar >= 1)) {
      i = obj->nvar - 1;
      for (ix = 0; ix <= i; ix++) {
        obj->grad[ix] = obj->grad[ix] + f[ix];
      }
    }
  } break;
  }
}

} // namespace Objective
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeGrad_StoreHx.cpp
//
// [EOF]
//
