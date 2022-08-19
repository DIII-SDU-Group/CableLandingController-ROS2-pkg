//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeGrad_StoreHx.cpp
//
// Code generation for function 'computeGrad_StoreHx'
//

// Include files
#include "computeGrad_StoreHx.h"
#include "PositionMPCStepFunction_internal_types.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cstring>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
void computeGrad_StoreHx(b_struct_T *obj, const double H[25600],
                         const double f[160], const double x[161])
{
  switch (obj->objtype) {
  case 5: {
    int i;
    i = obj->nvar;
    if (0 <= i - 2) {
      std::memset(&obj->grad[0], 0, (i + -1) * sizeof(double));
    }
    obj->grad[obj->nvar - 1] = obj->gammaScalar;
  } break;
  case 3: {
    int i;
    int lda;
    int m_tmp_tmp;
    m_tmp_tmp = obj->nvar - 1;
    lda = obj->nvar;
    if (obj->nvar != 0) {
      int ix;
      if (0 <= m_tmp_tmp) {
        std::memset(&obj->Hx[0], 0, (m_tmp_tmp + 1) * sizeof(double));
      }
      ix = 0;
      i = obj->nvar * (obj->nvar - 1) + 1;
      for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
        int i1;
        i1 = iac + m_tmp_tmp;
        for (int ia{iac}; ia <= i1; ia++) {
          int i2;
          i2 = ia - iac;
          obj->Hx[i2] += H[ia - 1] * x[ix];
        }
        ix++;
      }
    }
    i = obj->nvar;
    if (0 <= i - 1) {
      std::copy(&obj->Hx[0], &obj->Hx[i], &obj->grad[0]);
    }
    if (obj->hasLinear && (obj->nvar >= 1)) {
      i = obj->nvar - 1;
      for (m_tmp_tmp = 0; m_tmp_tmp <= i; m_tmp_tmp++) {
        obj->grad[m_tmp_tmp] += f[m_tmp_tmp];
      }
    }
  } break;
  default: {
    int i;
    int lda;
    int m_tmp_tmp;
    m_tmp_tmp = obj->nvar - 1;
    lda = obj->nvar;
    if (obj->nvar != 0) {
      int ix;
      if (0 <= m_tmp_tmp) {
        std::memset(&obj->Hx[0], 0, (m_tmp_tmp + 1) * sizeof(double));
      }
      ix = 0;
      i = obj->nvar * (obj->nvar - 1) + 1;
      for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
        int i1;
        i1 = iac + m_tmp_tmp;
        for (int ia{iac}; ia <= i1; ia++) {
          int i2;
          i2 = ia - iac;
          obj->Hx[i2] += H[ia - 1] * x[ix];
        }
        ix++;
      }
    }
    i = obj->nvar + 1;
    for (m_tmp_tmp = i; m_tmp_tmp < 161; m_tmp_tmp++) {
      obj->Hx[m_tmp_tmp - 1] = 0.0 * x[m_tmp_tmp - 1];
    }
    std::copy(&obj->Hx[0], &obj->Hx[160], &obj->grad[0]);
    if (obj->hasLinear && (obj->nvar >= 1)) {
      i = obj->nvar - 1;
      for (m_tmp_tmp = 0; m_tmp_tmp <= i; m_tmp_tmp++) {
        obj->grad[m_tmp_tmp] += f[m_tmp_tmp];
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
} // namespace pos_MPC

// End of code generation (computeGrad_StoreHx.cpp)
