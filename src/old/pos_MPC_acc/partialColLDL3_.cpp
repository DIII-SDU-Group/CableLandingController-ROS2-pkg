//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// partialColLDL3_.cpp
//
// Code generation for function 'partialColLDL3_'
//

// Include files
#include "partialColLDL3_.h"
#include "MPCStepFunction_internal_types.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace DynamicRegCholManager {
void partialColLDL3_(c_struct_T *obj, int LD_offset, int NColsRemain,
                     double REG_PRIMAL)
{
  int LD_diagOffset;
  int W_diagOffset;
  int i;
  int i1;
  int i2;
  int i3;
  int ia;
  int idx;
  int ix;
  int k;
  int offsetColK;
  int subRows;
  i = NColsRemain - 1;
  for (k = 0; k < 48; k++) {
    subRows = (NColsRemain - k) - 1;
    LD_diagOffset = (LD_offset + 201 * k) - 1;
    W_diagOffset = 201 * k;
    for (idx = 0; idx <= subRows; idx++) {
      obj->workspace_[W_diagOffset + idx] = obj->FMat[LD_diagOffset + idx];
    }
    offsetColK = 200 * k;
    for (idx = 0; idx <= i; idx++) {
      obj->workspace2_[idx] = obj->workspace_[offsetColK + idx];
    }
    if ((NColsRemain != 0) && (k != 0)) {
      ix = LD_offset + k;
      i1 = 200 * (k - 1) + 1;
      for (idx = 1; idx <= i1; idx += 200) {
        i2 = (idx + NColsRemain) - 1;
        for (ia = idx; ia <= i2; ia++) {
          i3 = ia - idx;
          obj->workspace2_[i3] += obj->workspace_[ia - 1] * -obj->FMat[ix - 1];
        }
        ix += 200;
      }
    }
    for (idx = 0; idx <= i; idx++) {
      obj->workspace_[offsetColK + idx] = obj->workspace2_[idx];
    }
    for (idx = 0; idx <= subRows; idx++) {
      obj->FMat[LD_diagOffset + idx] = obj->workspace_[W_diagOffset + idx];
    }
    if (std::abs(obj->FMat[LD_diagOffset]) <= obj->regTol_) {
      obj->FMat[LD_diagOffset] += REG_PRIMAL;
    }
    for (idx = 0; idx < subRows; idx++) {
      i1 = (LD_diagOffset + idx) + 1;
      obj->FMat[i1] /= obj->FMat[LD_diagOffset];
    }
  }
  for (int j{48}; j <= i; j += 48) {
    int ia0;
    int m;
    int subBlockSize;
    W_diagOffset = NColsRemain - j;
    if (48 < W_diagOffset) {
      subBlockSize = 48;
    } else {
      subBlockSize = W_diagOffset;
    }
    ia0 = j + subBlockSize;
    i1 = ia0 - 1;
    for (k = j; k <= i1; k++) {
      m = ia0 - k;
      offsetColK = (LD_offset + 201 * k) - 1;
      for (idx = 0; idx < 48; idx++) {
        obj->workspace2_[idx] = obj->FMat[((LD_offset + k) + idx * 200) - 1];
      }
      subRows = k + 1;
      if (m != 0) {
        ix = 0;
        i2 = k + 9401;
        for (idx = subRows; idx <= i2; idx += 200) {
          i3 = (idx + m) - 1;
          for (ia = idx; ia <= i3; ia++) {
            LD_diagOffset = (offsetColK + ia) - idx;
            obj->FMat[LD_diagOffset] +=
                obj->workspace_[ia - 1] * -obj->workspace2_[ix];
          }
          ix++;
        }
      }
    }
    if (ia0 < NColsRemain) {
      m = W_diagOffset - subBlockSize;
      ia = ((LD_offset + subBlockSize) + 201 * j) - 1;
      i1 = subBlockSize - 1;
      for (idx = 0; idx < 48; idx++) {
        subRows = (LD_offset + j) + idx * 200;
        LD_diagOffset = idx * 200;
        for (W_diagOffset = 0; W_diagOffset <= i1; W_diagOffset++) {
          obj->workspace2_[LD_diagOffset + W_diagOffset] =
              obj->FMat[(subRows + W_diagOffset) - 1];
        }
      }
      if ((m != 0) && (subBlockSize != 0)) {
        subRows = ia + 200 * (subBlockSize - 1);
        LD_diagOffset = 0;
        for (offsetColK = ia; offsetColK <= subRows; offsetColK += 200) {
          W_diagOffset = ia0 - 1;
          LD_diagOffset++;
          i1 = LD_diagOffset + 9400;
          for (idx = LD_diagOffset; idx <= i1; idx += 200) {
            i2 = offsetColK + 1;
            i3 = offsetColK + m;
            for (ix = i2; ix <= i3; ix++) {
              obj->FMat[ix - 1] +=
                  -obj->workspace2_[idx - 1] *
                  obj->workspace_[(W_diagOffset + ix) - offsetColK];
            }
            W_diagOffset += 200;
          }
        }
      }
    }
  }
}

} // namespace DynamicRegCholManager
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (partialColLDL3_.cpp)
