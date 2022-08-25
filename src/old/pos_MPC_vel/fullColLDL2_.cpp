//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// fullColLDL2_.cpp
//
// Code generation for function 'fullColLDL2_'
//

// Include files
#include "fullColLDL2_.h"
#include "PositionMPCStepFunction_internal_types.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace DynamicRegCholManager {
void fullColLDL2_(c_struct_T *obj, int LD_offset, int NColsRemain,
                  double REG_PRIMAL)
{
  int jA;
  for (int k{0}; k < NColsRemain; k++) {
    double alpha1;
    int LD_diagOffset;
    int i;
    int subMatrixDim;
    LD_diagOffset = (LD_offset + 241 * k) - 1;
    if (std::abs(obj->FMat[LD_diagOffset]) <= obj->regTol_) {
      obj->FMat[LD_diagOffset] += REG_PRIMAL;
    }
    alpha1 = -1.0 / obj->FMat[LD_diagOffset];
    subMatrixDim = (NColsRemain - k) - 2;
    for (jA = 0; jA <= subMatrixDim; jA++) {
      obj->workspace_[jA] = obj->FMat[(LD_diagOffset + jA) + 1];
    }
    if (!(alpha1 == 0.0)) {
      jA = LD_diagOffset;
      for (int j{0}; j <= subMatrixDim; j++) {
        if (obj->workspace_[j] != 0.0) {
          double temp;
          int i1;
          temp = obj->workspace_[j] * alpha1;
          i = jA + 242;
          i1 = subMatrixDim + jA;
          for (int ijA{i}; ijA <= i1 + 242; ijA++) {
            obj->FMat[ijA - 1] += obj->workspace_[(ijA - jA) - 242] * temp;
          }
        }
        jA += 240;
      }
    }
    for (jA = 0; jA <= subMatrixDim; jA++) {
      i = (LD_diagOffset + jA) + 1;
      obj->FMat[i] /= obj->FMat[LD_diagOffset];
    }
  }
  jA = (LD_offset + 241 * (NColsRemain - 1)) - 1;
  if (std::abs(obj->FMat[jA]) <= obj->regTol_) {
    obj->FMat[jA] += REG_PRIMAL;
  }
}

} // namespace DynamicRegCholManager
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (fullColLDL2_.cpp)
