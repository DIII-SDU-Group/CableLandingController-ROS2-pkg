//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: compressBounds.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "compressBounds.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : int nVar
//                ::coder::array<int, 1U> &indexLB
//                ::coder::array<int, 1U> &indexUB
//                ::coder::array<int, 1U> &indexFixed
//                const ::coder::array<double, 1U> &lb
//                const ::coder::array<double, 1U> &ub
//                int *mLB
//                int *mUB
//                int *mFixed
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
void compressBounds(int nVar, ::coder::array<int, 1U> &indexLB,
                    ::coder::array<int, 1U> &indexUB,
                    ::coder::array<int, 1U> &indexFixed,
                    const ::coder::array<double, 1U> &lb,
                    const ::coder::array<double, 1U> &ub, int *mLB, int *mUB,
                    int *mFixed)
{
  *mLB = 0;
  *mUB = 0;
  *mFixed = 0;
  if (ub.size(0) != 0) {
    if (lb.size(0) != 0) {
      for (int idx{0}; idx < nVar; idx++) {
        boolean_T guard1{false};
        guard1 = false;
        if ((!std::isinf(lb[idx])) && (!std::isnan(lb[idx]))) {
          if (std::abs(lb[idx] - ub[idx]) < 1.0E-8) {
            (*mFixed)++;
            indexFixed[*mFixed - 1] = idx + 1;
          } else {
            (*mLB)++;
            indexLB[*mLB - 1] = idx + 1;
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
        if (guard1 && ((!std::isinf(ub[idx])) && (!std::isnan(ub[idx])))) {
          (*mUB)++;
          indexUB[*mUB - 1] = idx + 1;
        }
      }
    } else {
      for (int idx{0}; idx < nVar; idx++) {
        if ((!std::isinf(ub[idx])) && (!std::isnan(ub[idx]))) {
          (*mUB)++;
          indexUB[*mUB - 1] = idx + 1;
        }
      }
    }
  } else if (lb.size(0) != 0) {
    for (int idx{0}; idx < nVar; idx++) {
      if ((!std::isinf(lb[idx])) && (!std::isnan(lb[idx]))) {
        (*mLB)++;
        indexLB[*mLB - 1] = idx + 1;
      }
    }
  }
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for compressBounds.cpp
//
// [EOF]
//
