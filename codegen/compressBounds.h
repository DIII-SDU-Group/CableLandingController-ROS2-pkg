//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: compressBounds.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef COMPRESSBOUNDS_H
#define COMPRESSBOUNDS_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
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
                    int *mFixed);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for compressBounds.h
//
// [EOF]
//
