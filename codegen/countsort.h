//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: countsort.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef COUNTSORT_H
#define COUNTSORT_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace utils {
void countsort(::coder::array<int, 1U> &x, int xLen,
               ::coder::array<int, 1U> &workspace, int xMin, int xMax);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for countsort.h
//
// [EOF]
//