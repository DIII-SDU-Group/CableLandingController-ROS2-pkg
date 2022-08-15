//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeFirstOrderOpt.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef COMPUTEFIRSTORDEROPT_H
#define COMPUTEFIRSTORDEROPT_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct b_struct_T;

struct c_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace parseoutput {
void computeFirstOrderOpt(b_struct_T *solution, const c_struct_T *objective,
                          int workingset_nVar, int workingset_ldA,
                          const ::coder::array<double, 1U> &workingset_ATwset,
                          int workingset_nActiveConstr,
                          ::coder::array<double, 2U> &workspace);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for computeFirstOrderOpt.h
//
// [EOF]
//
