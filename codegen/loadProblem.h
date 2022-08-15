//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: loadProblem.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef LOADPROBLEM_H
#define LOADPROBLEM_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct f_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void loadProblem(f_struct_T *obj, int mIneq, int mLinIneq,
                 const ::coder::array<double, 2U> &Aineq,
                 const ::coder::array<double, 1U> &bineq, int mEq, int mLinEq,
                 const ::coder::array<double, 2U> &Aeq,
                 const ::coder::array<double, 1U> &beq, int mLB,
                 const ::coder::array<double, 1U> &lb, int mUB,
                 const ::coder::array<double, 1U> &ub, int mFixed,
                 int mConstrMax);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for loadProblem.h
//
// [EOF]
//
