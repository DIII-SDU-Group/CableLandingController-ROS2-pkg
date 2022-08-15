//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: quadprog.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef QUADPROG_H
#define QUADPROG_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct h_struct_T;

// Function Declarations
namespace coder {
void quadprog(const ::coder::array<double, 2U> &H,
              const ::coder::array<double, 1U> &f,
              const ::coder::array<double, 2U> &Aineq,
              const ::coder::array<double, 1U> &bineq,
              const ::coder::array<double, 2U> &Aeq,
              const ::coder::array<double, 1U> &beq,
              const ::coder::array<double, 1U> &lb,
              const ::coder::array<double, 1U> &ub,
              const ::coder::array<double, 1U> &x0, const h_struct_T *b_options,
              ::coder::array<double, 1U> &x);

}

#endif
//
// File trailer for quadprog.h
//
// [EOF]
//
