//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzgeqp3.h
//
// Code generation for function 'xzgeqp3'
//

#ifndef XZGEQP3_H
#define XZGEQP3_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace pos_MPC {
namespace coder {
namespace internal {
namespace reflapack {
void qrf(double A[144300], int m, int n, int nfxd, double tau[300]);

void qrpf(double A[144300], int m, int n, int nfxd, double tau[300],
          int jpvt[481]);

} // namespace reflapack
} // namespace internal
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (xzgeqp3.h)
