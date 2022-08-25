//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// eye.cpp
//
// Code generation for function 'eye'
//

// Include files
#include "eye.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace pos_MPC {
namespace coder {
void b_eye(double b_I[64])
{
  std::memset(&b_I[0], 0, 64U * sizeof(double));
  for (int k{0}; k < 8; k++) {
    b_I[k + (k << 3)] = 1.0;
  }
}

void eye(double b_I[16])
{
  std::memset(&b_I[0], 0, 16U * sizeof(double));
  b_I[0] = 1.0;
  b_I[5] = 1.0;
  b_I[10] = 1.0;
  b_I[15] = 1.0;
}

} // namespace coder
} // namespace pos_MPC

// End of code generation (eye.cpp)
