//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// fullColLDL2_.h
//
// Code generation for function 'fullColLDL2_'
//

#ifndef FULLCOLLDL2__H
#define FULLCOLLDL2__H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace pos_MPC {
struct c_struct_T;

}

// Function Declarations
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace DynamicRegCholManager {
void fullColLDL2_(c_struct_T *obj, int LD_offset, int NColsRemain,
                  double REG_PRIMAL);

}
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (fullColLDL2_.h)
