//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeFirstOrderOpt.h
//
// Code generation for function 'computeFirstOrderOpt'
//

#ifndef COMPUTEFIRSTORDEROPT_H
#define COMPUTEFIRSTORDEROPT_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace pos_MPC {
struct struct_T;

struct b_struct_T;

} // namespace pos_MPC

// Function Declarations
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace parseoutput {
void computeFirstOrderOpt(struct_T *solution, const b_struct_T *objective,
                          int workingset_nVar,
                          const double workingset_ATwset[87061],
                          int workingset_nActiveConstr,
                          double workspace[87061]);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (computeFirstOrderOpt.h)
