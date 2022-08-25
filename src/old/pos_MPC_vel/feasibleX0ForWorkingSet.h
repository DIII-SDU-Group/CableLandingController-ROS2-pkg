//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// feasibleX0ForWorkingSet.h
//
// Code generation for function 'feasibleX0ForWorkingSet'
//

#ifndef FEASIBLEX0FORWORKINGSET_H
#define FEASIBLEX0FORWORKINGSET_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace pos_MPC {
struct d_struct_T;

struct g_struct_T;

} // namespace pos_MPC

// Function Declarations
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
boolean_T feasibleX0ForWorkingSet(double workspace[90321], double xCurrent[161],
                                  d_struct_T *workingset,
                                  g_struct_T *qrmanager);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (feasibleX0ForWorkingSet.h)
