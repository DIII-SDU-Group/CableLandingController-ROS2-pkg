//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// checkStoppingAndUpdateFval.h
//
// Code generation for function 'checkStoppingAndUpdateFval'
//

#ifndef CHECKSTOPPINGANDUPDATEFVAL_H
#define CHECKSTOPPINGANDUPDATEFVAL_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace pos_MPC {
struct struct_T;

struct f_struct_T;

struct b_struct_T;

struct d_struct_T;

struct g_struct_T;

} // namespace pos_MPC

// Function Declarations
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace stopping {
void checkStoppingAndUpdateFval(int *activeSetChangeID, const double f[160],
                                struct_T *solution, f_struct_T *memspace,
                                const b_struct_T *objective,
                                d_struct_T *workingset, g_struct_T *qrmanager,
                                double options_ObjectiveLimit,
                                double runTimeOptions_ConstrRelTolFactor,
                                boolean_T updateFval);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (checkStoppingAndUpdateFval.h)
