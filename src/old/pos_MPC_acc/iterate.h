//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// iterate.h
//
// Code generation for function 'iterate'
//

#ifndef ITERATE_H
#define ITERATE_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace pos_MPC {
struct struct_T;

struct e_struct_T;

struct g_struct_T;

struct f_struct_T;

struct c_struct_T;

struct b_struct_T;

} // namespace pos_MPC

// Function Declarations
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void iterate(const double H[14400], const double f[120], struct_T *solution,
             e_struct_T *memspace, g_struct_T *workingset,
             f_struct_T *qrmanager, c_struct_T *cholmanager,
             b_struct_T *objective, double options_ObjectiveLimit,
             double options_StepTolerance,
             double runTimeOptions_ConstrRelTolFactor,
             double runTimeOptions_ProbRelTolFactor,
             boolean_T runTimeOptions_RemainFeasible);

}
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

#endif
// End of code generation (iterate.h)
