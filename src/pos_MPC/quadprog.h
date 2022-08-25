//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// quadprog.h
//
// Code generation for function 'quadprog'
//

#ifndef QUADPROG_H
#define QUADPROG_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace pos_MPC {
namespace coder {
void quadprog(const double H[32400], const double f[180],
              const double Aeq[21600], const double b_beq[120],
              const double lb[180], const double ub[180], const double x0[180],
              double x[180]);

}
} // namespace pos_MPC

#endif
// End of code generation (quadprog.h)
