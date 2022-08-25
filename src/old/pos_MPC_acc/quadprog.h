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
void quadprog(const double H[14400], const double f[120],
              const double Aeq[9600], const double b_beq[80],
              const double lb[120], const double ub[120], const double x0[120],
              double x[120]);

}
} // namespace pos_MPC

#endif
// End of code generation (quadprog.h)
