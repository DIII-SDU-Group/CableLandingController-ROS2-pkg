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
void quadprog(const double H[25600], const double f[160],
              const double Aineq[25600], const double bineq[160],
              const double Aeq[12800], const double b_beq[80],
              const double lb[160], const double ub[160], const double x0[160],
              double x[160]);

}
} // namespace pos_MPC

#endif
// End of code generation (quadprog.h)
