//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// MPCStepFunction_initialize.cpp
//
// Code generation for function 'MPCStepFunction_initialize'
//

// Include files
#include "MPCStepFunction_initialize.h"
#include "MPCStepFunction.h"
#include "MPCStepFunction_data.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace pos_MPC {
void MPCStepFunction_initialize()
{
  X_not_empty_init();
  UB_not_empty_init();
  LB_not_empty_init();
  Aeq_not_empty_init();
  f_not_empty_init();
  H_not_empty_init();
  Q_last_not_empty_init();
  Q_not_empty_init();
  B_not_empty_init();
  A_not_empty_init();
  MPCStepFunction_init();
  isInitialized_MPCStepFunction = true;
}

} // namespace pos_MPC

// End of code generation (MPCStepFunction_initialize.cpp)
