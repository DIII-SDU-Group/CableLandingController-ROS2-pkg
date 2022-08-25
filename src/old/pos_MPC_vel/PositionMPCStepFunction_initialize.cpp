//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// PositionMPCStepFunction_initialize.cpp
//
// Code generation for function 'PositionMPCStepFunction_initialize'
//

// Include files
#include "PositionMPCStepFunction_initialize.h"
#include "PositionMPCStepFunction.h"
#include "PositionMPCStepFunction_data.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace pos_MPC {
void PositionMPCStepFunction_initialize()
{
  X_not_empty_init();
  UB_not_empty_init();
  LB_not_empty_init();
  Aeq_not_empty_init();
  bineq_not_empty_init();
  Aineq_not_empty_init();
  f_not_empty_init();
  H_not_empty_init();
  Q_last_not_empty_init();
  Q_not_empty_init();
  B_not_empty_init();
  PositionMPCStepFunction_init();
  isInitialized_PositionMPCStepFunction = true;
}

} // namespace pos_MPC

// End of code generation (PositionMPCStepFunction_initialize.cpp)