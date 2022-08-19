//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// PositionMPCStepFunction.h
//
// Code generation for function 'PositionMPCStepFunction'
//

#ifndef POSITIONMPCSTEPFUNCTION_H
#define POSITIONMPCSTEPFUNCTION_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace pos_MPC {
void Aeq_not_empty_init();

void Aineq_not_empty_init();

void B_not_empty_init();

void H_not_empty_init();

void LB_not_empty_init();

extern void PositionMPCStepFunction(
    double x[8], const double target[4], double dt, double vx_max,
    double vy_max, double vz_max, double vpsi_max, double ax_max, double ay_max,
    double az_max, double apsi_max, double wx, double wy, double wz,
    double wpsi, double wvx, double wvy, double wvz, double wvpsi,
    double reset_target, double reset_trajectory, double reset_bounds,
    double reset_weights, double planned_traj[160]);

void PositionMPCStepFunction_init();

void Q_not_empty_init();

void UB_not_empty_init();

void X_not_empty_init();

void bineq_not_empty_init();

void f_not_empty_init();

} // namespace pos_MPC

#endif
// End of code generation (PositionMPCStepFunction.h)
