//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// MPCStepFunction.h
//
// Code generation for function 'MPCStepFunction'
//

#ifndef MPCSTEPFUNCTION_H
#define MPCSTEPFUNCTION_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace pos_MPC {
void A_not_empty_init();

void Aeq_not_empty_init();

void B_not_empty_init();

void H_not_empty_init();

void LB_not_empty_init();

extern void MPCStepFunction(
    double x[12], const double target[4], double dt, double vx_max,
    double vy_max, double vz_max, double vpsi_max, double ax_max, double ay_max,
    double az_max, double apsi_max, double wx, double wy, double wz,
    double wpsi, double wvx, double wvy, double wvz, double wvpsi, double wax,
    double way, double waz, double wapsi, double wx_last, double wy_last,
    double wz_last, double wpsi_last, double wvx_last, double wvy_last,
    double wvz_last, double wvpsi_last, double wax_last, double way_last,
    double waz_last, double wapsi_last, double reset_target,
    double reset_trajectory, double reset_bounds, double reset_weights,
    double planned_traj[120]);

void MPCStepFunction_init();

void Q_last_not_empty_init();

void Q_not_empty_init();

void UB_not_empty_init();

void X_not_empty_init();

void f_not_empty_init();

} // namespace pos_MPC

#endif
// End of code generation (MPCStepFunction.h)
