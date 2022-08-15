//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mpc_fun.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef MPC_FUN_H
#define MPC_FUN_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void B_not_empty_init();

void N_not_empty_init();

void N_variables_not_empty_init();

extern void mpc_fun(double x[6], double dt, double hp, double vy_max,
                    double vz_max, double vpsi_max, double ay_max,
                    double az_max, double apsi_max, double approach_width,
                    double approach_region_slope, double min_cable_height,
                    double wy, double wz, double wpsi, double wvy, double wvz,
                    double wvpsi, double we_approach_region, double we_ground);

void mpc_fun_init();

#endif
//
// File trailer for mpc_fun.h
//
// [EOF]
//
