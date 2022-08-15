//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mpc_fun.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "mpc_fun.h"
#include "eye.h"
#include "mpc_fun_data.h"
#include "mpc_fun_initialize.h"
#include "mpc_fun_internal_types.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>
#include <string.h>

// Variable Definitions
static boolean_T N_not_empty;

static boolean_T N_variables_not_empty;

static double A[9];

static boolean_T B_not_empty;

static boolean_T Q_not_empty;

static boolean_T Aineq_not_empty;

static boolean_T bineq_not_empty;

static boolean_T Aeq_not_empty;

static boolean_T beq_not_empty;

static boolean_T LB_not_empty;

static boolean_T UB_not_empty;

static h_struct_T options;

static boolean_T Z_not_empty;

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void B_not_empty_init()
{
  B_not_empty = false;
}

//
// Arguments    : void
// Return Type  : void
//
void N_not_empty_init()
{
  N_not_empty = false;
}

//
// Arguments    : void
// Return Type  : void
//
void N_variables_not_empty_init()
{
  N_variables_not_empty = false;
}

//
// Arguments    : double x[6]
//                double dt
//                double hp
//                double vy_max
//                double vz_max
//                double vpsi_max
//                double ay_max
//                double az_max
//                double apsi_max
//                double approach_width
//                double approach_region_slope
//                double min_cable_height
//                double wy
//                double wz
//                double wpsi
//                double wvy
//                double wvz
//                double wvpsi
//                double we_approach_region
//                double we_ground
// Return Type  : void
//
void mpc_fun(double x[6], double dt, double hp, double vy_max, double vz_max,
             double vpsi_max, double ay_max, double az_max, double apsi_max,
             double approach_width, double approach_region_slope,
             double min_cable_height, double wy, double wz, double wpsi,
             double wvy, double wvz, double wvpsi, double we_approach_region,
             double we_ground)
{
  static coder::array<double, 2U> Aeq;
  static coder::array<double, 2U> Aineq;
  static coder::array<double, 2U> Q;
  static coder::array<double, 1U> LB;
  static coder::array<double, 1U> UB;
  static coder::array<double, 1U> X;
  static coder::array<double, 1U> Z;
  static coder::array<double, 1U> beq;
  static coder::array<double, 1U> bineq;
  static coder::array<double, 1U> slack;
  static double B[9];
  static double N;
  static double N_variables;
  static const signed char a[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
  coder::array<double, 1U> r;
  coder::array<double, 1U> r1;
  double b_Aeq_tmp[18];
  double Aeq_tmp[9];
  double dv[6];
  double dv1[3];
  double y[3];
  double Aineq_tmp_tmp;
  double Q_tmp_tmp;
  double d;
  double d1;
  double d2;
  double unnamed_idx_1;
  int Aineq_tmp;
  int b_i;
  int i;
  int i1;
  int i2;
  int loop_ub;
  if (!isInitialized_mpc_fun) {
    mpc_fun_initialize();
  }
  //  Params
  if (!N_not_empty) {
    N = hp / dt;
    N_not_empty = true;
  }
  if (!N_variables_not_empty) {
    N_variables = (6.0 * N + N) + N;
    N_variables_not_empty = true;
  }
  if (!B_not_empty) {
    for (i = 0; i < 9; i++) {
      B[i] = static_cast<double>(a[i]) * dt;
    }
    B_not_empty = true;
  }
  //  Optimization preparation
  //  Weights
  if (!Q_not_empty) {
    coder::eye(N_variables, Q);
    Q_not_empty = ((Q.size(0) != 0) && (Q.size(1) != 0));
    i = static_cast<int>(N);
    for (b_i = 0; b_i < i; b_i++) {
      Q_tmp_tmp = ((static_cast<double>(b_i) + 1.0) - 1.0) * 6.0;
      Q[(static_cast<int>(Q_tmp_tmp + 1.0) +
         Q.size(0) * (static_cast<int>(Q_tmp_tmp + 1.0) - 1)) -
        1] = wy;
      Q[(static_cast<int>(Q_tmp_tmp + 2.0) +
         Q.size(0) * (static_cast<int>(Q_tmp_tmp + 2.0) - 1)) -
        1] = wz;
      Q[(static_cast<int>(Q_tmp_tmp + 3.0) +
         Q.size(0) * (static_cast<int>(Q_tmp_tmp + 3.0) - 1)) -
        1] = wpsi;
      Q[(static_cast<int>(Q_tmp_tmp + 4.0) +
         Q.size(0) * (static_cast<int>(Q_tmp_tmp + 4.0) - 1)) -
        1] = wvy;
      Q[(static_cast<int>(Q_tmp_tmp + 5.0) +
         Q.size(0) * (static_cast<int>(Q_tmp_tmp + 5.0) - 1)) -
        1] = wvz;
      Q[(static_cast<int>(Q_tmp_tmp + 6.0) +
         Q.size(0) * (static_cast<int>(Q_tmp_tmp + 6.0) - 1)) -
        1] = wvpsi;
    }
    d = 6.0 * N + 1.0;
    d1 = 6.0 * N + N;
    i = static_cast<int>(d1 + (1.0 - d));
    for (b_i = 0; b_i < i; b_i++) {
      Q_tmp_tmp = d + static_cast<double>(b_i);
      Q[(static_cast<int>(Q_tmp_tmp) +
         Q.size(0) * (static_cast<int>(Q_tmp_tmp) - 1)) -
        1] = we_approach_region;
    }
    i = static_cast<int>(((6.0 * N + N) + N) + (1.0 - (d1 + 1.0)));
    for (b_i = 0; b_i < i; b_i++) {
      Q_tmp_tmp = (d1 + 1.0) + static_cast<double>(b_i);
      Q[(static_cast<int>(Q_tmp_tmp) +
         Q.size(0) * (static_cast<int>(Q_tmp_tmp) - 1)) -
        1] = we_ground;
    }
  }
  //  Linear inequality constraints:
  //  6N for min/max accelerations, 3N for approach region, N for ground
  if (!Aineq_not_empty) {
    d = 6.0 * N + 3.0 * N;
    i = static_cast<int>(d + N);
    Aineq.set_size(i, static_cast<int>(N_variables));
    loop_ub = i * static_cast<int>(N_variables);
    for (i = 0; i < loop_ub; i++) {
      Aineq[i] = 0.0;
    }
    Aineq_not_empty = ((Aineq.size(0) != 0) && (Aineq.size(1) != 0));
    i = static_cast<int>(N);
    for (b_i = 0; b_i < i; b_i++) {
      Q_tmp_tmp = ((static_cast<double>(b_i) + 1.0) - 1.0) * 6.0;
      Aineq[(static_cast<int>(Q_tmp_tmp + 1.0) +
             Aineq.size(0) * (static_cast<int>(Q_tmp_tmp + 4.0) - 1)) -
            1] = 1.0;
      Aineq[(static_cast<int>(Q_tmp_tmp + 2.0) +
             Aineq.size(0) * (static_cast<int>(Q_tmp_tmp + 4.0) - 1)) -
            1] = -1.0;
      Aineq[(static_cast<int>(Q_tmp_tmp + 3.0) +
             Aineq.size(0) * (static_cast<int>(Q_tmp_tmp + 5.0) - 1)) -
            1] = 1.0;
      Aineq[(static_cast<int>(Q_tmp_tmp + 4.0) +
             Aineq.size(0) * (static_cast<int>(Q_tmp_tmp + 5.0) - 1)) -
            1] = -1.0;
      Aineq[(static_cast<int>(Q_tmp_tmp + 5.0) +
             Aineq.size(0) * (static_cast<int>(Q_tmp_tmp + 6.0) - 1)) -
            1] = 1.0;
      Aineq[(static_cast<int>(Q_tmp_tmp + 6.0) +
             Aineq.size(0) * (static_cast<int>(Q_tmp_tmp + 6.0) - 1)) -
            1] = -1.0;
      if (b_i + 1U > 1U) {
        Aineq_tmp_tmp = ((static_cast<double>(b_i) + 1.0) - 2.0) * 6.0;
        Aineq[(static_cast<int>(Q_tmp_tmp + 1.0) +
               Aineq.size(0) * (static_cast<int>(Aineq_tmp_tmp + 4.0) - 1)) -
              1] = -1.0;
        Aineq[(static_cast<int>(Q_tmp_tmp + 2.0) +
               Aineq.size(0) * (static_cast<int>(Aineq_tmp_tmp + 4.0) - 1)) -
              1] = 1.0;
        Aineq[(static_cast<int>(Q_tmp_tmp + 3.0) +
               Aineq.size(0) * (static_cast<int>(Aineq_tmp_tmp + 5.0) - 1)) -
              1] = -1.0;
        Aineq[(static_cast<int>(Q_tmp_tmp + 4.0) +
               Aineq.size(0) * (static_cast<int>(Aineq_tmp_tmp + 5.0) - 1)) -
              1] = 1.0;
        Aineq[(static_cast<int>(Q_tmp_tmp + 5.0) +
               Aineq.size(0) * (static_cast<int>(Aineq_tmp_tmp + 6.0) - 1)) -
              1] = -1.0;
        Aineq[(static_cast<int>(Q_tmp_tmp + 6.0) +
               Aineq.size(0) * (static_cast<int>(Aineq_tmp_tmp + 6.0) - 1)) -
              1] = 1.0;
      }
    }
    for (b_i = 0; b_i < i; b_i++) {
      loop_ub =
          static_cast<int>(6.0 * N + (static_cast<double>(b_i) + 1.0)) - 1;
      Q_tmp_tmp = ((static_cast<double>(b_i) + 1.0) - 1.0) * 6.0;
      Aineq[loop_ub + Aineq.size(0) * (static_cast<int>(Q_tmp_tmp + 1.0) - 1)] =
          -approach_region_slope;
      Aineq[loop_ub + Aineq.size(0) * (static_cast<int>(Q_tmp_tmp + 2.0) - 1)] =
          1.0;
      Aineq[loop_ub + Aineq.size(0) * loop_ub] = 1.0;
      Aineq_tmp =
          static_cast<int>((6.0 * N + N) + (static_cast<double>(b_i) + 1.0)) -
          1;
      Aineq[Aineq_tmp + Aineq.size(0) * (static_cast<int>(Q_tmp_tmp + 1.0) -
                                         1)] = approach_region_slope;
      Aineq[Aineq_tmp +
            Aineq.size(0) * (static_cast<int>(Q_tmp_tmp + 2.0) - 1)] = 1.0;
      Aineq[Aineq_tmp + Aineq.size(0) * loop_ub] = 1.0;
      Aineq_tmp = static_cast<int>((6.0 * N + 2.0 * N) +
                                   (static_cast<double>(b_i) + 1.0)) -
                  1;
      Aineq[Aineq_tmp +
            Aineq.size(0) * (static_cast<int>(Q_tmp_tmp + 2.0) - 1)] = 1.0;
      Aineq[Aineq_tmp + Aineq.size(0) * loop_ub] = 1.0;
    }
    for (b_i = 0; b_i < i; b_i++) {
      loop_ub = static_cast<int>(d + (static_cast<double>(b_i) + 1.0)) - 1;
      Aineq[loop_ub +
            Aineq.size(0) *
                (static_cast<int>(
                     ((static_cast<double>(b_i) + 1.0) - 1.0) * 6.0 + 2.0) -
                 1)] = -1.0;
      Aineq[loop_ub + Aineq.size(0) *
                          (static_cast<int>((6.0 * N + N) +
                                            (static_cast<double>(b_i) + 1.0)) -
                           1)] = 1.0;
    }
  }
  if (!bineq_not_empty) {
    loop_ub = static_cast<int>((6.0 * N + 3.0 * N) + N);
    bineq.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
      bineq[i] = 0.0;
    }
    bineq_not_empty = (bineq.size(0) != 0);
    i = static_cast<int>(N);
    for (b_i = 0; b_i < i; b_i++) {
      if (b_i + 1U > 1U) {
        Q_tmp_tmp = ay_max * dt;
        Aineq_tmp_tmp = ((static_cast<double>(b_i) + 1.0) - 1.0) * 6.0;
        bineq[static_cast<int>(Aineq_tmp_tmp + 1.0) - 1] = Q_tmp_tmp;
        bineq[static_cast<int>(Aineq_tmp_tmp + 2.0) - 1] = Q_tmp_tmp;
        Q_tmp_tmp = az_max * dt;
        bineq[static_cast<int>(Aineq_tmp_tmp + 3.0) - 1] = Q_tmp_tmp;
        bineq[static_cast<int>(Aineq_tmp_tmp + 4.0) - 1] = Q_tmp_tmp;
        Q_tmp_tmp = apsi_max * dt;
        bineq[static_cast<int>(Aineq_tmp_tmp + 5.0) - 1] = Q_tmp_tmp;
        bineq[static_cast<int>(Aineq_tmp_tmp + 6.0) - 1] = Q_tmp_tmp;
      }
    }
    for (b_i = 0; b_i < i; b_i++) {
      Q_tmp_tmp = approach_region_slope * approach_width / 2.0;
      bineq[static_cast<int>(6.0 * N + (static_cast<double>(b_i) + 1.0)) - 1] =
          Q_tmp_tmp;
      bineq[static_cast<int>((6.0 * N + N) + (static_cast<double>(b_i) + 1.0)) -
            1] = Q_tmp_tmp;
    }
    for (b_i = 0; b_i < i; b_i++) {
      bineq[static_cast<int>((6.0 * N + 3.0 * N) +
                             (static_cast<double>(b_i) + 1.0)) -
            1] = min_cable_height;
    }
  }
  //  Linear equality constraints:
  //  3N for model constraints
  if (!Aeq_not_empty) {
    i = static_cast<int>(3.0 * N);
    Aeq.set_size(i, static_cast<int>(N_variables));
    loop_ub = i * static_cast<int>(N_variables);
    for (i = 0; i < loop_ub; i++) {
      Aeq[i] = 0.0;
    }
    Aeq_not_empty = ((Aeq.size(0) != 0) && (Aeq.size(1) != 0));
    coder::eye(Aeq_tmp);
    for (i = 0; i < 3; i++) {
      Aeq[Aeq.size(0) * i] = Aeq_tmp[3 * i];
      Aeq[Aeq.size(0) * (i + 3)] = 0.0;
      Aeq[Aeq.size(0) * i + 1] = Aeq_tmp[3 * i + 1];
      Aeq[Aeq.size(0) * (i + 3) + 1] = 0.0;
      Aeq[Aeq.size(0) * i + 2] = Aeq_tmp[3 * i + 2];
      Aeq[Aeq.size(0) * (i + 3) + 2] = 0.0;
    }
    i = static_cast<int>(N + -1.0);
    for (b_i = 0; b_i < i; b_i++) {
      int i3;
      int i4;
      int i5;
      d = ((static_cast<double>(b_i) + 2.0) - 1.0) * 3.0 + 1.0;
      d1 = (static_cast<double>(b_i) + 2.0) * 3.0;
      if (d > d1) {
        i1 = 0;
        i2 = 0;
      } else {
        i1 = static_cast<int>(d) - 1;
        i2 = static_cast<int>(d1);
      }
      d2 = ((static_cast<double>(b_i) + 2.0) - 1.0) * 6.0;
      Q_tmp_tmp = (static_cast<double>(b_i) + 2.0) * 6.0;
      if (d2 + 1.0 > Q_tmp_tmp) {
        i3 = 0;
        i4 = 0;
      } else {
        i3 = static_cast<int>(d2 + 1.0) - 1;
        i4 = static_cast<int>(Q_tmp_tmp);
      }
      for (i5 = 0; i5 < 3; i5++) {
        b_Aeq_tmp[3 * i5] = Aeq_tmp[3 * i5];
        Aineq_tmp = 3 * (i5 + 3);
        b_Aeq_tmp[Aineq_tmp] = 0.0;
        loop_ub = 3 * i5 + 1;
        b_Aeq_tmp[loop_ub] = Aeq_tmp[loop_ub];
        b_Aeq_tmp[Aineq_tmp + 1] = 0.0;
        loop_ub = 3 * i5 + 2;
        b_Aeq_tmp[loop_ub] = Aeq_tmp[loop_ub];
        b_Aeq_tmp[Aineq_tmp + 2] = 0.0;
      }
      Aineq_tmp = i2 - i1;
      loop_ub = i4 - i3;
      for (i2 = 0; i2 < loop_ub; i2++) {
        for (i4 = 0; i4 < Aineq_tmp; i4++) {
          Aeq[(i1 + i4) + Aeq.size(0) * (i3 + i2)] =
              b_Aeq_tmp[i4 + Aineq_tmp * i2];
        }
      }
      if (d > d1) {
        i1 = 0;
        i2 = 0;
      } else {
        i1 = static_cast<int>(d) - 1;
        i2 = static_cast<int>(d1);
      }
      d = ((static_cast<double>(b_i) + 2.0) - 2.0) * 6.0 + 1.0;
      if (d > d2) {
        i3 = 0;
        i4 = 0;
      } else {
        i3 = static_cast<int>(d) - 1;
        i4 = static_cast<int>(d2);
      }
      for (i5 = 0; i5 < 3; i5++) {
        b_Aeq_tmp[3 * i5] = -A[3 * i5];
        Aineq_tmp = 3 * (i5 + 3);
        b_Aeq_tmp[Aineq_tmp] = -B[3 * i5];
        loop_ub = 3 * i5 + 1;
        b_Aeq_tmp[loop_ub] = -A[loop_ub];
        b_Aeq_tmp[Aineq_tmp + 1] = -B[loop_ub];
        loop_ub = 3 * i5 + 2;
        b_Aeq_tmp[loop_ub] = -A[loop_ub];
        b_Aeq_tmp[Aineq_tmp + 2] = -B[loop_ub];
      }
      Aineq_tmp = i2 - i1;
      loop_ub = i4 - i3;
      for (i2 = 0; i2 < loop_ub; i2++) {
        for (i4 = 0; i4 < Aineq_tmp; i4++) {
          Aeq[(i1 + i4) + Aeq.size(0) * (i3 + i2)] =
              b_Aeq_tmp[i4 + Aineq_tmp * i2];
        }
      }
    }
  }
  if (!beq_not_empty) {
    loop_ub = static_cast<int>(3.0 * N);
    beq.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
      beq[i] = 0.0;
    }
    beq_not_empty = (beq.size(0) != 0);
  }
  //  Upper/lower bounds:
  if (!LB_not_empty) {
    loop_ub = static_cast<int>(N_variables);
    LB.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
      LB[i] = 0.0;
    }
    LB_not_empty = (LB.size(0) != 0);
    i = static_cast<int>(N);
    if (0 <= static_cast<int>(N) - 1) {
      dv[0] = rtMinusInf;
      dv[1] = rtMinusInf;
      dv[2] = rtMinusInf;
      dv[3] = -vy_max;
      dv[4] = -vz_max;
      dv[5] = -vpsi_max;
    }
    for (b_i = 0; b_i < i; b_i++) {
      d = ((static_cast<double>(b_i) + 1.0) - 1.0) * 6.0 + 1.0;
      d1 = (static_cast<double>(b_i) + 1.0) * 6.0;
      if (d > d1) {
        i1 = 0;
        i2 = 0;
      } else {
        i1 = static_cast<int>(d) - 1;
        i2 = static_cast<int>(d1);
      }
      loop_ub = i2 - i1;
      for (i2 = 0; i2 < loop_ub; i2++) {
        LB[i1 + i2] = dv[i2];
      }
    }
    d = 6.0 * N + 1.0;
    i = static_cast<int>((6.0 * N + 2.0 * N) + (1.0 - d));
    for (b_i = 0; b_i < i; b_i++) {
      LB[static_cast<int>(d + static_cast<double>(b_i)) - 1] = rtMinusInf;
    }
  }
  if (!UB_not_empty) {
    loop_ub = static_cast<int>(N_variables);
    UB.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
      UB[i] = 0.0;
    }
    UB_not_empty = (UB.size(0) != 0);
    i = static_cast<int>(N);
    if (0 <= static_cast<int>(N) - 1) {
      dv[0] = rtInf;
      dv[1] = rtInf;
      dv[2] = rtInf;
      dv[3] = vy_max;
      dv[4] = vz_max;
      dv[5] = vpsi_max;
    }
    for (b_i = 0; b_i < i; b_i++) {
      d = ((static_cast<double>(b_i) + 1.0) - 1.0) * 6.0 + 1.0;
      d1 = (static_cast<double>(b_i) + 1.0) * 6.0;
      if (d > d1) {
        i1 = 0;
        i2 = 0;
      } else {
        i1 = static_cast<int>(d) - 1;
        i2 = static_cast<int>(d1);
      }
      loop_ub = i2 - i1;
      for (i2 = 0; i2 < loop_ub; i2++) {
        UB[i1 + i2] = dv[i2];
      }
    }
    d = 6.0 * N + 1.0;
    i = static_cast<int>((6.0 * N + 2.0 * N) + (1.0 - d));
    for (b_i = 0; b_i < i; b_i++) {
      UB[static_cast<int>(d + static_cast<double>(b_i)) - 1] = rtInf;
    }
  }
  //  Options
  //  hej
  if (!Z_not_empty) {
    loop_ub = static_cast<int>(6.0 * N);
    X.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
      X[i] = 0.0;
    }
    for (i = 0; i < 6; i++) {
      X[i] = x[i];
    }
    i = static_cast<int>(N + -1.0);
    for (b_i = 0; b_i < i; b_i++) {
      double unnamed_idx_2;
      Q_tmp_tmp = ((static_cast<double>(b_i) + 2.0) - 2.0) * 6.0;
      y[0] = X[static_cast<int>(Q_tmp_tmp + 1.0) - 1];
      Aineq_tmp_tmp = X[static_cast<int>(Q_tmp_tmp + 4.0) - 1];
      y[1] = X[static_cast<int>(Q_tmp_tmp + 2.0) - 1];
      unnamed_idx_1 = X[static_cast<int>(Q_tmp_tmp + 5.0) - 1];
      y[2] = X[static_cast<int>(Q_tmp_tmp + 3.0) - 1];
      unnamed_idx_2 = X[static_cast<int>(Q_tmp_tmp + 6.0) - 1];
      Q_tmp_tmp = ((static_cast<double>(b_i) + 2.0) - 1.0) * 6.0;
      d = y[0];
      d1 = y[1];
      d2 = y[2];
      for (i1 = 0; i1 < 3; i1++) {
        X[static_cast<int>(Q_tmp_tmp + (static_cast<double>(i1) + 1.0)) - 1] =
            ((A[i1] * d + A[i1 + 3] * d1) + A[i1 + 6] * d2) +
            ((B[i1] * Aineq_tmp_tmp + B[i1 + 3] * unnamed_idx_1) +
             B[i1 + 6] * unnamed_idx_2);
      }
      unnamed_idx_1 = X[4];
      unnamed_idx_2 = X[5];
      X[static_cast<int>(Q_tmp_tmp + 4.0) - 1] = X[3];
      X[static_cast<int>(Q_tmp_tmp + 5.0) - 1] = unnamed_idx_1;
      X[static_cast<int>(Q_tmp_tmp + 6.0) - 1] = unnamed_idx_2;
    }
    loop_ub = static_cast<int>(2.0 * N);
    slack.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
      slack[i] = 0.0;
    }
    Z.set_size(X.size(0) + slack.size(0));
    loop_ub = X.size(0);
    for (i = 0; i < loop_ub; i++) {
      Z[i] = X[i];
    }
    loop_ub = slack.size(0);
    for (i = 0; i < loop_ub; i++) {
      Z[i + X.size(0)] = slack[i];
    }
  }
  for (i = 0; i < 3; i++) {
    b_Aeq_tmp[3 * i] = A[3 * i];
    Aineq_tmp = 3 * (i + 3);
    b_Aeq_tmp[Aineq_tmp] = B[3 * i];
    loop_ub = 3 * i + 1;
    b_Aeq_tmp[loop_ub] = A[loop_ub];
    b_Aeq_tmp[Aineq_tmp + 1] = B[loop_ub];
    loop_ub = 3 * i + 2;
    b_Aeq_tmp[loop_ub] = A[loop_ub];
    b_Aeq_tmp[Aineq_tmp + 2] = B[loop_ub];
  }
  for (i = 0; i < 3; i++) {
    d = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      d += b_Aeq_tmp[i + 3 * i1] * x[i1];
    }
    beq[i] = d;
  }
  Q_tmp_tmp = ay_max * dt;
  bineq[0] = Q_tmp_tmp + x[3];
  bineq[1] = Q_tmp_tmp - x[3];
  Q_tmp_tmp = az_max * dt;
  bineq[2] = Q_tmp_tmp + x[4];
  bineq[3] = Q_tmp_tmp - x[4];
  Q_tmp_tmp = apsi_max * dt;
  bineq[4] = Q_tmp_tmp + x[5];
  bineq[5] = Q_tmp_tmp - x[5];
  //      Z = fmincon(fun, Z, Aineq, bineq, Aeq, beq, LB, UB, [], options);
  loop_ub = static_cast<int>(N_variables);
  r.set_size(loop_ub);
  for (i = 0; i < loop_ub; i++) {
    r[i] = 0.0;
  }
  r1.set_size(Z.size(0));
  loop_ub = Z.size(0) - 1;
  for (i = 0; i <= loop_ub; i++) {
    r1[i] = Z[i];
  }
  coder::quadprog(Q, r, Aineq, bineq, Aeq, beq, LB, UB, r1, &options, Z);
  Z_not_empty = true;
  d = 6.0 * N;
  if (1.0 > d) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(d);
  }
  X.set_size(loop_ub);
  for (i = 0; i < loop_ub; i++) {
    X[i] = Z[i];
  }
  d = 6.0 * N + 1.0;
  if (d > Z.size(0)) {
    i = 0;
    i1 = 0;
  } else {
    i = static_cast<int>(d) - 1;
    i1 = Z.size(0);
  }
  loop_ub = i1 - i;
  slack.set_size(loop_ub);
  for (i1 = 0; i1 < loop_ub; i1++) {
    slack[i1] = Z[i + i1];
  }
  for (i = 0; i < 6; i++) {
    x[i] = X[i];
  }
  d = (N - 1.0) * 6.0;
  if (1.0 > d) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(d);
  }
  y[0] = X[static_cast<int>(d + 1.0) - 1];
  Q_tmp_tmp = X[static_cast<int>(d + 4.0) - 1];
  y[1] = X[static_cast<int>(d + 2.0) - 1];
  Aineq_tmp_tmp = X[static_cast<int>(d + 5.0) - 1];
  y[2] = X[static_cast<int>(d + 3.0) - 1];
  unnamed_idx_1 = X[static_cast<int>(d + 6.0) - 1];
  d = y[0];
  d1 = y[1];
  d2 = y[2];
  for (i = 0; i < 3; i++) {
    dv1[i] = (A[i] * d + A[i + 3] * d1) + A[i + 6] * d2;
  }
  for (i = 0; i < 3; i++) {
    y[i] = (B[i] * Q_tmp_tmp + B[i + 3] * Aineq_tmp_tmp) +
           B[i + 6] * unnamed_idx_1;
  }
  r.set_size(loop_ub + 6);
  for (i = 0; i < loop_ub; i++) {
    r[i] = X[i];
  }
  r[loop_ub] = dv1[0] + y[0];
  r[loop_ub + 3] = Q_tmp_tmp;
  r[loop_ub + 1] = dv1[1] + y[1];
  r[loop_ub + 4] = Aineq_tmp_tmp;
  r[loop_ub + 2] = dv1[2] + y[2];
  r[loop_ub + 5] = unnamed_idx_1;
  X.set_size(r.size(0));
  loop_ub = r.size(0);
  for (i = 0; i < loop_ub; i++) {
    X[i] = r[i];
  }
  d = (N_variables - 6.0 * N) - 1.0;
  if (1.0 > d) {
    i = 0;
  } else {
    i = static_cast<int>(d);
  }
  slack.set_size(i + 1);
  slack[i] = 0.0;
  Z.set_size(X.size(0) + slack.size(0));
  loop_ub = X.size(0);
  for (i = 0; i < loop_ub; i++) {
    Z[i] = X[i];
  }
  loop_ub = slack.size(0);
  for (i = 0; i < loop_ub; i++) {
    Z[i + X.size(0)] = slack[i];
  }
}

//
// Arguments    : void
// Return Type  : void
//
void mpc_fun_init()
{
  static const char t0_Algorithm[10]{'a', 'c', 't', 'i', 'v',
                                     'e', '-', 's', 'e', 't'};
  static const char t0_SolverName[8]{'q', 'u', 'a', 'd', 'p', 'r', 'o', 'g'};
  static const char t0_FiniteDifferenceType[7]{'f', 'o', 'r', 'w',
                                               'a', 'r', 'd'};
  static const char t0_Display[5]{'f', 'i', 'n', 'a', 'l'};
  int i;
  Z_not_empty = false;
  UB_not_empty = false;
  LB_not_empty = false;
  beq_not_empty = false;
  Aeq_not_empty = false;
  bineq_not_empty = false;
  Aineq_not_empty = false;
  Q_not_empty = false;
  std::memset(&A[0], 0, 9U * sizeof(double));
  A[0] = 1.0;
  A[4] = 1.0;
  A[8] = 1.0;
  options.InitDamping = 0.01;
  for (i = 0; i < 7; i++) {
    options.FiniteDifferenceType[i] = t0_FiniteDifferenceType[i];
  }
  options.SpecifyObjectiveGradient = false;
  options.ScaleProblem = false;
  options.SpecifyConstraintGradient = false;
  options.NonFiniteSupport = true;
  options.IterDisplaySQP = false;
  options.FiniteDifferenceStepSize = -1.0;
  options.MaxFunctionEvaluations = -1.0;
  options.IterDisplayQP = false;
  options.PricingTolerance = 0.0;
  for (i = 0; i < 10; i++) {
    options.Algorithm[i] = t0_Algorithm[i];
  }
  options.ObjectiveLimit = -1.0E+20;
  options.ConstraintTolerance = 1.0E-8;
  options.OptimalityTolerance = 0.999;
  options.StepTolerance = 0.001;
  options.MaxIterations = -1.0;
  options.FunctionTolerance = rtInf;
  for (i = 0; i < 8; i++) {
    options.SolverName[i] = t0_SolverName[i];
  }
  options.CheckGradients = false;
  options.Diagnostics[0] = 'o';
  options.Diagnostics[1] = 'f';
  options.Diagnostics[2] = 'f';
  options.DiffMaxChange = rtInf;
  options.DiffMinChange = 0.0;
  for (i = 0; i < 5; i++) {
    options.Display[i] = t0_Display[i];
  }
  options.FunValCheck[0] = 'o';
  options.FunValCheck[1] = 'f';
  options.FunValCheck[2] = 'f';
  options.UseParallel = false;
  options.LinearSolver[0] = 'a';
  options.LinearSolver[1] = 'u';
  options.LinearSolver[2] = 't';
  options.LinearSolver[3] = 'o';
  options.SubproblemAlgorithm[0] = 'c';
  options.SubproblemAlgorithm[1] = 'g';
  //          options = optimoptions('fmincon', 'Algorithm', 'sqp',
  //          'OptimalityTolerance', 0.999, 'StepTolerance', 0.001, ...
  //              'SpecifyObjectiveGradient', true, 'SpecifyConstraintGradient',
  //              false);
}

//
// File trailer for mpc_fun.cpp
//
// [EOF]
//
