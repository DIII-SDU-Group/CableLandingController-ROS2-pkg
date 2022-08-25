//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// MPCStepFunction.cpp
//
// Code generation for function 'MPCStepFunction'
//

// Include files
#include "MPCStepFunction.h"
#include "MPCStepFunction_data.h"
#include "MPCStepFunction_initialize.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cstring>

// Variable Definitions
namespace pos_MPC {
static double A[64];

static boolean_T A_not_empty;

static double B[32];

static boolean_T B_not_empty;

static boolean_T Q_not_empty;

static boolean_T Q_last_not_empty;

static boolean_T H_not_empty;

static boolean_T f_not_empty;

static boolean_T Aeq_not_empty;

static double beq[80];

static boolean_T LB_not_empty;

static boolean_T UB_not_empty;

static double X[120];

static boolean_T X_not_empty;

} // namespace pos_MPC

// Function Definitions
namespace pos_MPC {
void A_not_empty_init()
{
  A_not_empty = false;
}

void Aeq_not_empty_init()
{
  Aeq_not_empty = false;
}

void B_not_empty_init()
{
  B_not_empty = false;
}

void H_not_empty_init()
{
  H_not_empty = false;
}

void LB_not_empty_init()
{
  LB_not_empty = false;
}

void MPCStepFunction(double x[12], const double target[4], double dt,
                     double vx_max, double vy_max, double vz_max,
                     double vpsi_max, double ax_max, double ay_max,
                     double az_max, double apsi_max, double wx, double wy,
                     double wz, double wpsi, double wvx, double wvy, double wvz,
                     double wvpsi, double wax, double way, double waz,
                     double wapsi, double wx_last, double wy_last,
                     double wz_last, double wpsi_last, double wvx_last,
                     double wvy_last, double wvz_last, double wvpsi_last,
                     double wax_last, double way_last, double waz_last,
                     double wapsi_last, double reset_target,
                     double reset_trajectory, double reset_bounds,
                     double reset_weights, double planned_traj[120])
{
  static double H[14400];
  static double Aeq[9600];
  static double Q[144];
  static double Q_last[144];
  static double LB[120];
  static double UB[120];
  static double f[120];
  static const signed char b[16]{1, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 1};
  double dv1[144];
  double dv4[120];
  double dv3[96];
  double dv[12];
  double dv2[12];
  double dv5[8];
  double y[8];
  double a;
  int b_i;
  int i;
  int i1;
  int i3;
  int i4;
  int k;
  int unnamed_idx_1;
  signed char c_I[96];
  signed char b_I[64];
  if (!isInitialized_MPCStepFunction) {
    MPCStepFunction_initialize();
  }
  //  Params
  if (!A_not_empty) {
    std::memset(&A[0], 0, 64U * sizeof(double));
    for (k = 0; k < 8; k++) {
      A[k + (k << 3)] = 1.0;
    }
    A_not_empty = true;
    A[32] = dt;
    A[41] = dt;
    A[50] = dt;
    A[59] = dt;
  }
  if ((!B_not_empty) || (reset_trajectory == 1.0)) {
    a = 0.5 * (dt * dt);
    for (i = 0; i < 4; i++) {
      signed char i2;
      i1 = i << 2;
      i2 = b[i1];
      k = i << 3;
      B[k] = a * static_cast<double>(i2);
      B[k + 4] = dt * static_cast<double>(i2);
      i2 = b[i1 + 1];
      B[k + 1] = a * static_cast<double>(i2);
      B[k + 5] = dt * static_cast<double>(i2);
      i2 = b[i1 + 2];
      B[k + 2] = a * static_cast<double>(i2);
      B[k + 6] = dt * static_cast<double>(i2);
      i2 = b[i1 + 3];
      B[k + 3] = a * static_cast<double>(i2);
      B[k + 7] = dt * static_cast<double>(i2);
    }
    B_not_empty = true;
  }
  //  Optimization preparation
  //  Weights
  if ((!Q_not_empty) || (reset_weights == 1.0)) {
    std::memset(&Q[0], 0, 144U * sizeof(double));
    for (k = 0; k < 12; k++) {
      Q[k + 12 * k] = 1.0;
    }
    Q_not_empty = true;
    Q[0] = wx;
    Q[13] = wy;
    Q[26] = wz;
    Q[39] = wpsi;
    Q[52] = wvx;
    Q[65] = wvy;
    Q[78] = wvz;
    Q[91] = wvpsi;
    Q[104] = wax;
    Q[117] = way;
    Q[130] = waz;
    Q[143] = wapsi;
  }
  if ((!Q_last_not_empty) || (reset_weights == 1.0)) {
    std::memset(&Q_last[0], 0, 144U * sizeof(double));
    for (k = 0; k < 12; k++) {
      Q_last[k + 12 * k] = 1.0;
    }
    Q_last_not_empty = true;
    Q_last[0] = wx_last;
    Q_last[13] = wy_last;
    Q_last[26] = wz_last;
    Q_last[39] = wpsi_last;
    Q_last[52] = wvx_last;
    Q_last[65] = wvy_last;
    Q_last[78] = wvz_last;
    Q_last[91] = wvpsi_last;
    Q_last[104] = wax_last;
    Q_last[117] = way_last;
    Q_last[130] = waz_last;
    Q_last[143] = wapsi_last;
  }
  if ((!H_not_empty) || (reset_weights == 1.0)) {
    std::memset(&H[0], 0, 14400U * sizeof(double));
    H_not_empty = true;
    for (i = 0; i < 144; i++) {
      dv1[i] = 2.0 * Q[i];
    }
    for (b_i = 0; b_i < 9; b_i++) {
      i = b_i * 12;
      i1 = (b_i + 1) * 12;
      if (i + 1 > i1) {
        i = 0;
        i1 = 0;
        i3 = 0;
        i4 = 0;
      } else {
        i3 = i;
        i4 = i1;
      }
      k = i1 - i;
      unnamed_idx_1 = i4 - i3;
      for (i1 = 0; i1 < unnamed_idx_1; i1++) {
        for (i4 = 0; i4 < k; i4++) {
          H[(i + i4) + 120 * (i3 + i1)] = dv1[i4 + k * i1];
        }
      }
    }
    for (i = 0; i < 144; i++) {
      dv1[i] = 2.0 * Q_last[i];
    }
    for (b_i = 0; b_i < 10; b_i++) {
      i = b_i * 12;
      i1 = (b_i + 1) * 12;
      if (i + 1 > i1) {
        i = 0;
        i1 = 0;
        i3 = 0;
        i4 = 0;
      } else {
        i3 = i;
        i4 = i1;
      }
      k = i1 - i;
      unnamed_idx_1 = i4 - i3;
      for (i1 = 0; i1 < unnamed_idx_1; i1++) {
        for (i4 = 0; i4 < k; i4++) {
          H[(i + i4) + 120 * (i3 + i1)] = dv1[i4 + k * i1];
        }
      }
    }
  }
  if ((!f_not_empty) || (reset_target == 1.0) || (reset_weights == 1.0)) {
    std::memset(&f[0], 0, 120U * sizeof(double));
    f_not_empty = true;
    dv[0] = -2.0 * target[0];
    dv[1] = -2.0 * target[1];
    dv[2] = -2.0 * target[2];
    dv[3] = -2.0 * target[3];
    std::memset(&dv[4], 0, 8U * sizeof(double));
    for (b_i = 0; b_i < 9; b_i++) {
      i = b_i * 12;
      i1 = (b_i + 1) * 12;
      if (i + 1 > i1) {
        i = 0;
        i1 = 0;
      }
      for (i3 = 0; i3 < 12; i3++) {
        a = 0.0;
        for (i4 = 0; i4 < 12; i4++) {
          a += dv[i4] * Q[i4 + 12 * i3];
        }
        dv2[i3] = a;
      }
      k = i1 - i;
      for (i1 = 0; i1 < k; i1++) {
        f[i + i1] = dv2[i1];
      }
    }
    dv[0] = -2.0 * target[0];
    dv[1] = -2.0 * target[1];
    dv[2] = -2.0 * target[2];
    dv[3] = -2.0 * target[3];
    std::memset(&dv[4], 0, 8U * sizeof(double));
    for (b_i = 0; b_i < 10; b_i++) {
      i = b_i * 12;
      i1 = (b_i + 1) * 12;
      if (i + 1 > i1) {
        i = 0;
        i1 = 0;
      }
      for (i3 = 0; i3 < 12; i3++) {
        a = 0.0;
        for (i4 = 0; i4 < 12; i4++) {
          a += dv[i4] * Q_last[i4 + 12 * i3];
        }
        dv2[i3] = a;
      }
      k = i1 - i;
      for (i1 = 0; i1 < k; i1++) {
        f[i + i1] = dv2[i1];
      }
    }
  }
  //  Linear inequality constraints:
  //  8N for min/max accelerations
  //      if isempty(Aineq)
  //          Aineq = zeros(8*N,N_variables);
  //
  //          for i=1:N
  //              Aineq((i-1)*8+1,(i-1)*8+5) = 1;
  //              Aineq((i-1)*8+2,(i-1)*8+5) = -1;
  //              Aineq((i-1)*8+3,(i-1)*8+6) = 1;
  //              Aineq((i-1)*8+4,(i-1)*8+6) = -1;
  //              Aineq((i-1)*8+5,(i-1)*8+7) = 1;
  //              Aineq((i-1)*8+6,(i-1)*8+7) = -1;
  //              Aineq((i-1)*8+7,(i-1)*8+8) = 1;
  //              Aineq((i-1)*8+8,(i-1)*8+8) = -1;
  //
  //              if (i > 1)
  //  %             Aineq((i-1)*8+1,(i-1)*8+5) = 1;
  //  %             Aineq((i-1)*8+2,(i-1)*8+5) = -1;
  //  %             Aineq((i-1)*8+3,(i-1)*8+6) = 1;
  //  %             Aineq((i-1)*8+4,(i-1)*8+6) = -1;
  //  %             Aineq((i-1)*8+5,(i-1)*8+7) = 1;
  //  %             Aineq((i-1)*8+6,(i-1)*8+7) = -1;
  //  %             Aineq((i-1)*8+7,(i-1)*8+8) = 1;
  //  %             Aineq((i-1)*8+8,(i-1)*8+8) = -1;
  //
  //                  Aineq((i-1)*8+1,(i-2)*8+5) = -1;
  //                  Aineq((i-1)*8+2,(i-2)*8+5) = 1;
  //                  Aineq((i-1)*8+3,(i-2)*8+6) = -1;
  //                  Aineq((i-1)*8+4,(i-2)*8+6) = 1;
  //                  Aineq((i-1)*8+5,(i-2)*8+7) = -1;
  //                  Aineq((i-1)*8+6,(i-2)*8+7) = 1;
  //                  Aineq((i-1)*8+7,(i-2)*8+8) = -1;
  //                  Aineq((i-1)*8+8,(i-2)*8+8) = 1;
  //              end
  //          end
  //      end
  //
  //      if isempty(bineq) || reset_bounds==1
  //          bineq = zeros(8*N,1);
  //
  //          for i=1:N
  //              if (i > 1)
  //                  bineq((i-1)*8+1) = ax_max*dt;
  //                  bineq((i-1)*8+2) = ax_max*dt;
  //                  bineq((i-1)*8+3) = ay_max*dt;
  //                  bineq((i-1)*8+4) = ay_max*dt;
  //                  bineq((i-1)*8+5) = az_max*dt;
  //                  bineq((i-1)*8+6) = az_max*dt;
  //                  bineq((i-1)*8+7) = apsi_max*dt;
  //                  bineq((i-1)*8+8) = apsi_max*dt;
  //              end
  //          end
  //      end
  //      if isempty(Aineq)
  //          Aineq = zeros(12*N,N_variables);
  //
  //          Aineq(1,5) = 1;
  //          Aineq(2,5) = -1;
  //          Aineq(3,6) = 1;
  //          Aineq(4,6) = -1;
  //          Aineq(5,7) = 1;
  //          Aineq(6,7) = -1;
  //          Aineq(7,8) = 1;
  //          Aineq(8,8) = -1;
  //
  //          for i=2:N
  //              Aineq((i-1)*8+1,(i-1)*8+5) = 1;
  //              Aineq((i-1)*8+2,(i-1)*8+5) = -1;
  //              Aineq((i-1)*8+3,(i-1)*8+6) = 1;
  //              Aineq((i-1)*8+4,(i-1)*8+6) = -1;
  //              Aineq((i-1)*8+5,(i-1)*8+7) = 1;
  //              Aineq((i-1)*8+6,(i-1)*8+7) = -1;
  //              Aineq((i-1)*8+7,(i-1)*8+8) = 1;
  //              Aineq((i-1)*8+8,(i-1)*8+8) = -1;
  //
  //              Aineq((i-1)*8+1,(i-2)*8+5) = -1;
  //              Aineq((i-1)*8+2,(i-2)*8+5) = 1;
  //              Aineq((i-1)*8+3,(i-2)*8+6) = -1;
  //              Aineq((i-1)*8+4,(i-2)*8+6) = 1;
  //              Aineq((i-1)*8+5,(i-2)*8+7) = -1;
  //              Aineq((i-1)*8+6,(i-2)*8+7) = 1;
  //              Aineq((i-1)*8+7,(i-2)*8+8) = -1;
  //              Aineq((i-1)*8+8,(i-2)*8+8) = 1;
  //
  //          end
  //      end
  //      if isempty(bineq) || reset_bounds==1
  //          bineq = zeros(8*N,1);
  //
  //  %         bineq(1)
  //
  //          for i=2:N
  //              bineq((i-1)*8+1) = ax_max*dt;
  //              bineq((i-1)*8+2) = ax_max*dt;
  //              bineq((i-1)*8+3) = ay_max*dt;
  //              bineq((i-1)*8+4) = ay_max*dt;
  //              bineq((i-1)*8+5) = az_max*dt;
  //              bineq((i-1)*8+6) = az_max*dt;
  //              bineq((i-1)*8+7) = apsi_max*dt;
  //              bineq((i-1)*8+8) = apsi_max*dt;
  //          end
  //      end
  //  Linear equality constraints:
  //  4N for model constraints
  if ((!Aeq_not_empty) || (reset_trajectory == 1.0)) {
    std::memset(&Aeq[0], 0, 9600U * sizeof(double));
    Aeq_not_empty = true;
    std::memset(&b_I[0], 0, 64U * sizeof(signed char));
    for (k = 0; k < 8; k++) {
      b_I[k + (k << 3)] = 1;
    }
    for (i = 0; i < 8; i++) {
      for (i1 = 0; i1 < 8; i1++) {
        Aeq[i1 + 80 * i] = b_I[i1 + (i << 3)];
      }
    }
    for (i = 0; i < 4; i++) {
      std::memset(&Aeq[i * 80 + 640], 0, 8U * sizeof(double));
    }
    for (b_i = 0; b_i < 9; b_i++) {
      int i5;
      int i6;
      int i7;
      int i8;
      i = (b_i + 1) << 3;
      i1 = (b_i + 2) << 3;
      if (i + 1 > i1) {
        i3 = 0;
        i4 = 0;
      } else {
        i3 = i;
        i4 = i1;
      }
      i5 = (b_i + 1) * 12;
      i6 = (b_i + 2) * 12;
      if (i5 + 1 > i6) {
        i7 = 0;
        i6 = 0;
      } else {
        i7 = i5;
      }
      std::memset(&b_I[0], 0, 64U * sizeof(signed char));
      for (k = 0; k < 8; k++) {
        b_I[k + (k << 3)] = 1;
      }
      for (unnamed_idx_1 = 0; unnamed_idx_1 < 8; unnamed_idx_1++) {
        for (i8 = 0; i8 < 8; i8++) {
          k = i8 + (unnamed_idx_1 << 3);
          c_I[k] = b_I[k];
        }
      }
      for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
        for (i8 = 0; i8 < 8; i8++) {
          c_I[i8 + ((unnamed_idx_1 + 8) << 3)] = 0;
        }
      }
      k = i4 - i3;
      unnamed_idx_1 = i6 - i7;
      for (i4 = 0; i4 < unnamed_idx_1; i4++) {
        for (i6 = 0; i6 < k; i6++) {
          Aeq[(i3 + i6) + 80 * (i7 + i4)] = c_I[i6 + k * i4];
        }
      }
      if (i + 1 > i1) {
        i = 0;
        i1 = 0;
      }
      i3 = b_i * 12;
      if (i3 + 1 > i5) {
        i3 = 0;
        i5 = 0;
      }
      for (i4 = 0; i4 < 8; i4++) {
        for (i6 = 0; i6 < 8; i6++) {
          i7 = i6 + (i4 << 3);
          dv3[i7] = -A[i7];
        }
      }
      for (i4 = 0; i4 < 4; i4++) {
        for (i6 = 0; i6 < 8; i6++) {
          dv3[i6 + ((i4 + 8) << 3)] = -B[i6 + (i4 << 3)];
        }
      }
      k = i1 - i;
      unnamed_idx_1 = i5 - i3;
      for (i1 = 0; i1 < unnamed_idx_1; i1++) {
        for (i4 = 0; i4 < k; i4++) {
          Aeq[(i + i4) + 80 * (i3 + i1)] = dv3[i4 + k * i1];
        }
      }
    }
  }
  //  Upper/lower bounds:
  if ((!LB_not_empty) || (reset_bounds == 1.0)) {
    std::memset(&LB[0], 0, 120U * sizeof(double));
    LB_not_empty = true;
    dv[0] = rtMinusInf;
    dv[1] = rtMinusInf;
    dv[2] = rtMinusInf;
    dv[3] = rtMinusInf;
    dv[4] = -vx_max;
    dv[5] = -vy_max;
    dv[6] = -vz_max;
    dv[7] = -vpsi_max;
    dv[8] = -ax_max;
    dv[9] = -ay_max;
    dv[10] = -az_max;
    dv[11] = -apsi_max;
    for (b_i = 0; b_i < 10; b_i++) {
      i = b_i * 12;
      i1 = (b_i + 1) * 12;
      if (i + 1 > i1) {
        i = 0;
        i1 = 0;
      }
      k = i1 - i;
      for (i1 = 0; i1 < k; i1++) {
        LB[i + i1] = dv[i1];
      }
    }
  }
  if ((!UB_not_empty) || (reset_bounds == 1.0)) {
    std::memset(&UB[0], 0, 120U * sizeof(double));
    UB_not_empty = true;
    dv[0] = rtInf;
    dv[1] = rtInf;
    dv[2] = rtInf;
    dv[3] = rtInf;
    dv[4] = vx_max;
    dv[5] = vy_max;
    dv[6] = vz_max;
    dv[7] = vpsi_max;
    dv[8] = ax_max;
    dv[9] = ay_max;
    dv[10] = az_max;
    dv[11] = apsi_max;
    for (b_i = 0; b_i < 10; b_i++) {
      i = b_i * 12;
      i1 = (b_i + 1) * 12;
      if (i + 1 > i1) {
        i = 0;
        i1 = 0;
      }
      k = i1 - i;
      for (i1 = 0; i1 < k; i1++) {
        UB[i + i1] = dv[i1];
      }
    }
  }
  //  Options
  //  For HDL coder, doesn't like inf
  //  Step
  if ((!X_not_empty) || (reset_trajectory == 1.0)) {
    std::memset(&X[0], 0, 120U * sizeof(double));
    X_not_empty = true;
    std::copy(&x[0], &x[12], &X[0]);
    for (b_i = 0; b_i < 9; b_i++) {
      double b_unnamed_idx_1;
      double unnamed_idx_0;
      double unnamed_idx_2;
      double unnamed_idx_3;
      k = b_i * 12;
      std::copy(&X[k], &X[static_cast<int>(k + 8U)], &y[0]);
      unnamed_idx_0 = X[k + 8];
      b_unnamed_idx_1 = X[k + 9];
      unnamed_idx_2 = X[k + 10];
      unnamed_idx_3 = X[k + 11];
      k = (b_i + 1) * 12;
      for (i = 0; i < 8; i++) {
        a = 0.0;
        for (i1 = 0; i1 < 8; i1++) {
          a += A[i + (i1 << 3)] * y[i1];
        }
        X[i + k] = a + (((B[i] * unnamed_idx_0 + B[i + 8] * b_unnamed_idx_1) +
                         B[i + 16] * unnamed_idx_2) +
                        B[i + 24] * unnamed_idx_3);
      }
      X[k + 8] = X[8];
      X[k + 9] = X[9];
      X[k + 10] = X[10];
      X[k + 11] = X[11];
    }
    //          Z = [X];
  }
  std::copy(&x[0], &x[12], &X[0]);
  for (i = 0; i < 8; i++) {
    std::copy(&A[i * 8], &A[static_cast<int>(i * 8 + 8U)], &dv3[i * 8]);
  }
  for (i = 0; i < 4; i++) {
    std::copy(&B[i * 8], &B[static_cast<int>(i * 8 + 8U)], &dv3[i * 8 + 64]);
  }
  for (b_i = 0; b_i < 8; b_i++) {
    a = 0.0;
    for (i = 0; i < 12; i++) {
      a += dv3[b_i + (i << 3)] * x[i];
    }
    beq[b_i] = a;
  }
  //      bineq(1) = ax_max*dt+x(5);
  //      bineq(2) = ax_max*dt-x(5);
  //      bineq(3) = ay_max*dt+x(6);
  //      bineq(4) = ay_max*dt-x(6);
  //      bineq(5) = az_max*dt+x(7);
  //      bineq(6) = az_max*dt-x(7);
  //      bineq(7) = apsi_max*dt+x(8);
  //      bineq(8) = apsi_max*dt-x(8);
  //      X = quadprog(H,f,Aineq,bineq,Aeq,beq,LB,UB,X,options);
  std::copy(&X[0], &X[120], &dv4[0]);
  coder::quadprog(H, f, Aeq, beq, LB, UB, dv4, X);
  //      X = Z(1:8*N);
  std::copy(&X[0], &X[12], &x[0]);
  for (i = 0; i < 8; i++) {
    a = 0.0;
    for (i1 = 0; i1 < 8; i1++) {
      a += A[i + (i1 << 3)] * X[i1 + 108];
    }
    y[i] = a;
    dv5[i] = ((B[i] * X[116] + B[i + 8] * X[117]) + B[i + 16] * X[118]) +
             B[i + 24] * X[119];
  }
  std::copy(&X[12], &X[120], &dv4[0]);
  for (i = 0; i < 8; i++) {
    dv4[i + 108] = y[i] + dv5[i];
  }
  dv4[116] = X[116];
  dv4[117] = X[117];
  dv4[118] = X[118];
  dv4[119] = X[119];
  //      Z = [X];
  for (b_i = 0; b_i < 120; b_i++) {
    a = dv4[b_i];
    X[b_i] = a;
    planned_traj[b_i] = a;
  }
}

void MPCStepFunction_init()
{
  std::memset(&beq[0], 0, 80U * sizeof(double));
}

void Q_last_not_empty_init()
{
  Q_last_not_empty = false;
}

void Q_not_empty_init()
{
  Q_not_empty = false;
}

void UB_not_empty_init()
{
  UB_not_empty = false;
}

void X_not_empty_init()
{
  X_not_empty = false;
}

void f_not_empty_init()
{
  f_not_empty = false;
}

} // namespace pos_MPC

// End of code generation (MPCStepFunction.cpp)
