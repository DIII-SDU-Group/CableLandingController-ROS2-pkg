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
static boolean_T A_not_empty;

static boolean_T B_not_empty;

static boolean_T Q_not_empty;

static boolean_T Q_last_not_empty;

static boolean_T H_not_empty;

static boolean_T f_not_empty;

static boolean_T Aeq_not_empty;

static double beq[120];

static boolean_T LB_not_empty;

static boolean_T UB_not_empty;

static double X[180];

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

void MPCStepFunction(double x[9], const double target[3], double dt,
                     double vx_max, double vy_max, double vz_max, double ax_max,
                     double ay_max, double az_max, double wx, double wy,
                     double wz, double wvx, double wvy, double wvz, double wax,
                     double way, double waz, double wx_last, double wy_last,
                     double wz_last, double wvx_last, double wvy_last,
                     double wvz_last, double wax_last, double way_last,
                     double waz_last, double reset_target,
                     double reset_trajectory, double reset_bounds,
                     double reset_weights, double planned_traj[180])
{
  static double H[32400];
  static double Aeq[21600];
  static double LB[180];
  static double UB[180];
  static double f[180];
  static double Q[81];
  static double Q_last[81];
  static double A[36];
  static double B[18];
  static const signed char b[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
  double dv4[180];
  double dv1[81];
  double dv3[54];
  double dv[9];
  double dv2[9];
  double dv5[6];
  double y[6];
  double a;
  int b_i;
  int i;
  int i2;
  int i3;
  int i4;
  int k;
  int unnamed_idx_1;
  signed char c_I[54];
  signed char b_I[36];
  if (!isInitialized_MPCStepFunction) {
    MPCStepFunction_initialize();
  }
  //  Params
  if (!A_not_empty) {
    std::memset(&A[0], 0, 36U * sizeof(double));
    for (k = 0; k < 6; k++) {
      A[k + 6 * k] = 1.0;
    }
    A_not_empty = true;
    A[18] = dt;
    A[25] = dt;
    A[32] = dt;
  }
  if ((!B_not_empty) || (reset_trajectory == 1.0)) {
    a = 0.5 * (dt * dt);
    for (i = 0; i < 3; i++) {
      signed char i1;
      i1 = b[3 * i];
      B[6 * i] = a * static_cast<double>(i1);
      B[6 * i + 3] = dt * static_cast<double>(i1);
      i1 = b[3 * i + 1];
      B[6 * i + 1] = a * static_cast<double>(i1);
      B[6 * i + 4] = dt * static_cast<double>(i1);
      i1 = b[3 * i + 2];
      B[6 * i + 2] = a * static_cast<double>(i1);
      B[6 * i + 5] = dt * static_cast<double>(i1);
    }
    B_not_empty = true;
  }
  //  Optimization preparation
  //  Weights
  if ((!Q_not_empty) || (reset_weights == 1.0)) {
    std::memset(&Q[0], 0, 81U * sizeof(double));
    for (k = 0; k < 9; k++) {
      Q[k + 9 * k] = 1.0;
    }
    Q_not_empty = true;
    Q[0] = wx;
    Q[10] = wy;
    Q[20] = wz;
    Q[30] = wvx;
    Q[40] = wvy;
    Q[50] = wvz;
    Q[60] = wax;
    Q[70] = way;
    Q[80] = waz;
  }
  if ((!Q_last_not_empty) || (reset_weights == 1.0)) {
    std::memset(&Q_last[0], 0, 81U * sizeof(double));
    for (k = 0; k < 9; k++) {
      Q_last[k + 9 * k] = 1.0;
    }
    Q_last_not_empty = true;
    Q_last[0] = wx_last;
    Q_last[10] = wy_last;
    Q_last[20] = wz_last;
    Q_last[30] = wvx_last;
    Q_last[40] = wvy_last;
    Q_last[50] = wvz_last;
    Q_last[60] = wax_last;
    Q_last[70] = way_last;
    Q_last[80] = waz_last;
  }
  if ((!H_not_empty) || (reset_weights == 1.0)) {
    std::memset(&H[0], 0, 32400U * sizeof(double));
    H_not_empty = true;
    for (i = 0; i < 81; i++) {
      dv1[i] = 2.0 * Q[i];
    }
    for (b_i = 0; b_i < 19; b_i++) {
      i = b_i * 9;
      i2 = (b_i + 1) * 9;
      if (i + 1 > i2) {
        i = 0;
        i2 = 0;
        i3 = 0;
        i4 = 0;
      } else {
        i3 = i;
        i4 = i2;
      }
      k = i2 - i;
      unnamed_idx_1 = i4 - i3;
      for (i2 = 0; i2 < unnamed_idx_1; i2++) {
        for (i4 = 0; i4 < k; i4++) {
          H[(i + i4) + 180 * (i3 + i2)] = dv1[i4 + k * i2];
        }
      }
    }
    for (i = 0; i < 81; i++) {
      dv1[i] = 2.0 * Q_last[i];
    }
    for (b_i = 0; b_i < 20; b_i++) {
      i = b_i * 9;
      i2 = (b_i + 1) * 9;
      if (i + 1 > i2) {
        i = 0;
        i2 = 0;
        i3 = 0;
        i4 = 0;
      } else {
        i3 = i;
        i4 = i2;
      }
      k = i2 - i;
      unnamed_idx_1 = i4 - i3;
      for (i2 = 0; i2 < unnamed_idx_1; i2++) {
        for (i4 = 0; i4 < k; i4++) {
          H[(i + i4) + 180 * (i3 + i2)] = dv1[i4 + k * i2];
        }
      }
    }
  }
  if ((!f_not_empty) || (reset_target == 1.0) || (reset_weights == 1.0)) {
    std::memset(&f[0], 0, 180U * sizeof(double));
    f_not_empty = true;
    dv[0] = -2.0 * target[0];
    dv[1] = -2.0 * target[1];
    dv[2] = -2.0 * target[2];
    dv[3] = -0.0;
    dv[4] = -0.0;
    dv[5] = -0.0;
    dv[6] = -0.0;
    dv[7] = -0.0;
    dv[8] = -0.0;
    for (b_i = 0; b_i < 19; b_i++) {
      i = b_i * 9;
      i2 = (b_i + 1) * 9;
      if (i + 1 > i2) {
        i = 0;
        i2 = 0;
      }
      for (i3 = 0; i3 < 9; i3++) {
        a = 0.0;
        for (i4 = 0; i4 < 9; i4++) {
          a += dv[i4] * Q[i4 + 9 * i3];
        }
        dv2[i3] = a;
      }
      k = i2 - i;
      for (i2 = 0; i2 < k; i2++) {
        f[i + i2] = dv2[i2];
      }
    }
    dv[0] = -2.0 * target[0];
    dv[1] = -2.0 * target[1];
    dv[2] = -2.0 * target[2];
    dv[3] = -0.0;
    dv[4] = -0.0;
    dv[5] = -0.0;
    dv[6] = -0.0;
    dv[7] = -0.0;
    dv[8] = -0.0;
    for (b_i = 0; b_i < 20; b_i++) {
      i = b_i * 9;
      i2 = (b_i + 1) * 9;
      if (i + 1 > i2) {
        i = 0;
        i2 = 0;
      }
      for (i3 = 0; i3 < 9; i3++) {
        a = 0.0;
        for (i4 = 0; i4 < 9; i4++) {
          a += dv[i4] * Q_last[i4 + 9 * i3];
        }
        dv2[i3] = a;
      }
      k = i2 - i;
      for (i2 = 0; i2 < k; i2++) {
        f[i + i2] = dv2[i2];
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
    std::memset(&Aeq[0], 0, 21600U * sizeof(double));
    Aeq_not_empty = true;
    for (i = 0; i < 36; i++) {
      b_I[i] = 0;
    }
    for (k = 0; k < 6; k++) {
      b_I[k + 6 * k] = 1;
    }
    for (i = 0; i < 6; i++) {
      for (i2 = 0; i2 < 6; i2++) {
        Aeq[i2 + 120 * i] = b_I[i2 + 6 * i];
      }
    }
    for (i = 0; i < 3; i++) {
      for (i2 = 0; i2 < 6; i2++) {
        Aeq[i2 + 120 * (i + 6)] = 0.0;
      }
    }
    for (b_i = 0; b_i < 19; b_i++) {
      int i5;
      int i6;
      int i7;
      int i8;
      i = (b_i + 1) * 6;
      i2 = (b_i + 2) * 6;
      if (i + 1 > i2) {
        i3 = 0;
        i4 = 0;
      } else {
        i3 = i;
        i4 = i2;
      }
      i5 = (b_i + 1) * 9;
      i6 = (b_i + 2) * 9;
      if (i5 + 1 > i6) {
        i7 = 0;
        i6 = 0;
      } else {
        i7 = i5;
      }
      for (unnamed_idx_1 = 0; unnamed_idx_1 < 36; unnamed_idx_1++) {
        b_I[unnamed_idx_1] = 0;
      }
      for (k = 0; k < 6; k++) {
        b_I[k + 6 * k] = 1;
      }
      for (unnamed_idx_1 = 0; unnamed_idx_1 < 6; unnamed_idx_1++) {
        for (i8 = 0; i8 < 6; i8++) {
          k = i8 + 6 * unnamed_idx_1;
          c_I[k] = b_I[k];
        }
      }
      for (unnamed_idx_1 = 0; unnamed_idx_1 < 3; unnamed_idx_1++) {
        for (i8 = 0; i8 < 6; i8++) {
          c_I[i8 + 6 * (unnamed_idx_1 + 6)] = 0;
        }
      }
      k = i4 - i3;
      unnamed_idx_1 = i6 - i7;
      for (i4 = 0; i4 < unnamed_idx_1; i4++) {
        for (i6 = 0; i6 < k; i6++) {
          Aeq[(i3 + i6) + 120 * (i7 + i4)] = c_I[i6 + k * i4];
        }
      }
      if (i + 1 > i2) {
        i = 0;
        i2 = 0;
      }
      i3 = b_i * 9;
      if (i3 + 1 > i5) {
        i3 = 0;
        i5 = 0;
      }
      for (i4 = 0; i4 < 6; i4++) {
        for (i6 = 0; i6 < 6; i6++) {
          i7 = i6 + 6 * i4;
          dv3[i7] = -A[i7];
        }
      }
      for (i4 = 0; i4 < 3; i4++) {
        for (i6 = 0; i6 < 6; i6++) {
          dv3[i6 + 6 * (i4 + 6)] = -B[i6 + 6 * i4];
        }
      }
      k = i2 - i;
      unnamed_idx_1 = i5 - i3;
      for (i2 = 0; i2 < unnamed_idx_1; i2++) {
        for (i4 = 0; i4 < k; i4++) {
          Aeq[(i + i4) + 120 * (i3 + i2)] = dv3[i4 + k * i2];
        }
      }
    }
  }
  //  Upper/lower bounds:
  if ((!LB_not_empty) || (reset_bounds == 1.0)) {
    std::memset(&LB[0], 0, 180U * sizeof(double));
    LB_not_empty = true;
    dv[0] = rtMinusInf;
    dv[1] = rtMinusInf;
    dv[2] = rtMinusInf;
    dv[3] = -vx_max;
    dv[4] = -vy_max;
    dv[5] = -vz_max;
    dv[6] = -ax_max;
    dv[7] = -ay_max;
    dv[8] = -az_max;
    for (b_i = 0; b_i < 20; b_i++) {
      i = b_i * 9;
      i2 = (b_i + 1) * 9;
      if (i + 1 > i2) {
        i = 0;
        i2 = 0;
      }
      k = i2 - i;
      for (i2 = 0; i2 < k; i2++) {
        LB[i + i2] = dv[i2];
      }
    }
  }
  if ((!UB_not_empty) || (reset_bounds == 1.0)) {
    std::memset(&UB[0], 0, 180U * sizeof(double));
    UB_not_empty = true;
    dv[0] = rtInf;
    dv[1] = rtInf;
    dv[2] = rtInf;
    dv[3] = vx_max;
    dv[4] = vy_max;
    dv[5] = vz_max;
    dv[6] = ax_max;
    dv[7] = ay_max;
    dv[8] = az_max;
    for (b_i = 0; b_i < 20; b_i++) {
      i = b_i * 9;
      i2 = (b_i + 1) * 9;
      if (i + 1 > i2) {
        i = 0;
        i2 = 0;
      }
      k = i2 - i;
      for (i2 = 0; i2 < k; i2++) {
        UB[i + i2] = dv[i2];
      }
    }
  }
  //  Options
  //  For HDL coder, doesn't like inf
  //  Step
  if ((!X_not_empty) || (reset_trajectory == 1.0)) {
    std::memset(&X[0], 0, 180U * sizeof(double));
    X_not_empty = true;
    std::copy(&x[0], &x[9], &X[0]);
    for (b_i = 0; b_i < 19; b_i++) {
      double b_unnamed_idx_1;
      double unnamed_idx_0;
      double unnamed_idx_2;
      k = b_i * 9;
      for (i = 0; i < 6; i++) {
        y[i] = X[i + k];
      }
      unnamed_idx_0 = X[k + 6];
      b_unnamed_idx_1 = X[k + 7];
      unnamed_idx_2 = X[k + 8];
      k = (b_i + 1) * 9;
      for (i = 0; i < 6; i++) {
        a = 0.0;
        for (i2 = 0; i2 < 6; i2++) {
          a += A[i + 6 * i2] * y[i2];
        }
        X[i + k] = a + ((B[i] * unnamed_idx_0 + B[i + 6] * b_unnamed_idx_1) +
                        B[i + 12] * unnamed_idx_2);
      }
      X[k + 6] = X[6];
      X[k + 7] = X[7];
      X[k + 8] = X[8];
    }
    //          Z = [X];
  }
  std::copy(&x[0], &x[9], &X[0]);
  for (i = 0; i < 6; i++) {
    for (i2 = 0; i2 < 6; i2++) {
      i3 = i2 + 6 * i;
      dv3[i3] = A[i3];
    }
  }
  for (i = 0; i < 3; i++) {
    for (i2 = 0; i2 < 6; i2++) {
      dv3[i2 + 6 * (i + 6)] = B[i2 + 6 * i];
    }
  }
  for (b_i = 0; b_i < 6; b_i++) {
    a = 0.0;
    for (i = 0; i < 9; i++) {
      a += dv3[b_i + 6 * i] * x[i];
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
  std::copy(&X[0], &X[180], &dv4[0]);
  coder::quadprog(H, f, Aeq, beq, LB, UB, dv4, X);
  //      X = Z(1:8*N);
  std::copy(&X[0], &X[9], &x[0]);
  //      Z = [X];
  std::copy(&X[0], &X[180], &planned_traj[0]);
  for (i = 0; i < 171; i++) {
    X[i] = X[i + 9];
  }
  for (i = 0; i < 6; i++) {
    a = 0.0;
    for (i2 = 0; i2 < 6; i2++) {
      a += A[i + 6 * i2] * X[i2 + 162];
    }
    y[i] = a;
    dv5[i] = (B[i] * X[168] + B[i + 6] * X[169]) + B[i + 12] * X[170];
  }
  for (i = 0; i < 6; i++) {
    X[i + 171] = y[i] + dv5[i];
  }
  X[177] = X[168];
  X[178] = X[169];
  X[179] = X[170];
}

void MPCStepFunction_init()
{
  std::memset(&beq[0], 0, 120U * sizeof(double));
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
