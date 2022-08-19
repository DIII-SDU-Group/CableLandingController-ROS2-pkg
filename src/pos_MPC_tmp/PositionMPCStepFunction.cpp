//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// PositionMPCStepFunction.cpp
//
// Code generation for function 'PositionMPCStepFunction'
//

// Include files
#include "PositionMPCStepFunction.h"
#include "PositionMPCStepFunction_data.h"
#include "PositionMPCStepFunction_initialize.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cstring>

// Variable Definitions
namespace pos_MPC {
static double A[16];

static boolean_T B_not_empty;

static boolean_T Q_not_empty;

static boolean_T H_not_empty;

static boolean_T f_not_empty;

static boolean_T Aineq_not_empty;

static boolean_T bineq_not_empty;

static boolean_T Aeq_not_empty;

static double beq[80];

static boolean_T LB_not_empty;

static boolean_T UB_not_empty;

static double X[160];

static boolean_T X_not_empty;

} // namespace pos_MPC

// Function Definitions
namespace pos_MPC {
void Aeq_not_empty_init()
{
  Aeq_not_empty = false;
}

void Aineq_not_empty_init()
{
  Aineq_not_empty = false;
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

void PositionMPCStepFunction(double x[8], const double target[4], double dt,
                             double vx_max, double vy_max, double vz_max,
                             double vpsi_max, double ax_max, double ay_max,
                             double az_max, double apsi_max, double wx,
                             double wy, double wz, double wpsi, double wvx,
                             double wvy, double wvz, double wvpsi,
                             double reset_target, double reset_trajectory,
                             double reset_bounds, double reset_weights,
                             double planned_traj[160])
{
  static double Aineq[25600];
  static double H[25600];
  static double Aeq[12800];
  static double LB[160];
  static double UB[160];
  static double bineq[160];
  static double f[160];
  static double Q[64];
  static double B[16];
  static const signed char a[16]{1, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 1};
  double dv4[160];
  double dv1[64];
  double dv3[32];
  double dv[8];
  double dv2[8];
  double dv5[4];
  double y[4];
  double bineq_tmp;
  double d;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  int Aineq_tmp_tmp;
  int b_i;
  int i;
  int i1;
  int i2;
  int i3;
  int k;
  int unnamed_idx_1;
  signed char b_Aeq_tmp[32];
  signed char Aeq_tmp[16];
  if (!isInitialized_PositionMPCStepFunction) {
    PositionMPCStepFunction_initialize();
  }
  //  Params
  if ((!B_not_empty) || (reset_trajectory == 1.0)) {
    for (i = 0; i < 16; i++) {
      B[i] = static_cast<double>(a[i]) * dt;
    }
    B_not_empty = true;
  }
  //  Optimization preparation
  //  Weights
  if ((!Q_not_empty) || (reset_weights == 1.0)) {
    std::memset(&Q[0], 0, 64U * sizeof(double));
    for (k = 0; k < 8; k++) {
      Q[k + (k << 3)] = 1.0;
    }
    Q_not_empty = true;
    Q[0] = wx;
    Q[9] = wy;
    Q[18] = wz;
    Q[27] = wpsi;
    Q[36] = wvx;
    Q[45] = wvy;
    Q[54] = wvz;
    Q[63] = wvpsi;
  }
  if ((!H_not_empty) || (reset_weights == 1.0)) {
    std::memset(&H[0], 0, 25600U * sizeof(double));
    H_not_empty = true;
    for (i = 0; i < 64; i++) {
      dv1[i] = 2.0 * Q[i];
    }
    for (b_i = 0; b_i < 20; b_i++) {
      i = b_i << 3;
      i1 = (b_i + 1) << 3;
      if (i + 1 > i1) {
        i = 0;
        i1 = 0;
        i2 = 0;
        i3 = 0;
      } else {
        i2 = i;
        i3 = i1;
      }
      k = i1 - i;
      unnamed_idx_1 = i3 - i2;
      for (i1 = 0; i1 < unnamed_idx_1; i1++) {
        for (i3 = 0; i3 < k; i3++) {
          H[(i + i3) + 160 * (i2 + i1)] = dv1[i3 + k * i1];
        }
      }
    }
  }
  if ((!f_not_empty) || (reset_target == 1.0) || (reset_weights == 1.0)) {
    std::memset(&f[0], 0, 160U * sizeof(double));
    f_not_empty = true;
    dv[0] = -2.0 * target[0];
    dv[1] = -2.0 * target[1];
    dv[2] = -2.0 * target[2];
    dv[3] = -2.0 * target[3];
    dv[4] = -0.0;
    dv[5] = -0.0;
    dv[6] = -0.0;
    dv[7] = -0.0;
    for (b_i = 0; b_i < 20; b_i++) {
      i = b_i << 3;
      i1 = (b_i + 1) << 3;
      if (i + 1 > i1) {
        i = 0;
        i1 = 0;
      }
      for (i2 = 0; i2 < 8; i2++) {
        bineq_tmp = 0.0;
        for (i3 = 0; i3 < 8; i3++) {
          bineq_tmp += dv[i3] * Q[i3 + (i2 << 3)];
        }
        dv2[i2] = bineq_tmp;
      }
      k = i1 - i;
      for (i1 = 0; i1 < k; i1++) {
        f[i + i1] = dv2[i1];
      }
    }
  }
  //  Linear inequality constraints:
  //  8N for min/max accelerations
  if (!Aineq_not_empty) {
    std::memset(&Aineq[0], 0, 25600U * sizeof(double));
    Aineq_not_empty = true;
    for (b_i = 0; b_i < 20; b_i++) {
      k = b_i << 3;
      unnamed_idx_1 = k + 160 * (k + 4);
      Aineq[unnamed_idx_1] = 1.0;
      Aineq[unnamed_idx_1 + 1] = -1.0;
      unnamed_idx_1 = k + 160 * (k + 5);
      Aineq[unnamed_idx_1 + 2] = 1.0;
      Aineq[unnamed_idx_1 + 3] = -1.0;
      unnamed_idx_1 = k + 160 * (k + 6);
      Aineq[unnamed_idx_1 + 4] = 1.0;
      Aineq[unnamed_idx_1 + 5] = -1.0;
      unnamed_idx_1 = k + 160 * (k + 7);
      Aineq[unnamed_idx_1 + 6] = 1.0;
      Aineq[unnamed_idx_1 + 7] = -1.0;
      if (b_i + 1 > 1) {
        Aineq_tmp_tmp = (b_i - 1) << 3;
        unnamed_idx_1 = k + 160 * (Aineq_tmp_tmp + 4);
        Aineq[unnamed_idx_1] = -1.0;
        Aineq[unnamed_idx_1 + 1] = 1.0;
        unnamed_idx_1 = k + 160 * (Aineq_tmp_tmp + 5);
        Aineq[unnamed_idx_1 + 2] = -1.0;
        Aineq[unnamed_idx_1 + 3] = 1.0;
        unnamed_idx_1 = k + 160 * (Aineq_tmp_tmp + 6);
        Aineq[unnamed_idx_1 + 4] = -1.0;
        Aineq[unnamed_idx_1 + 5] = 1.0;
        k += 160 * (Aineq_tmp_tmp + 7);
        Aineq[k + 6] = -1.0;
        Aineq[k + 7] = 1.0;
      }
    }
  }
  if ((!bineq_not_empty) || (reset_bounds == 1.0)) {
    std::memset(&bineq[0], 0, 160U * sizeof(double));
    bineq_not_empty = true;
    for (b_i = 0; b_i < 20; b_i++) {
      if (b_i + 1 > 1) {
        bineq_tmp = ax_max * dt;
        k = b_i << 3;
        bineq[k] = bineq_tmp;
        bineq[k + 1] = bineq_tmp;
        bineq_tmp = ay_max * dt;
        bineq[k + 2] = bineq_tmp;
        bineq[k + 3] = bineq_tmp;
        bineq_tmp = az_max * dt;
        bineq[k + 4] = bineq_tmp;
        bineq[k + 5] = bineq_tmp;
        bineq_tmp = apsi_max * dt;
        bineq[k + 6] = bineq_tmp;
        bineq[k + 7] = bineq_tmp;
      }
    }
  }
  //  Linear equality constraints:
  //  4N for model constraints
  if ((!Aeq_not_empty) || (reset_trajectory == 1.0)) {
    std::memset(&Aeq[0], 0, 12800U * sizeof(double));
    Aeq_not_empty = true;
    for (i = 0; i < 16; i++) {
      Aeq_tmp[i] = 0;
    }
    Aeq_tmp[0] = 1;
    Aeq_tmp[5] = 1;
    Aeq_tmp[10] = 1;
    Aeq_tmp[15] = 1;
    for (i = 0; i < 4; i++) {
      signed char i4;
      signed char i5;
      signed char i6;
      signed char i9;
      unnamed_idx_1 = i << 2;
      i4 = Aeq_tmp[unnamed_idx_1];
      Aeq[80 * i] = i4;
      k = 80 * (i + 4);
      Aeq[k] = 0.0;
      i5 = Aeq_tmp[unnamed_idx_1 + 1];
      Aeq[80 * i + 1] = i5;
      Aeq[k + 1] = 0.0;
      i6 = Aeq_tmp[unnamed_idx_1 + 2];
      Aeq[80 * i + 2] = i6;
      Aeq[k + 2] = 0.0;
      i9 = Aeq_tmp[unnamed_idx_1 + 3];
      Aeq[80 * i + 3] = i9;
      Aeq[k + 3] = 0.0;
      b_Aeq_tmp[unnamed_idx_1] = i4;
      k = (i + 4) << 2;
      b_Aeq_tmp[k] = 0;
      b_Aeq_tmp[unnamed_idx_1 + 1] = i5;
      b_Aeq_tmp[k + 1] = 0;
      b_Aeq_tmp[unnamed_idx_1 + 2] = i6;
      b_Aeq_tmp[k + 2] = 0;
      b_Aeq_tmp[unnamed_idx_1 + 3] = i9;
      b_Aeq_tmp[k + 3] = 0;
      dv3[unnamed_idx_1] = -A[unnamed_idx_1];
      dv3[k] = -B[unnamed_idx_1];
      dv3[unnamed_idx_1 + 1] = -A[unnamed_idx_1 + 1];
      dv3[k + 1] = -B[unnamed_idx_1 + 1];
      dv3[unnamed_idx_1 + 2] = -A[unnamed_idx_1 + 2];
      dv3[k + 2] = -B[unnamed_idx_1 + 2];
      dv3[unnamed_idx_1 + 3] = -A[unnamed_idx_1 + 3];
      dv3[k + 3] = -B[unnamed_idx_1 + 3];
    }
    for (b_i = 0; b_i < 19; b_i++) {
      int i7;
      int i8;
      i = (b_i + 1) << 2;
      i1 = (b_i + 2) << 2;
      if (i + 1 > i1) {
        i2 = 0;
        i3 = 0;
      } else {
        i2 = i;
        i3 = i1;
      }
      Aineq_tmp_tmp = (b_i + 1) << 3;
      i7 = (b_i + 2) << 3;
      if (Aineq_tmp_tmp + 1 > i7) {
        i8 = 0;
        i7 = 0;
      } else {
        i8 = Aineq_tmp_tmp;
      }
      k = i3 - i2;
      unnamed_idx_1 = i7 - i8;
      for (i3 = 0; i3 < unnamed_idx_1; i3++) {
        for (i7 = 0; i7 < k; i7++) {
          Aeq[(i2 + i7) + 80 * (i8 + i3)] = b_Aeq_tmp[i7 + k * i3];
        }
      }
      if (i + 1 > i1) {
        i = 0;
        i1 = 0;
      }
      i2 = b_i << 3;
      if (i2 + 1 > Aineq_tmp_tmp) {
        i2 = 0;
        Aineq_tmp_tmp = 0;
      }
      k = i1 - i;
      unnamed_idx_1 = Aineq_tmp_tmp - i2;
      for (i1 = 0; i1 < unnamed_idx_1; i1++) {
        for (i3 = 0; i3 < k; i3++) {
          Aeq[(i + i3) + 80 * (i2 + i1)] = dv3[i3 + k * i1];
        }
      }
    }
  }
  //  Upper/lower bounds:
  if ((!LB_not_empty) || (reset_bounds == 1.0)) {
    std::memset(&LB[0], 0, 160U * sizeof(double));
    LB_not_empty = true;
    dv[0] = rtMinusInf;
    dv[1] = rtMinusInf;
    dv[2] = rtMinusInf;
    dv[3] = rtMinusInf;
    dv[4] = -vx_max;
    dv[5] = -vy_max;
    dv[6] = -vz_max;
    dv[7] = -vpsi_max;
    for (b_i = 0; b_i < 20; b_i++) {
      i = b_i << 3;
      i1 = (b_i + 1) << 3;
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
    std::memset(&UB[0], 0, 160U * sizeof(double));
    UB_not_empty = true;
    dv[0] = rtInf;
    dv[1] = rtInf;
    dv[2] = rtInf;
    dv[3] = rtInf;
    dv[4] = vx_max;
    dv[5] = vy_max;
    dv[6] = vz_max;
    dv[7] = vpsi_max;
    for (b_i = 0; b_i < 20; b_i++) {
      i = b_i << 3;
      i1 = (b_i + 1) << 3;
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
  //  Step
  if ((!X_not_empty) || (reset_trajectory == 1.0)) {
    std::memset(&X[0], 0, 160U * sizeof(double));
    X_not_empty = true;
    std::copy(&x[0], &x[8], &X[0]);
    for (b_i = 0; b_i < 19; b_i++) {
      k = b_i << 3;
      unnamed_idx_1 = (b_i + 1) << 3;
      bineq_tmp = X[k];
      d = X[k + 1];
      d1 = X[k + 2];
      d2 = X[k + 3];
      d3 = X[k + 4];
      d4 = X[k + 5];
      d5 = X[k + 6];
      d6 = X[k + 7];
      for (i = 0; i < 4; i++) {
        k = i + unnamed_idx_1;
        X[k] = (((A[i] * bineq_tmp + A[i + 4] * d) + A[i + 8] * d1) +
                A[i + 12] * d2) +
               (((B[i] * d3 + B[i + 4] * d4) + B[i + 8] * d5) + B[i + 12] * d6);
        X[k + 4] = X[i + 4];
      }
    }
    //          Z = [X];
  }
  std::copy(&x[0], &x[8], &X[0]);
  for (i = 0; i < 4; i++) {
    i1 = i << 2;
    dv3[i1] = A[i1];
    i2 = (i + 4) << 2;
    dv3[i2] = B[i1];
    dv3[i1 + 1] = A[i1 + 1];
    dv3[i2 + 1] = B[i1 + 1];
    dv3[i1 + 2] = A[i1 + 2];
    dv3[i2 + 2] = B[i1 + 2];
    dv3[i1 + 3] = A[i1 + 3];
    dv3[i2 + 3] = B[i1 + 3];
  }
  for (b_i = 0; b_i < 4; b_i++) {
    bineq_tmp = 0.0;
    for (i = 0; i < 8; i++) {
      bineq_tmp += dv3[b_i + (i << 2)] * x[i];
    }
    beq[b_i] = bineq_tmp;
  }
  bineq_tmp = ax_max * dt;
  bineq[0] = bineq_tmp + x[4];
  bineq[1] = bineq_tmp - x[4];
  bineq_tmp = ay_max * dt;
  bineq[2] = bineq_tmp + x[5];
  bineq[3] = bineq_tmp - x[5];
  bineq_tmp = az_max * dt;
  bineq[4] = bineq_tmp + x[6];
  bineq[5] = bineq_tmp - x[6];
  bineq_tmp = apsi_max * dt;
  bineq[6] = bineq_tmp + x[7];
  bineq[7] = bineq_tmp - x[7];
  std::copy(&X[0], &X[160], &dv4[0]);
  coder::quadprog(H, f, Aineq, bineq, Aeq, beq, LB, UB, dv4, X);
  //      Z = quadprog(H,f,[],[],Aeq,beq,[],[],Z,options);
  //      X = Z(1:8*N);
  std::copy(&X[0], &X[8], &x[0]);
  bineq_tmp = X[156];
  d = X[157];
  d1 = X[158];
  d2 = X[159];
  d3 = X[152];
  d4 = X[153];
  d5 = X[154];
  d6 = X[155];
  for (i = 0; i < 4; i++) {
    dv5[i] =
        ((B[i] * bineq_tmp + B[i + 4] * d) + B[i + 8] * d1) + B[i + 12] * d2;
    y[i] = ((A[i] * d3 + A[i + 4] * d4) + A[i + 8] * d5) + A[i + 12] * d6;
  }
  std::copy(&X[8], &X[160], &dv4[0]);
  dv4[152] = y[0] + dv5[0];
  dv4[156] = X[156];
  dv4[153] = y[1] + dv5[1];
  dv4[157] = X[157];
  dv4[154] = y[2] + dv5[2];
  dv4[158] = X[158];
  dv4[155] = y[3] + dv5[3];
  dv4[159] = X[159];
  //      Z = [X];
  for (b_i = 0; b_i < 160; b_i++) {
    bineq_tmp = dv4[b_i];
    X[b_i] = bineq_tmp;
    planned_traj[b_i] = bineq_tmp;
  }
}

void PositionMPCStepFunction_init()
{
  std::memset(&A[0], 0, 16U * sizeof(double));
  A[0] = 1.0;
  A[5] = 1.0;
  A[10] = 1.0;
  A[15] = 1.0;
  std::memset(&beq[0], 0, 80U * sizeof(double));
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

void bineq_not_empty_init()
{
  bineq_not_empty = false;
}

void f_not_empty_init()
{
  f_not_empty = false;
}

} // namespace pos_MPC

// End of code generation (PositionMPCStepFunction.cpp)
