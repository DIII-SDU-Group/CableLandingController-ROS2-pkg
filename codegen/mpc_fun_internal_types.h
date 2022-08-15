//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mpc_fun_internal_types.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

#ifndef MPC_FUN_INTERNAL_TYPES_H
#define MPC_FUN_INTERNAL_TYPES_H

// Include Files
#include "mpc_fun_types.h"
#include "rtwtypes.h"
#include "coder_array.h"

// Type Definitions
struct struct_T {
  int MaxIterations;
  double ConstrRelTolFactor;
  double ProbRelTolFactor;
  boolean_T RemainFeasible;
};

struct b_struct_T {
  coder::array<double, 1U> xstar;
  double fstar;
  double firstorderopt;
  coder::array<double, 1U> lambda;
  int state;
  double maxConstr;
  int iterations;
  coder::array<double, 1U> searchDir;
};

struct c_struct_T {
  coder::array<double, 1U> grad;
  coder::array<double, 1U> Hx;
  boolean_T hasLinear;
  int nvar;
  int maxVar;
  double beta;
  double rho;
  int objtype;
  int prev_objtype;
  int prev_nvar;
  boolean_T prev_hasLinear;
  double gammaScalar;
};

struct d_struct_T {
  int ldq;
  coder::array<double, 2U> QR;
  coder::array<double, 2U> Q;
  coder::array<int, 1U> jpvt;
  int mrows;
  int ncols;
  coder::array<double, 1U> tau;
  int minRowCol;
  boolean_T usedPivoting;
};

struct e_struct_T {
  coder::array<double, 1U> FMat;
  int ldm;
  int ndims;
  int info;
  double scaleFactor;
  boolean_T ConvexCheck;
  double regTol_;
  coder::array<double, 1U> workspace_;
  coder::array<double, 1U> workspace2_;
};

struct f_struct_T {
  int mConstr;
  int mConstrOrig;
  int mConstrMax;
  int nVar;
  int nVarOrig;
  int nVarMax;
  int ldA;
  coder::array<double, 1U> Aineq;
  coder::array<double, 1U> bineq;
  coder::array<double, 1U> Aeq;
  coder::array<double, 1U> beq;
  coder::array<double, 1U> lb;
  coder::array<double, 1U> ub;
  coder::array<int, 1U> indexLB;
  coder::array<int, 1U> indexUB;
  coder::array<int, 1U> indexFixed;
  int mEqRemoved;
  coder::array<int, 1U> indexEqRemoved;
  coder::array<double, 1U> ATwset;
  coder::array<double, 1U> bwset;
  int nActiveConstr;
  coder::array<double, 1U> maxConstrWorkspace;
  int sizes[5];
  int sizesNormal[5];
  int sizesPhaseOne[5];
  int sizesRegularized[5];
  int sizesRegPhaseOne[5];
  int isActiveIdx[6];
  int isActiveIdxNormal[6];
  int isActiveIdxPhaseOne[6];
  int isActiveIdxRegularized[6];
  int isActiveIdxRegPhaseOne[6];
  coder::array<boolean_T, 1U> isActiveConstr;
  coder::array<int, 1U> Wid;
  coder::array<int, 1U> Wlocalidx;
  int nWConstr[5];
  int probType;
  double SLACK0;
};

struct g_struct_T {
  coder::array<double, 2U> workspace_double;
  coder::array<int, 1U> workspace_int;
  coder::array<int, 1U> workspace_sort;
};

struct h_struct_T {
  double InitDamping;
  char FiniteDifferenceType[7];
  boolean_T SpecifyObjectiveGradient;
  boolean_T ScaleProblem;
  boolean_T SpecifyConstraintGradient;
  boolean_T NonFiniteSupport;
  boolean_T IterDisplaySQP;
  double FiniteDifferenceStepSize;
  double MaxFunctionEvaluations;
  boolean_T IterDisplayQP;
  double PricingTolerance;
  char Algorithm[10];
  double ObjectiveLimit;
  double ConstraintTolerance;
  double OptimalityTolerance;
  double StepTolerance;
  double MaxIterations;
  double FunctionTolerance;
  char SolverName[8];
  boolean_T CheckGradients;
  char Diagnostics[3];
  double DiffMaxChange;
  double DiffMinChange;
  char Display[5];
  char FunValCheck[3];
  boolean_T UseParallel;
  char LinearSolver[4];
  char SubproblemAlgorithm[2];
};

#endif
//
// File trailer for mpc_fun_internal_types.h
//
// [EOF]
//
