//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// MPCStepFunction_internal_types.h
//
// Code generation for function 'MPCStepFunction'
//

#ifndef MPCSTEPFUNCTION_INTERNAL_TYPES_H
#define MPCSTEPFUNCTION_INTERNAL_TYPES_H

// Include files
#include "MPCStepFunction_types.h"
#include "rtwtypes.h"

// Type Definitions
namespace pos_MPC {
struct struct_T {
  double xstar[181];
  double fstar;
  double firstorderopt;
  double lambda[481];
  int state;
  double maxConstr;
  int iterations;
  double searchDir[181];
};

struct b_struct_T {
  double grad[181];
  double Hx[180];
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

struct c_struct_T {
  double FMat[90000];
  int ldm;
  int ndims;
  int info;
  double scaleFactor;
  boolean_T ConvexCheck;
  double regTol_;
  double workspace_[14400];
  double workspace2_[14400];
};

struct d_struct_T {
  int MaxIterations;
  double ConstrRelTolFactor;
  double ProbRelTolFactor;
  boolean_T RemainFeasible;
};

struct e_struct_T {
  double workspace_double[87061];
  int workspace_int[481];
  int workspace_sort[481];
};

struct f_struct_T {
  int ldq;
  double QR[144300];
  double Q[90000];
  int jpvt[481];
  int mrows;
  int ncols;
  double tau[300];
  int minRowCol;
  boolean_T usedPivoting;
};

struct g_struct_T {
  int mConstr;
  int mConstrOrig;
  int mConstrMax;
  int nVar;
  int nVarOrig;
  int nVarMax;
  int ldA;
  double Aeq[21720];
  double beq[120];
  double lb[181];
  double ub[181];
  int indexLB[181];
  int indexUB[181];
  int indexFixed[181];
  int mEqRemoved;
  int indexEqRemoved[120];
  double ATwset[87061];
  double bwset[481];
  int nActiveConstr;
  double maxConstrWorkspace[481];
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
  boolean_T isActiveConstr[481];
  int Wid[481];
  int Wlocalidx[481];
  int nWConstr[5];
  int probType;
  double SLACK0;
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

} // namespace pos_MPC

#endif
// End of code generation (MPCStepFunction_internal_types.h)
