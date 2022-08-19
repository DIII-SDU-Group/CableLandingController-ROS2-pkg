//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// PositionMPCStepFunction_internal_types.h
//
// Code generation for function 'PositionMPCStepFunction'
//

#ifndef POSITIONMPCSTEPFUNCTION_INTERNAL_TYPES_H
#define POSITIONMPCSTEPFUNCTION_INTERNAL_TYPES_H

// Include files
#include "PositionMPCStepFunction_types.h"
#include "rtwtypes.h"

// Type Definitions
namespace pos_MPC {
struct struct_T {
  double xstar[161];
  double fstar;
  double firstorderopt;
  double lambda[561];
  int state;
  double maxConstr;
  int iterations;
  double searchDir[161];
};

struct b_struct_T {
  double grad[161];
  double Hx[160];
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
  double FMat[57600];
  int ldm;
  int ndims;
  int info;
  double scaleFactor;
  boolean_T ConvexCheck;
  double regTol_;
  double workspace_[11520];
  double workspace2_[11520];
};

struct d_struct_T {
  int mConstr;
  int mConstrOrig;
  int mConstrMax;
  int nVar;
  int nVarOrig;
  int nVarMax;
  int ldA;
  double Aineq[25760];
  double bineq[160];
  double Aeq[12880];
  double beq[80];
  double lb[161];
  double ub[161];
  int indexLB[161];
  int indexUB[161];
  int indexFixed[161];
  int mEqRemoved;
  int indexEqRemoved[80];
  double ATwset[90321];
  double bwset[561];
  int nActiveConstr;
  double maxConstrWorkspace[561];
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
  boolean_T isActiveConstr[561];
  int Wid[561];
  int Wlocalidx[561];
  int nWConstr[5];
  int probType;
  double SLACK0;
};

struct e_struct_T {
  int MaxIterations;
  double ConstrRelTolFactor;
  double ProbRelTolFactor;
  boolean_T RemainFeasible;
};

struct f_struct_T {
  double workspace_double[90321];
  int workspace_int[561];
  int workspace_sort[561];
};

struct g_struct_T {
  int ldq;
  double QR[134640];
  double Q[57600];
  int jpvt[561];
  int mrows;
  int ncols;
  double tau[240];
  int minRowCol;
  boolean_T usedPivoting;
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
// End of code generation (PositionMPCStepFunction_internal_types.h)
