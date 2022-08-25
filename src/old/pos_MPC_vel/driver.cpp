//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// driver.cpp
//
// Code generation for function 'driver'
//

// Include files
#include "driver.h"
#include "PositionMPCStepFunction_data.h"
#include "PositionMPCStepFunction_internal_types.h"
#include "PresolveWorkingSet.h"
#include "computeFirstOrderOpt.h"
#include "computeFval.h"
#include "feasibleX0ForWorkingSet.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "phaseone.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cstring>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void driver(const double H[25600], const double f[160], struct_T *solution,
            f_struct_T *memspace, d_struct_T *workingset, g_struct_T *qrmanager,
            c_struct_T *cholmanager, e_struct_T runTimeOptions,
            b_struct_T *objective)
{
  static const char b_cv[128]{
      '\x00', '\x01', '\x02', '\x03', '\x04', '\x05', '\x06', '\x07', '\x08',
      '\x09', '\x0a', '\x0b', '\x0c', '\x0d', '\x0e', '\x0f', '\x10', '\x11',
      '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18', '\x19', '\x1a',
      '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ',    '!',    '\"',   '#',
      '$',    '%',    '&',    '\'',   '(',    ')',    '*',    '+',    ',',
      '-',    '.',    '/',    '0',    '1',    '2',    '3',    '4',    '5',
      '6',    '7',    '8',    '9',    ':',    ';',    '<',    '=',    '>',
      '?',    '@',    'a',    'b',    'c',    'd',    'e',    'f',    'g',
      'h',    'i',    'j',    'k',    'l',    'm',    'n',    'o',    'p',
      'q',    'r',    's',    't',    'u',    'v',    'w',    'x',    'y',
      'z',    '[',    '\\',   ']',    '^',    '_',    '`',    'a',    'b',
      'c',    'd',    'e',    'f',    'g',    'h',    'i',    'j',    'k',
      'l',    'm',    'n',    'o',    'p',    'q',    'r',    's',    't',
      'u',    'v',    'w',    'x',    'y',    'z',    '{',    '|',    '}',
      '~',    '\x7f'};
  static const char cv1[8]{'q', 'u', 'a', 'd', 'p', 'r', 'o', 'g'};
  static const char t1_SolverName[8]{'q', 'u', 'a', 'd', 'p', 'r', 'o', 'g'};
  static const char t1_FiniteDifferenceType[7]{'f', 'o', 'r', 'w',
                                               'a', 'r', 'd'};
  static const char t1_Display[5]{'f', 'i', 'n', 'a', 'l'};
  h_struct_T options;
  int i;
  int idx;
  int nVar;
  std::memset(&objective->grad[0], 0, 161U * sizeof(double));
  std::memset(&objective->Hx[0], 0, 160U * sizeof(double));
  objective->hasLinear = true;
  objective->nvar = 160;
  objective->maxVar = 161;
  objective->beta = 0.0;
  objective->rho = 0.0;
  objective->objtype = 3;
  objective->prev_objtype = 3;
  objective->prev_nvar = 0;
  objective->prev_hasLinear = false;
  objective->gammaScalar = 0.0;
  solution->iterations = 0;
  runTimeOptions.RemainFeasible = true;
  nVar = workingset->nVar - 1;
  i = workingset->sizes[0];
  for (idx = 0; idx < i; idx++) {
    solution->xstar[workingset->indexFixed[idx] - 1] =
        workingset->ub[workingset->indexFixed[idx] - 1];
  }
  i = workingset->sizes[3];
  for (idx = 0; idx < i; idx++) {
    if (workingset->isActiveConstr[(workingset->isActiveIdx[3] + idx) - 1]) {
      solution->xstar[workingset->indexLB[idx] - 1] =
          -workingset->lb[workingset->indexLB[idx] - 1];
    }
  }
  i = workingset->sizes[4];
  for (idx = 0; idx < i; idx++) {
    if (workingset->isActiveConstr[(workingset->isActiveIdx[4] + idx) - 1]) {
      solution->xstar[workingset->indexUB[idx] - 1] =
          workingset->ub[workingset->indexUB[idx] - 1];
    }
  }
  initialize::PresolveWorkingSet(solution, memspace, workingset, qrmanager);
  options.InitDamping = 0.01;
  for (i = 0; i < 7; i++) {
    options.FiniteDifferenceType[i] = t1_FiniteDifferenceType[i];
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
    options.Algorithm[i] = cv[i];
  }
  options.ObjectiveLimit = -1.0E+20;
  options.ConstraintTolerance = 1.0E-8;
  options.OptimalityTolerance = 0.999;
  options.StepTolerance = 0.001;
  options.MaxIterations = 35.0;
  options.FunctionTolerance = rtInf;
  for (i = 0; i < 8; i++) {
    options.SolverName[i] = t1_SolverName[i];
  }
  options.CheckGradients = false;
  options.Diagnostics[0] = 'o';
  options.Diagnostics[1] = 'f';
  options.Diagnostics[2] = 'f';
  options.DiffMaxChange = rtInf;
  options.DiffMinChange = 0.0;
  for (i = 0; i < 5; i++) {
    options.Display[i] = t1_Display[i];
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
  if (solution->state >= 0) {
    double maxConstr_new;
    boolean_T guard1{false};
    solution->iterations = 0;
    solution->maxConstr =
        WorkingSet::b_maxConstraintViolation(workingset, solution->xstar);
    maxConstr_new = 1.0E-8 * runTimeOptions.ConstrRelTolFactor;
    guard1 = false;
    if (solution->maxConstr > maxConstr_new) {
      phaseone(H, f, solution, memspace, workingset, qrmanager, cholmanager,
               &runTimeOptions, objective, &options);
      if (solution->state != 0) {
        solution->maxConstr =
            WorkingSet::b_maxConstraintViolation(workingset, solution->xstar);
        if (solution->maxConstr > maxConstr_new) {
          std::memset(&solution->lambda[0], 0, 561U * sizeof(double));
          solution->fstar = Objective::computeFval(
              objective, memspace->workspace_double, H, f, solution->xstar);
          solution->state = -2;
        } else {
          if (solution->maxConstr > 0.0) {
            if (0 <= nVar) {
              std::copy(&solution->xstar[0], &solution->xstar[nVar + 1],
                        &solution->searchDir[0]);
            }
            initialize::PresolveWorkingSet(solution, memspace, workingset,
                                           qrmanager);
            maxConstr_new = WorkingSet::b_maxConstraintViolation(
                workingset, solution->xstar);
            if (maxConstr_new >= solution->maxConstr) {
              solution->maxConstr = maxConstr_new;
              if (0 <= nVar) {
                std::copy(&solution->searchDir[0],
                          &solution->searchDir[nVar + 1], &solution->xstar[0]);
              }
            }
          }
          guard1 = true;
        }
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      boolean_T b_bool;
      iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, options.ObjectiveLimit, options.StepTolerance,
              runTimeOptions.ConstrRelTolFactor,
              runTimeOptions.ProbRelTolFactor, true);
      b_bool = false;
      nVar = 0;
      int exitg1;
      do {
        exitg1 = 0;
        if (nVar < 8) {
          if (b_cv[static_cast<unsigned char>(options.SolverName[nVar])] !=
              b_cv[static_cast<int>(cv1[nVar])]) {
            exitg1 = 1;
          } else {
            nVar++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
      if (b_bool && (solution->state != -6)) {
        solution->maxConstr =
            WorkingSet::b_maxConstraintViolation(workingset, solution->xstar);
        parseoutput::computeFirstOrderOpt(
            solution, objective, workingset->nVar, workingset->ATwset,
            workingset->nActiveConstr, memspace->workspace_double);
        runTimeOptions.RemainFeasible = false;
        while ((solution->iterations < 35) &&
               ((solution->state == -7) ||
                ((solution->state == 1) &&
                 ((solution->maxConstr >
                   1.0E-8 * runTimeOptions.ConstrRelTolFactor) ||
                  (solution->firstorderopt >
                   0.999 * runTimeOptions.ProbRelTolFactor))))) {
          initialize::feasibleX0ForWorkingSet(memspace->workspace_double,
                                              solution->xstar, workingset,
                                              qrmanager);
          initialize::PresolveWorkingSet(solution, memspace, workingset,
                                         qrmanager);
          phaseone(H, f, solution, memspace, workingset, qrmanager, cholmanager,
                   objective, &options, &runTimeOptions);
          iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
                  objective, options.ObjectiveLimit, options.StepTolerance,
                  runTimeOptions.ConstrRelTolFactor,
                  runTimeOptions.ProbRelTolFactor, false);
          solution->maxConstr =
              WorkingSet::b_maxConstraintViolation(workingset, solution->xstar);
          parseoutput::computeFirstOrderOpt(
              solution, objective, workingset->nVar, workingset->ATwset,
              workingset->nActiveConstr, memspace->workspace_double);
        }
      }
    }
  }
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (driver.cpp)
