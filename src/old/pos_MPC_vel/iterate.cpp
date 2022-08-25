//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// iterate.cpp
//
// Code generation for function 'iterate'
//

// Include files
#include "iterate.h"
#include "PositionMPCStepFunction_internal_types.h"
#include "PositionMPCStepFunction_rtwutil.h"
#include "addBoundToActiveSetMatrix_.h"
#include "checkStoppingAndUpdateFval.h"
#include "computeFval_ReuseHx.h"
#include "computeGrad_StoreHx.h"
#include "computeQ_.h"
#include "compute_deltax.h"
#include "deleteColMoveEnd.h"
#include "factorQR.h"
#include "feasibleratiotest.h"
#include "ratiotest.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "xrotg.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void iterate(const double H[25600], const double f[160], struct_T *solution,
             f_struct_T *memspace, d_struct_T *workingset,
             g_struct_T *qrmanager, c_struct_T *cholmanager,
             b_struct_T *objective, double options_ObjectiveLimit,
             double options_StepTolerance,
             double runTimeOptions_ConstrRelTolFactor,
             double runTimeOptions_ProbRelTolFactor,
             boolean_T runTimeOptions_RemainFeasible)
{
  double c;
  double minLambda;
  double s;
  double temp_tmp;
  double tolDelta;
  int TYPE;
  int activeSetChangeID;
  int globalActiveConstrIdx;
  int iyend;
  int localActiveConstrIdx;
  int nVar_tmp_tmp;
  boolean_T newBlocking;
  boolean_T subProblemChanged;
  boolean_T updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective->objtype;
  tolDelta = 6.7434957617430445E-7;
  nVar_tmp_tmp = workingset->nVar;
  globalActiveConstrIdx = 0;
  Objective::computeGrad_StoreHx(objective, H, f, solution->xstar);
  solution->fstar = Objective::computeFval_ReuseHx(
      objective, memspace->workspace_double, f, solution->xstar);
  if (solution->iterations < 35) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }
  std::memset(&solution->lambda[0], 0, 561U * sizeof(double));
  int exitg1;
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      int i;
      int iAineq0;
      int iac;
      int idx;
      boolean_T guard1{false};
      guard1 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
        case 1:
          localActiveConstrIdx = 161 * (workingset->nActiveConstr - 1);
          iyend = qrmanager->mrows;
          iAineq0 = qrmanager->ncols + 1;
          if (iyend < iAineq0) {
            iAineq0 = iyend;
          }
          qrmanager->minRowCol = iAineq0;
          iAineq0 = 240 * qrmanager->ncols;
          if (qrmanager->mrows != 0) {
            iyend = iAineq0 + qrmanager->mrows;
            if (iAineq0 + 1 <= iyend) {
              std::memset(&qrmanager->QR[iAineq0], 0,
                          (iyend - iAineq0) * sizeof(double));
            }
            i = 240 * (qrmanager->mrows - 1) + 1;
            for (iac = 1; iac <= i; iac += 240) {
              c = 0.0;
              iyend = (iac + qrmanager->mrows) - 1;
              for (idx = iac; idx <= iyend; idx++) {
                c += qrmanager->Q[idx - 1] *
                     workingset->ATwset[(localActiveConstrIdx + idx) - iac];
              }
              iyend = iAineq0 + div_nde_s32_floor(iac - 1, 240);
              qrmanager->QR[iyend] += c;
            }
          }
          qrmanager->ncols++;
          qrmanager->jpvt[qrmanager->ncols - 1] = qrmanager->ncols;
          for (idx = qrmanager->mrows - 2; idx + 2 > qrmanager->ncols; idx--) {
            iyend = 240 * (qrmanager->ncols - 1);
            i = (idx + iyend) + 1;
            temp_tmp = qrmanager->QR[i];
            internal::blas::xrotg(&qrmanager->QR[idx + iyend], &temp_tmp, &c,
                                  &s);
            qrmanager->QR[i] = temp_tmp;
            iyend = 240 * idx;
            iAineq0 = qrmanager->mrows;
            if (qrmanager->mrows >= 1) {
              for (iac = 0; iac < iAineq0; iac++) {
                localActiveConstrIdx = iyend + iac;
                minLambda = qrmanager->Q[localActiveConstrIdx + 240];
                temp_tmp = qrmanager->Q[localActiveConstrIdx];
                qrmanager->Q[localActiveConstrIdx + 240] =
                    c * minLambda - s * temp_tmp;
                qrmanager->Q[localActiveConstrIdx] =
                    c * temp_tmp + s * minLambda;
              }
            }
          }
          break;
        case -1:
          QRManager::deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;
        default:
          QRManager::factorQR(qrmanager, workingset->ATwset, nVar_tmp_tmp,
                              workingset->nActiveConstr);
          QRManager::computeQ_(qrmanager, qrmanager->mrows);
          break;
        }
        compute_deltax(H, solution, memspace, qrmanager, cholmanager,
                       objective);
        if (solution->state != -5) {
          exitg1 = 1;
        } else if ((internal::blas::xnrm2(nVar_tmp_tmp, solution->searchDir) <
                    options_StepTolerance) ||
                   (workingset->nActiveConstr >= nVar_tmp_tmp)) {
          guard1 = true;
        } else {
          updateFval = (TYPE == 5);
          if (updateFval || runTimeOptions_RemainFeasible) {
            feasibleratiotest(
                solution->xstar, solution->searchDir,
                memspace->workspace_double, workingset->nVar, workingset->Aineq,
                workingset->bineq, workingset->lb, workingset->ub,
                workingset->indexLB, workingset->indexUB, workingset->sizes,
                workingset->isActiveIdx, workingset->isActiveConstr,
                workingset->nWConstr, updateFval, &minLambda, &newBlocking,
                &iyend, &localActiveConstrIdx);
          } else {
            ratiotest(solution->xstar, solution->searchDir,
                      memspace->workspace_double, workingset->nVar,
                      workingset->Aineq, workingset->bineq, workingset->lb,
                      workingset->ub, workingset->indexLB, workingset->indexUB,
                      workingset->sizes, workingset->isActiveIdx,
                      workingset->isActiveConstr, workingset->nWConstr,
                      &tolDelta, &minLambda, &newBlocking, &iyend,
                      &localActiveConstrIdx);
          }
          if (newBlocking) {
            switch (iyend) {
            case 3:
              workingset->nWConstr[2]++;
              workingset->isActiveConstr[(workingset->isActiveIdx[2] +
                                          localActiveConstrIdx) -
                                         2] = true;
              workingset->nActiveConstr++;
              workingset->Wid[workingset->nActiveConstr - 1] = 3;
              workingset->Wlocalidx[workingset->nActiveConstr - 1] =
                  localActiveConstrIdx;
              iAineq0 = 161 * (localActiveConstrIdx - 1);
              iyend = 161 * (workingset->nActiveConstr - 1);
              i = workingset->nVar - 1;
              for (idx = 0; idx <= i; idx++) {
                workingset->ATwset[iyend + idx] =
                    workingset->Aineq[iAineq0 + idx];
              }
              workingset->bwset[workingset->nActiveConstr - 1] =
                  workingset->bineq[localActiveConstrIdx - 1];
              break;
            case 4:
              WorkingSet::addBoundToActiveSetMatrix_(workingset, 4,
                                                     localActiveConstrIdx);
              break;
            default:
              WorkingSet::addBoundToActiveSetMatrix_(workingset, 5,
                                                     localActiveConstrIdx);
              break;
            }
            activeSetChangeID = 1;
          } else {
            if (objective->objtype == 5) {
              if (internal::blas::xnrm2(objective->nvar, solution->searchDir) >
                  100.0 * static_cast<double>(objective->nvar) *
                      1.4901161193847656E-8) {
                solution->state = 3;
              } else {
                solution->state = 4;
              }
            }
            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }
          if (!(minLambda == 0.0)) {
            iyend = nVar_tmp_tmp - 1;
            for (iac = 0; iac <= iyend; iac++) {
              solution->xstar[iac] += minLambda * solution->searchDir[iac];
            }
          }
          Objective::computeGrad_StoreHx(objective, H, f, solution->xstar);
          updateFval = true;
          stopping::checkStoppingAndUpdateFval(
              &activeSetChangeID, f, solution, memspace, objective, workingset,
              qrmanager, options_ObjectiveLimit,
              runTimeOptions_ConstrRelTolFactor, updateFval);
        }
      } else {
        if (0 <= nVar_tmp_tmp - 1) {
          std::memset(&solution->searchDir[0], 0,
                      nVar_tmp_tmp * sizeof(double));
        }
        guard1 = true;
      }
      if (guard1) {
        int nActiveConstr;
        nActiveConstr = qrmanager->ncols;
        if (qrmanager->ncols > 0) {
          minLambda = 100.0 * static_cast<double>(qrmanager->mrows) *
                      2.2204460492503131E-16;
          if ((qrmanager->mrows > 0) && (qrmanager->ncols > 0)) {
            updateFval = true;
          } else {
            updateFval = false;
          }
          if (updateFval) {
            boolean_T b_guard1{false};
            idx = qrmanager->ncols;
            b_guard1 = false;
            if (qrmanager->mrows < qrmanager->ncols) {
              iAineq0 = qrmanager->mrows + 240 * (qrmanager->ncols - 1);
              while ((idx > qrmanager->mrows) &&
                     (std::abs(qrmanager->QR[iAineq0 - 1]) >= minLambda)) {
                idx--;
                iAineq0 -= 240;
              }
              updateFval = (idx == qrmanager->mrows);
              if (updateFval) {
                b_guard1 = true;
              }
            } else {
              b_guard1 = true;
            }
            if (b_guard1) {
              iAineq0 = idx + 240 * (idx - 1);
              while ((idx >= 1) &&
                     (std::abs(qrmanager->QR[iAineq0 - 1]) >= minLambda)) {
                idx--;
                iAineq0 -= 241;
              }
              updateFval = (idx == 0);
            }
          }
          if (!updateFval) {
            solution->state = -7;
          } else {
            if (qrmanager->mrows != 0) {
              iyend = qrmanager->ncols;
              if (0 <= iyend - 1) {
                std::memset(&memspace->workspace_double[0], 0,
                            iyend * sizeof(double));
              }
              i = 240 * (qrmanager->ncols - 1) + 1;
              for (iac = 1; iac <= i; iac += 240) {
                c = 0.0;
                iyend = (iac + qrmanager->mrows) - 1;
                for (idx = iac; idx <= iyend; idx++) {
                  c += qrmanager->Q[idx - 1] * objective->grad[idx - iac];
                }
                iyend = div_nde_s32_floor(iac - 1, 240);
                memspace->workspace_double[iyend] += c;
              }
            }
            for (localActiveConstrIdx = nActiveConstr;
                 localActiveConstrIdx >= 1; localActiveConstrIdx--) {
              iyend =
                  (localActiveConstrIdx + (localActiveConstrIdx - 1) * 240) - 1;
              memspace->workspace_double[localActiveConstrIdx - 1] /=
                  qrmanager->QR[iyend];
              for (iac = 0; iac <= localActiveConstrIdx - 2; iac++) {
                iAineq0 = (localActiveConstrIdx - iac) - 2;
                memspace->workspace_double[iAineq0] -=
                    memspace->workspace_double[localActiveConstrIdx - 1] *
                    qrmanager->QR[(iyend - iac) - 1];
              }
            }
            for (idx = 0; idx < nActiveConstr; idx++) {
              solution->lambda[idx] = -memspace->workspace_double[idx];
            }
          }
        }
        if ((solution->state != -7) ||
            (workingset->nActiveConstr > nVar_tmp_tmp)) {
          localActiveConstrIdx = -1;
          minLambda = 0.0 * runTimeOptions_ProbRelTolFactor *
                      static_cast<double>(TYPE != 5);
          i = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
          iyend = workingset->nActiveConstr;
          for (idx = i; idx <= iyend; idx++) {
            temp_tmp = solution->lambda[idx - 1];
            if (temp_tmp < minLambda) {
              minLambda = temp_tmp;
              localActiveConstrIdx = idx - 1;
            }
          }
          if (localActiveConstrIdx + 1 == 0) {
            solution->state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = localActiveConstrIdx + 1;
            subProblemChanged = true;
            iyend = workingset->Wid[localActiveConstrIdx] - 1;
            workingset->isActiveConstr
                [(workingset
                      ->isActiveIdx[workingset->Wid[localActiveConstrIdx] - 1] +
                  workingset->Wlocalidx[localActiveConstrIdx]) -
                 2] = false;
            workingset->Wid[localActiveConstrIdx] =
                workingset->Wid[workingset->nActiveConstr - 1];
            workingset->Wlocalidx[localActiveConstrIdx] =
                workingset->Wlocalidx[workingset->nActiveConstr - 1];
            i = workingset->nVar;
            for (idx = 0; idx < i; idx++) {
              workingset->ATwset[idx + 161 * localActiveConstrIdx] =
                  workingset
                      ->ATwset[idx + 161 * (workingset->nActiveConstr - 1)];
            }
            workingset->bwset[localActiveConstrIdx] =
                workingset->bwset[workingset->nActiveConstr - 1];
            workingset->nActiveConstr--;
            workingset->nWConstr[iyend]--;
            solution->lambda[localActiveConstrIdx] = 0.0;
          }
        } else {
          localActiveConstrIdx = workingset->nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset->nActiveConstr;
          subProblemChanged = true;
          iyend = workingset->nActiveConstr - 1;
          iAineq0 = workingset->Wid[iyend] - 1;
          workingset->isActiveConstr[(workingset->isActiveIdx[iAineq0] +
                                      workingset->Wlocalidx[iyend]) -
                                     2] = false;
          workingset->nActiveConstr--;
          workingset->nWConstr[iAineq0]--;
          solution->lambda[localActiveConstrIdx - 1] = 0.0;
        }
        updateFval = false;
        stopping::checkStoppingAndUpdateFval(
            &activeSetChangeID, f, solution, memspace, objective, workingset,
            qrmanager, options_ObjectiveLimit,
            runTimeOptions_ConstrRelTolFactor, updateFval);
      }
    } else {
      if (!updateFval) {
        solution->fstar = Objective::computeFval_ReuseHx(
            objective, memspace->workspace_double, f, solution->xstar);
      }
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (iterate.cpp)
