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
#include "MPCStepFunction_internal_types.h"
#include "MPCStepFunction_rtwutil.h"
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
void iterate(const double H[14400], const double f[120], struct_T *solution,
             e_struct_T *memspace, g_struct_T *workingset,
             f_struct_T *qrmanager, c_struct_T *cholmanager,
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
  int idxRotGCol;
  int iyend;
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
  std::memset(&solution->lambda[0], 0, 321U * sizeof(double));
  int exitg1;
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      int ia;
      int idx;
      int idxMinLambda;
      int k;
      boolean_T guard1{false};
      guard1 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
        case 1:
          idxMinLambda = 121 * (workingset->nActiveConstr - 1);
          idxRotGCol = qrmanager->mrows;
          iyend = qrmanager->ncols + 1;
          if (idxRotGCol < iyend) {
            iyend = idxRotGCol;
          }
          qrmanager->minRowCol = iyend;
          idxRotGCol = 200 * qrmanager->ncols;
          if (qrmanager->mrows != 0) {
            iyend = idxRotGCol + qrmanager->mrows;
            if (idxRotGCol + 1 <= iyend) {
              std::memset(&qrmanager->QR[idxRotGCol], 0,
                          (iyend - idxRotGCol) * sizeof(double));
            }
            k = 200 * (qrmanager->mrows - 1) + 1;
            for (idx = 1; idx <= k; idx += 200) {
              c = 0.0;
              iyend = (idx + qrmanager->mrows) - 1;
              for (ia = idx; ia <= iyend; ia++) {
                c += qrmanager->Q[ia - 1] *
                     workingset->ATwset[(idxMinLambda + ia) - idx];
              }
              iyend = idxRotGCol + div_nde_s32_floor(idx - 1, 200);
              qrmanager->QR[iyend] += c;
            }
          }
          qrmanager->ncols++;
          qrmanager->jpvt[qrmanager->ncols - 1] = qrmanager->ncols;
          for (idx = qrmanager->mrows - 2; idx + 2 > qrmanager->ncols; idx--) {
            idxRotGCol = 200 * (qrmanager->ncols - 1);
            k = (idx + idxRotGCol) + 1;
            temp_tmp = qrmanager->QR[k];
            internal::blas::xrotg(&qrmanager->QR[idx + idxRotGCol], &temp_tmp,
                                  &c, &s);
            qrmanager->QR[k] = temp_tmp;
            idxRotGCol = 200 * idx;
            iyend = qrmanager->mrows;
            if (qrmanager->mrows >= 1) {
              for (k = 0; k < iyend; k++) {
                idxMinLambda = idxRotGCol + k;
                minLambda = qrmanager->Q[idxMinLambda + 200];
                temp_tmp = qrmanager->Q[idxMinLambda];
                qrmanager->Q[idxMinLambda + 200] = c * minLambda - s * temp_tmp;
                qrmanager->Q[idxMinLambda] = c * temp_tmp + s * minLambda;
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
                solution->xstar, solution->searchDir, workingset->nVar,
                workingset->lb, workingset->ub, workingset->indexLB,
                workingset->indexUB, workingset->sizes, workingset->isActiveIdx,
                workingset->isActiveConstr, workingset->nWConstr, updateFval,
                &minLambda, &newBlocking, &idxRotGCol, &iyend);
          } else {
            ratiotest(solution->xstar, solution->searchDir, workingset->nVar,
                      workingset->lb, workingset->ub, workingset->indexLB,
                      workingset->indexUB, workingset->sizes,
                      workingset->isActiveIdx, workingset->isActiveConstr,
                      workingset->nWConstr, &tolDelta, &minLambda, &newBlocking,
                      &idxRotGCol, &iyend);
          }
          if (newBlocking) {
            switch (idxRotGCol) {
            case 3:
              workingset->nWConstr[2]++;
              workingset
                  ->isActiveConstr[(workingset->isActiveIdx[2] + iyend) - 2] =
                  true;
              workingset->nActiveConstr++;
              workingset->Wid[workingset->nActiveConstr - 1] = 3;
              workingset->Wlocalidx[workingset->nActiveConstr - 1] = iyend;
              // A check that is always false is detected at compile-time.
              // Eliminating code that follows.
              break;
            case 4:
              WorkingSet::addBoundToActiveSetMatrix_(workingset, 4, iyend);
              break;
            default:
              WorkingSet::addBoundToActiveSetMatrix_(workingset, 5, iyend);
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
            idxRotGCol = nVar_tmp_tmp - 1;
            for (k = 0; k <= idxRotGCol; k++) {
              solution->xstar[k] += minLambda * solution->searchDir[k];
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
              iyend = qrmanager->mrows + 200 * (qrmanager->ncols - 1);
              while ((idx > qrmanager->mrows) &&
                     (std::abs(qrmanager->QR[iyend - 1]) >= minLambda)) {
                idx--;
                iyend -= 200;
              }
              updateFval = (idx == qrmanager->mrows);
              if (updateFval) {
                b_guard1 = true;
              }
            } else {
              b_guard1 = true;
            }
            if (b_guard1) {
              iyend = idx + 200 * (idx - 1);
              while ((idx >= 1) &&
                     (std::abs(qrmanager->QR[iyend - 1]) >= minLambda)) {
                idx--;
                iyend -= 201;
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
              k = 200 * (qrmanager->ncols - 1) + 1;
              for (idx = 1; idx <= k; idx += 200) {
                c = 0.0;
                iyend = (idx + qrmanager->mrows) - 1;
                for (ia = idx; ia <= iyend; ia++) {
                  c += qrmanager->Q[ia - 1] * objective->grad[ia - idx];
                }
                iyend = div_nde_s32_floor(idx - 1, 200);
                memspace->workspace_double[iyend] += c;
              }
            }
            for (idxMinLambda = nActiveConstr; idxMinLambda >= 1;
                 idxMinLambda--) {
              idxRotGCol = (idxMinLambda + (idxMinLambda - 1) * 200) - 1;
              memspace->workspace_double[idxMinLambda - 1] /=
                  qrmanager->QR[idxRotGCol];
              for (k = 0; k <= idxMinLambda - 2; k++) {
                iyend = (idxMinLambda - k) - 2;
                memspace->workspace_double[iyend] -=
                    memspace->workspace_double[idxMinLambda - 1] *
                    qrmanager->QR[(idxRotGCol - k) - 1];
              }
            }
            for (idx = 0; idx < nActiveConstr; idx++) {
              solution->lambda[idx] = -memspace->workspace_double[idx];
            }
          }
        }
        if ((solution->state != -7) ||
            (workingset->nActiveConstr > nVar_tmp_tmp)) {
          idxMinLambda = -1;
          minLambda = 0.0 * runTimeOptions_ProbRelTolFactor *
                      static_cast<double>(TYPE != 5);
          k = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
          iyend = workingset->nActiveConstr;
          for (idx = k; idx <= iyend; idx++) {
            temp_tmp = solution->lambda[idx - 1];
            if (temp_tmp < minLambda) {
              minLambda = temp_tmp;
              idxMinLambda = idx - 1;
            }
          }
          if (idxMinLambda + 1 == 0) {
            solution->state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = idxMinLambda + 1;
            subProblemChanged = true;
            idxRotGCol = workingset->Wid[idxMinLambda] - 1;
            workingset->isActiveConstr
                [(workingset->isActiveIdx[workingset->Wid[idxMinLambda] - 1] +
                  workingset->Wlocalidx[idxMinLambda]) -
                 2] = false;
            workingset->Wid[idxMinLambda] =
                workingset->Wid[workingset->nActiveConstr - 1];
            workingset->Wlocalidx[idxMinLambda] =
                workingset->Wlocalidx[workingset->nActiveConstr - 1];
            k = workingset->nVar;
            for (idx = 0; idx < k; idx++) {
              workingset->ATwset[idx + 121 * idxMinLambda] =
                  workingset
                      ->ATwset[idx + 121 * (workingset->nActiveConstr - 1)];
            }
            workingset->bwset[idxMinLambda] =
                workingset->bwset[workingset->nActiveConstr - 1];
            workingset->nActiveConstr--;
            workingset->nWConstr[idxRotGCol]--;
            solution->lambda[idxMinLambda] = 0.0;
          }
        } else {
          idxMinLambda = workingset->nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset->nActiveConstr;
          subProblemChanged = true;
          iyend = workingset->nActiveConstr - 1;
          idxRotGCol = workingset->Wid[iyend] - 1;
          workingset->isActiveConstr[(workingset->isActiveIdx[idxRotGCol] +
                                      workingset->Wlocalidx[iyend]) -
                                     2] = false;
          workingset->nActiveConstr--;
          workingset->nWConstr[idxRotGCol]--;
          solution->lambda[idxMinLambda - 1] = 0.0;
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
