//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: PresolveWorkingSet.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "PresolveWorkingSet.h"
#include "computeQ_.h"
#include "countsort.h"
#include "factorQRE.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "mpc_fun_internal_types.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : b_struct_T *solution
//                g_struct_T *memspace
//                f_struct_T *workingset
//                d_struct_T *qrmanager
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
void PresolveWorkingSet(b_struct_T *solution, g_struct_T *memspace,
                        f_struct_T *workingset, d_struct_T *qrmanager)
{
  double tol;
  int i;
  int idx_col;
  int ix;
  int k;
  int mTotalWorkingEq_tmp_tmp;
  int mWorkingFixed;
  int nDepInd;
  int nVar;
  int totalRank;
  solution->state = 82;
  nVar = workingset->nVar - 1;
  mWorkingFixed = workingset->nWConstr[0];
  mTotalWorkingEq_tmp_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    for (totalRank = 0; totalRank < mTotalWorkingEq_tmp_tmp; totalRank++) {
      for (idx_col = 0; idx_col <= nVar; idx_col++) {
        qrmanager->QR[totalRank + qrmanager->ldq * idx_col] =
            workingset->ATwset[idx_col + workingset->ldA * totalRank];
      }
    }
    nDepInd = mTotalWorkingEq_tmp_tmp - workingset->nVar;
    if (0 > nDepInd) {
      nDepInd = 0;
    }
    for (idx_col = 0; idx_col <= nVar; idx_col++) {
      qrmanager->jpvt[idx_col] = 0;
    }
    QRManager::factorQRE(qrmanager, mTotalWorkingEq_tmp_tmp, workingset->nVar);
    tol =
        100.0 * static_cast<double>(workingset->nVar) * 2.2204460492503131E-16;
    totalRank = workingset->nVar;
    if (totalRank >= mTotalWorkingEq_tmp_tmp) {
      totalRank = mTotalWorkingEq_tmp_tmp;
    }
    totalRank += qrmanager->ldq * (totalRank - 1);
    while ((totalRank > 0) && (std::abs(qrmanager->QR[totalRank - 1]) < tol)) {
      totalRank = (totalRank - qrmanager->ldq) - 1;
      nDepInd++;
    }
    if (nDepInd > 0) {
      boolean_T exitg1;
      QRManager::computeQ_(qrmanager, qrmanager->mrows);
      idx_col = 0;
      exitg1 = false;
      while ((!exitg1) && (idx_col <= nDepInd - 1)) {
        double qtb;
        ix = qrmanager->ldq * ((mTotalWorkingEq_tmp_tmp - idx_col) - 1);
        qtb = 0.0;
        for (k = 0; k < mTotalWorkingEq_tmp_tmp; k++) {
          qtb += qrmanager->Q[ix + k] * workingset->bwset[k];
        }
        if (std::abs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          idx_col++;
        }
      }
    }
    if (nDepInd > 0) {
      for (idx_col = 0; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
        totalRank = qrmanager->ldq * idx_col;
        ix = workingset->ldA * idx_col;
        for (k = 0; k <= nVar; k++) {
          qrmanager->QR[totalRank + k] = workingset->ATwset[ix + k];
        }
      }
      for (idx_col = 0; idx_col < mWorkingFixed; idx_col++) {
        qrmanager->jpvt[idx_col] = 1;
      }
      i = workingset->nWConstr[0] + 1;
      for (idx_col = i; idx_col <= mTotalWorkingEq_tmp_tmp; idx_col++) {
        qrmanager->jpvt[idx_col - 1] = 0;
      }
      QRManager::factorQRE(qrmanager, workingset->nVar,
                           mTotalWorkingEq_tmp_tmp);
      for (idx_col = 0; idx_col < nDepInd; idx_col++) {
        memspace->workspace_int[idx_col] =
            qrmanager->jpvt[(mTotalWorkingEq_tmp_tmp - nDepInd) + idx_col];
      }
      utils::countsort(memspace->workspace_int, nDepInd,
                       memspace->workspace_sort, 1, mTotalWorkingEq_tmp_tmp);
      for (idx_col = nDepInd; idx_col >= 1; idx_col--) {
        i = workingset->nWConstr[0] + workingset->nWConstr[1];
        if (i != 0) {
          mTotalWorkingEq_tmp_tmp = memspace->workspace_int[idx_col - 1];
          if (mTotalWorkingEq_tmp_tmp <= i) {
            if ((workingset->nActiveConstr == i) ||
                (mTotalWorkingEq_tmp_tmp == i)) {
              workingset->mEqRemoved++;
              workingset->indexEqRemoved[workingset->mEqRemoved - 1] =
                  workingset
                      ->Wlocalidx[memspace->workspace_int[idx_col - 1] - 1];
              WorkingSet::removeConstr(workingset,
                                       memspace->workspace_int[idx_col - 1]);
            } else {
              workingset->mEqRemoved++;
              mWorkingFixed = workingset->Wid[mTotalWorkingEq_tmp_tmp - 1] - 1;
              ix = workingset->Wlocalidx[mTotalWorkingEq_tmp_tmp - 1];
              workingset->indexEqRemoved[workingset->mEqRemoved - 1] = ix;
              workingset->isActiveConstr
                  [(workingset->isActiveIdx[mWorkingFixed] + ix) - 2] = false;
              workingset->Wid[mTotalWorkingEq_tmp_tmp - 1] =
                  workingset->Wid[i - 1];
              workingset->Wlocalidx[mTotalWorkingEq_tmp_tmp - 1] =
                  workingset->Wlocalidx[i - 1];
              totalRank = workingset->nVar;
              for (ix = 0; ix < totalRank; ix++) {
                workingset->ATwset[ix + workingset->ldA *
                                            (mTotalWorkingEq_tmp_tmp - 1)] =
                    workingset->ATwset[ix + workingset->ldA * (i - 1)];
              }
              workingset->bwset[mTotalWorkingEq_tmp_tmp - 1] =
                  workingset->bwset[i - 1];
              workingset->Wid[i - 1] =
                  workingset->Wid[workingset->nActiveConstr - 1];
              workingset->Wlocalidx[i - 1] =
                  workingset->Wlocalidx[workingset->nActiveConstr - 1];
              mTotalWorkingEq_tmp_tmp = workingset->nVar;
              for (ix = 0; ix < mTotalWorkingEq_tmp_tmp; ix++) {
                workingset->ATwset[ix + workingset->ldA * (i - 1)] =
                    workingset
                        ->ATwset[ix + workingset->ldA *
                                          (workingset->nActiveConstr - 1)];
              }
              workingset->bwset[i - 1] =
                  workingset->bwset[workingset->nActiveConstr - 1];
              workingset->nActiveConstr--;
              workingset->nWConstr[mWorkingFixed]--;
            }
          }
        }
      }
    }
  }
  if ((nDepInd != -1) && (workingset->nActiveConstr <= qrmanager->ldq)) {
    boolean_T guard1{false};
    boolean_T okWorkingSet;
    ix = workingset->nActiveConstr;
    i = workingset->nWConstr[0] + workingset->nWConstr[1];
    nVar = workingset->nVar;
    if ((workingset->nWConstr[2] + workingset->nWConstr[3]) +
            workingset->nWConstr[4] >
        0) {
      tol = 100.0 * static_cast<double>(workingset->nVar) *
            2.2204460492503131E-16;
      for (idx_col = 0; idx_col < i; idx_col++) {
        qrmanager->jpvt[idx_col] = 1;
      }
      mTotalWorkingEq_tmp_tmp = i + 1;
      for (idx_col = mTotalWorkingEq_tmp_tmp; idx_col <= ix; idx_col++) {
        qrmanager->jpvt[idx_col - 1] = 0;
      }
      mTotalWorkingEq_tmp_tmp = workingset->nActiveConstr;
      for (idx_col = 0; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
        totalRank = qrmanager->ldq * idx_col;
        ix = workingset->ldA * idx_col;
        for (k = 0; k < nVar; k++) {
          qrmanager->QR[totalRank + k] = workingset->ATwset[ix + k];
        }
      }
      QRManager::factorQRE(qrmanager, workingset->nVar,
                           workingset->nActiveConstr);
      ix = 0;
      for (idx_col = workingset->nActiveConstr - 1; idx_col + 1 > nVar;
           idx_col--) {
        ix++;
        memspace->workspace_int[ix - 1] = qrmanager->jpvt[idx_col];
      }
      if (idx_col + 1 <= workingset->nVar) {
        totalRank = idx_col + qrmanager->ldq * idx_col;
        while ((idx_col + 1 > i) &&
               (std::abs(qrmanager->QR[totalRank]) < tol)) {
          ix++;
          memspace->workspace_int[ix - 1] = qrmanager->jpvt[idx_col];
          idx_col--;
          totalRank = (totalRank - qrmanager->ldq) - 1;
        }
      }
      utils::countsort(memspace->workspace_int, ix, memspace->workspace_sort,
                       i + 1, workingset->nActiveConstr);
      for (idx_col = ix; idx_col >= 1; idx_col--) {
        WorkingSet::removeConstr(workingset,
                                 memspace->workspace_int[idx_col - 1]);
      }
    }
    okWorkingSet = feasibleX0ForWorkingSet(
        memspace->workspace_double, solution->xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      ix = workingset->nActiveConstr;
      i = workingset->nWConstr[0] + workingset->nWConstr[1];
      nVar = workingset->nVar;
      if ((workingset->nWConstr[2] + workingset->nWConstr[3]) +
              workingset->nWConstr[4] >
          0) {
        tol = 1000.0 * static_cast<double>(workingset->nVar) *
              2.2204460492503131E-16;
        for (idx_col = 0; idx_col < i; idx_col++) {
          qrmanager->jpvt[idx_col] = 1;
        }
        mTotalWorkingEq_tmp_tmp = i + 1;
        for (idx_col = mTotalWorkingEq_tmp_tmp; idx_col <= ix; idx_col++) {
          qrmanager->jpvt[idx_col - 1] = 0;
        }
        mTotalWorkingEq_tmp_tmp = workingset->nActiveConstr;
        for (idx_col = 0; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
          totalRank = qrmanager->ldq * idx_col;
          ix = workingset->ldA * idx_col;
          for (k = 0; k < nVar; k++) {
            qrmanager->QR[totalRank + k] = workingset->ATwset[ix + k];
          }
        }
        QRManager::factorQRE(qrmanager, workingset->nVar,
                             workingset->nActiveConstr);
        ix = 0;
        for (idx_col = workingset->nActiveConstr - 1; idx_col + 1 > nVar;
             idx_col--) {
          ix++;
          memspace->workspace_int[ix - 1] = qrmanager->jpvt[idx_col];
        }
        if (idx_col + 1 <= workingset->nVar) {
          totalRank = idx_col + qrmanager->ldq * idx_col;
          while ((idx_col + 1 > i) &&
                 (std::abs(qrmanager->QR[totalRank]) < tol)) {
            ix++;
            memspace->workspace_int[ix - 1] = qrmanager->jpvt[idx_col];
            idx_col--;
            totalRank = (totalRank - qrmanager->ldq) - 1;
          }
        }
        utils::countsort(memspace->workspace_int, ix, memspace->workspace_sort,
                         i + 1, workingset->nActiveConstr);
        for (idx_col = ix; idx_col >= 1; idx_col--) {
          WorkingSet::removeConstr(workingset,
                                   memspace->workspace_int[idx_col - 1]);
        }
      }
      okWorkingSet = feasibleX0ForWorkingSet(
          memspace->workspace_double, solution->xstar, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1 && (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                   workingset->nVar)) {
      tol = WorkingSet::maxConstraintViolation(workingset, solution->xstar);
      if (tol > 1.0E-8) {
        solution->state = -2;
      }
    }
  } else {
    solution->state = -3;
    totalRank = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
    ix = workingset->nActiveConstr;
    for (mWorkingFixed = totalRank; mWorkingFixed <= ix; mWorkingFixed++) {
      workingset->isActiveConstr
          [(workingset->isActiveIdx[workingset->Wid[mWorkingFixed - 1] - 1] +
            workingset->Wlocalidx[mWorkingFixed - 1]) -
           2] = false;
    }
    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr =
        workingset->nWConstr[0] + workingset->nWConstr[1];
  }
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for PresolveWorkingSet.cpp
//
// [EOF]
//
