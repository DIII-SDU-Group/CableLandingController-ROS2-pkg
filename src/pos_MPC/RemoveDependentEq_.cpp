//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// RemoveDependentEq_.cpp
//
// Code generation for function 'RemoveDependentEq_'
//

// Include files
#include "RemoveDependentEq_.h"
#include "MPCStepFunction_internal_types.h"
#include "computeQ_.h"
#include "countsort.h"
#include "rt_nonfinite.h"
#include "xgeqp3.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
int RemoveDependentEq_(e_struct_T *memspace, g_struct_T *workingset,
                       f_struct_T *qrmanager)
{
  int mTotalWorkingEq_tmp_tmp;
  int mWorkingFixed;
  int nDepInd;
  int nVar;
  nVar = workingset->nVar - 1;
  mWorkingFixed = workingset->nWConstr[0];
  mTotalWorkingEq_tmp_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    double tol;
    int i;
    int idx;
    int idxDiag;
    int idx_col;
    int k;
    int u0;
    for (idxDiag = 0; idxDiag < mTotalWorkingEq_tmp_tmp; idxDiag++) {
      for (idx_col = 0; idx_col <= nVar; idx_col++) {
        qrmanager->QR[idxDiag + 300 * idx_col] =
            workingset->ATwset[idx_col + 181 * idxDiag];
      }
    }
    nDepInd = mTotalWorkingEq_tmp_tmp - workingset->nVar;
    if (0 > nDepInd) {
      nDepInd = 0;
    }
    if (0 <= nVar) {
      std::memset(&qrmanager->jpvt[0], 0, (nVar + 1) * sizeof(int));
    }
    i = mTotalWorkingEq_tmp_tmp * workingset->nVar;
    if (i == 0) {
      qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager->ncols = workingset->nVar;
      qrmanager->minRowCol = 0;
    } else {
      qrmanager->usedPivoting = true;
      qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager->ncols = workingset->nVar;
      idxDiag = workingset->nVar;
      if (mTotalWorkingEq_tmp_tmp < idxDiag) {
        idxDiag = mTotalWorkingEq_tmp_tmp;
      }
      qrmanager->minRowCol = idxDiag;
      internal::lapack::xgeqp3(qrmanager->QR, mTotalWorkingEq_tmp_tmp,
                               workingset->nVar, qrmanager->jpvt,
                               qrmanager->tau);
    }
    tol =
        100.0 * static_cast<double>(workingset->nVar) * 2.2204460492503131E-16;
    u0 = workingset->nVar;
    if (u0 >= mTotalWorkingEq_tmp_tmp) {
      u0 = mTotalWorkingEq_tmp_tmp;
    }
    idxDiag = u0 + 300 * (u0 - 1);
    while ((idxDiag > 0) && (std::abs(qrmanager->QR[idxDiag - 1]) < tol)) {
      idxDiag -= 301;
      nDepInd++;
    }
    if (nDepInd > 0) {
      boolean_T exitg1;
      QRManager::computeQ_(qrmanager, qrmanager->mrows);
      idx = 0;
      exitg1 = false;
      while ((!exitg1) && (idx <= nDepInd - 1)) {
        double qtb;
        idxDiag = 300 * ((mTotalWorkingEq_tmp_tmp - idx) - 1);
        qtb = 0.0;
        for (k = 0; k < mTotalWorkingEq_tmp_tmp; k++) {
          qtb += qrmanager->Q[idxDiag + k] * workingset->bwset[k];
        }
        if (std::abs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          idx++;
        }
      }
    }
    if (nDepInd > 0) {
      int ix0;
      for (idx_col = 0; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
        idxDiag = 300 * idx_col;
        ix0 = 181 * idx_col;
        for (k = 0; k <= nVar; k++) {
          qrmanager->QR[idxDiag + k] = workingset->ATwset[ix0 + k];
        }
      }
      for (idx = 0; idx < mWorkingFixed; idx++) {
        qrmanager->jpvt[idx] = 1;
      }
      idx_col = workingset->nWConstr[0] + 1;
      if (idx_col <= mTotalWorkingEq_tmp_tmp) {
        std::memset(&qrmanager->jpvt[idx_col + -1], 0,
                    ((mTotalWorkingEq_tmp_tmp - idx_col) + 1) * sizeof(int));
      }
      if (i == 0) {
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager->minRowCol = 0;
      } else {
        qrmanager->usedPivoting = true;
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager->minRowCol = u0;
        internal::lapack::xgeqp3(qrmanager->QR, workingset->nVar,
                                 mTotalWorkingEq_tmp_tmp, qrmanager->jpvt,
                                 qrmanager->tau);
      }
      for (idx = 0; idx < nDepInd; idx++) {
        memspace->workspace_int[idx] =
            qrmanager->jpvt[(mTotalWorkingEq_tmp_tmp - nDepInd) + idx];
      }
      utils::countsort(memspace->workspace_int, nDepInd,
                       memspace->workspace_sort, 1, mTotalWorkingEq_tmp_tmp);
      for (idx = nDepInd; idx >= 1; idx--) {
        i = workingset->nWConstr[0] + workingset->nWConstr[1];
        if (i != 0) {
          idx_col = memspace->workspace_int[idx - 1];
          if (idx_col <= i) {
            if ((workingset->nActiveConstr == i) || (idx_col == i)) {
              workingset->mEqRemoved++;
              i = memspace->workspace_int[idx - 1] - 1;
              workingset->indexEqRemoved[workingset->mEqRemoved - 1] =
                  workingset->Wlocalidx[i];
              idxDiag = workingset->Wid[i] - 1;
              workingset->isActiveConstr[(workingset->isActiveIdx[idxDiag] +
                                          workingset->Wlocalidx[i]) -
                                         2] = false;
              workingset->Wid[i] =
                  workingset->Wid[workingset->nActiveConstr - 1];
              workingset->Wlocalidx[i] =
                  workingset->Wlocalidx[workingset->nActiveConstr - 1];
              idx_col = workingset->nVar;
              for (u0 = 0; u0 < idx_col; u0++) {
                workingset
                    ->ATwset[u0 +
                             181 * (memspace->workspace_int[idx - 1] - 1)] =
                    workingset
                        ->ATwset[u0 + 181 * (workingset->nActiveConstr - 1)];
              }
              workingset->bwset[i] =
                  workingset->bwset[workingset->nActiveConstr - 1];
              workingset->nActiveConstr--;
              workingset->nWConstr[idxDiag]--;
            } else {
              workingset->mEqRemoved++;
              idxDiag = workingset->Wid[idx_col - 1] - 1;
              workingset->indexEqRemoved[workingset->mEqRemoved - 1] =
                  workingset->Wlocalidx[idx_col - 1];
              workingset->isActiveConstr
                  [(workingset->isActiveIdx[workingset->Wid[idx_col - 1] - 1] +
                    workingset->Wlocalidx[idx_col - 1]) -
                   2] = false;
              workingset->Wid[idx_col - 1] = workingset->Wid[i - 1];
              workingset->Wlocalidx[idx_col - 1] = workingset->Wlocalidx[i - 1];
              ix0 = workingset->nVar;
              for (u0 = 0; u0 < ix0; u0++) {
                workingset->ATwset[u0 + 181 * (idx_col - 1)] =
                    workingset->ATwset[u0 + 181 * (i - 1)];
              }
              workingset->bwset[idx_col - 1] = workingset->bwset[i - 1];
              workingset->Wid[i - 1] =
                  workingset->Wid[workingset->nActiveConstr - 1];
              workingset->Wlocalidx[i - 1] =
                  workingset->Wlocalidx[workingset->nActiveConstr - 1];
              idx_col = workingset->nVar;
              for (u0 = 0; u0 < idx_col; u0++) {
                workingset->ATwset[u0 + 181 * (i - 1)] =
                    workingset
                        ->ATwset[u0 + 181 * (workingset->nActiveConstr - 1)];
              }
              workingset->bwset[i - 1] =
                  workingset->bwset[workingset->nActiveConstr - 1];
              workingset->nActiveConstr--;
              workingset->nWConstr[idxDiag]--;
            }
          }
        }
      }
    }
  }
  return nDepInd;
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (RemoveDependentEq_.cpp)
