//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// feasibleX0ForWorkingSet.cpp
//
// Code generation for function 'feasibleX0ForWorkingSet'
//

// Include files
#include "feasibleX0ForWorkingSet.h"
#include "MPCStepFunction_internal_types.h"
#include "MPCStepFunction_rtwutil.h"
#include "computeQ_.h"
#include "factorQR.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
boolean_T feasibleX0ForWorkingSet(double workspace[38841], double xCurrent[121],
                                  g_struct_T *workingset, f_struct_T *qrmanager)
{
  static double b_B[38841];
  int mWConstr;
  int nVar;
  boolean_T nonDegenerateWset;
  mWConstr = workingset->nActiveConstr;
  nVar = workingset->nVar;
  nonDegenerateWset = true;
  if (mWConstr != 0) {
    double c;
    int ar;
    int i;
    int i1;
    int iAcol;
    int idx;
    int jBcol;
    int k;
    int mLB;
    for (idx = 0; idx < mWConstr; idx++) {
      workspace[idx] = workingset->bwset[idx];
      workspace[idx + 321] = workingset->bwset[idx];
    }
    if (mWConstr != 0) {
      i = 121 * (mWConstr - 1) + 1;
      for (iAcol = 1; iAcol <= i; iAcol += 121) {
        c = 0.0;
        i1 = (iAcol + nVar) - 1;
        for (idx = iAcol; idx <= i1; idx++) {
          c += workingset->ATwset[idx - 1] * xCurrent[idx - iAcol];
        }
        i1 = div_nde_s32_floor(iAcol - 1, 121);
        workspace[i1] += -c;
      }
    }
    if (mWConstr >= nVar) {
      qrmanager->usedPivoting = false;
      qrmanager->mrows = mWConstr;
      qrmanager->ncols = nVar;
      for (mLB = 0; mLB < nVar; mLB++) {
        iAcol = 200 * mLB;
        for (jBcol = 0; jBcol < mWConstr; jBcol++) {
          qrmanager->QR[jBcol + iAcol] = workingset->ATwset[mLB + 121 * jBcol];
        }
        qrmanager->jpvt[mLB] = mLB + 1;
      }
      if (mWConstr < nVar) {
        i = mWConstr;
      } else {
        i = nVar;
      }
      qrmanager->minRowCol = i;
      std::memset(&qrmanager->tau[0], 0, 200U * sizeof(double));
      if (i >= 1) {
        internal::reflapack::qrf(qrmanager->QR, mWConstr, nVar, i,
                                 qrmanager->tau);
      }
      QRManager::computeQ_(qrmanager, mWConstr);
      std::copy(&workspace[0], &workspace[38841], &b_B[0]);
      for (k = 0; k <= 321; k += 321) {
        i = k + 1;
        i1 = k + nVar;
        if (i <= i1) {
          std::memset(&workspace[i + -1], 0, ((i1 - i) + 1) * sizeof(double));
        }
      }
      jBcol = -1;
      for (k = 0; k <= 321; k += 321) {
        ar = -1;
        i = k + 1;
        i1 = k + nVar;
        for (int ic{i}; ic <= i1; ic++) {
          c = 0.0;
          for (iAcol = 0; iAcol < mWConstr; iAcol++) {
            c += qrmanager->Q[(iAcol + ar) + 1] * b_B[(iAcol + jBcol) + 1];
          }
          workspace[ic - 1] += c;
          ar += 200;
        }
        jBcol += 321;
      }
      for (idx = 0; idx < 2; idx++) {
        jBcol = 321 * idx - 1;
        for (k = nVar; k >= 1; k--) {
          iAcol = 200 * (k - 1) - 1;
          i = k + jBcol;
          c = workspace[i];
          if (c != 0.0) {
            workspace[i] = c / qrmanager->QR[k + iAcol];
            for (ar = 0; ar <= k - 2; ar++) {
              i1 = (ar + jBcol) + 1;
              workspace[i1] -= workspace[i] * qrmanager->QR[(ar + iAcol) + 1];
            }
          }
        }
      }
    } else {
      QRManager::factorQR(qrmanager, workingset->ATwset, nVar, mWConstr);
      QRManager::computeQ_(qrmanager, qrmanager->minRowCol);
      for (idx = 0; idx < 2; idx++) {
        jBcol = 321 * idx;
        for (ar = 0; ar < mWConstr; ar++) {
          iAcol = 200 * ar;
          mLB = ar + jBcol;
          c = workspace[mLB];
          for (k = 0; k < ar; k++) {
            c -= qrmanager->QR[k + iAcol] * workspace[k + jBcol];
          }
          workspace[mLB] = c / qrmanager->QR[ar + iAcol];
        }
      }
      std::copy(&workspace[0], &workspace[38841], &b_B[0]);
      for (k = 0; k <= 321; k += 321) {
        i = k + 1;
        i1 = k + nVar;
        if (i <= i1) {
          std::memset(&workspace[i + -1], 0, ((i1 - i) + 1) * sizeof(double));
        }
      }
      jBcol = 0;
      for (k = 0; k <= 321; k += 321) {
        ar = -1;
        i = jBcol + 1;
        i1 = jBcol + mWConstr;
        for (idx = i; idx <= i1; idx++) {
          iAcol = k + 1;
          mLB = k + nVar;
          for (int ic{iAcol}; ic <= mLB; ic++) {
            workspace[ic - 1] += b_B[idx - 1] * qrmanager->Q[(ar + ic) - k];
          }
          ar += 200;
        }
        jBcol += 321;
      }
    }
    idx = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (idx <= nVar - 1) {
        if (std::isinf(workspace[idx]) || std::isnan(workspace[idx])) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          c = workspace[idx + 321];
          if (std::isinf(c) || std::isnan(c)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            idx++;
          }
        }
      } else {
        double b_v;
        double v;
        iAcol = nVar - 1;
        for (k = 0; k <= iAcol; k++) {
          workspace[k] += xCurrent[k];
        }
        mLB = workingset->sizes[3];
        jBcol = workingset->sizes[4];
        ar = workingset->sizes[0];
        switch (workingset->probType) {
        case 2:
          v = 0.0;
          for (k = 0; k < 80; k++) {
            workingset->maxConstrWorkspace[k] = workingset->beq[k];
            workingset->maxConstrWorkspace[k] =
                -workingset->maxConstrWorkspace[k];
          }
          for (iAcol = 0; iAcol <= 9559; iAcol += 121) {
            c = 0.0;
            i = iAcol + 120;
            for (idx = iAcol + 1; idx <= i; idx++) {
              c += workingset->Aeq[idx - 1] * workspace[(idx - iAcol) - 1];
            }
            i = div_nde_s32_floor(iAcol, 121);
            workingset->maxConstrWorkspace[i] += c;
          }
          for (idx = 0; idx < 80; idx++) {
            workingset->maxConstrWorkspace[idx] =
                (workingset->maxConstrWorkspace[idx] - workspace[idx + 120]) +
                workspace[idx + 200];
            v = std::fmax(v, std::abs(workingset->maxConstrWorkspace[idx]));
          }
          break;
        default:
          v = 0.0;
          for (k = 0; k < 80; k++) {
            workingset->maxConstrWorkspace[k] = workingset->beq[k];
            workingset->maxConstrWorkspace[k] =
                -workingset->maxConstrWorkspace[k];
          }
          for (iAcol = 0; iAcol <= 9559; iAcol += 121) {
            c = 0.0;
            i = iAcol + workingset->nVar;
            for (idx = iAcol + 1; idx <= i; idx++) {
              c += workingset->Aeq[idx - 1] * workspace[(idx - iAcol) - 1];
            }
            i = div_nde_s32_floor(iAcol, 121);
            workingset->maxConstrWorkspace[i] += c;
          }
          for (idx = 0; idx < 80; idx++) {
            v = std::fmax(v, std::abs(workingset->maxConstrWorkspace[idx]));
          }
          break;
        }
        if (workingset->sizes[3] > 0) {
          for (idx = 0; idx < mLB; idx++) {
            iAcol = workingset->indexLB[idx] - 1;
            v = std::fmax(v, -workspace[iAcol] - workingset->lb[iAcol]);
          }
        }
        if (workingset->sizes[4] > 0) {
          for (idx = 0; idx < jBcol; idx++) {
            iAcol = workingset->indexUB[idx] - 1;
            v = std::fmax(v, workspace[iAcol] - workingset->ub[iAcol]);
          }
        }
        if (workingset->sizes[0] > 0) {
          for (idx = 0; idx < ar; idx++) {
            v = std::fmax(
                v, std::abs(workspace[workingset->indexFixed[idx] - 1] -
                            workingset->ub[workingset->indexFixed[idx] - 1]));
          }
        }
        mLB = workingset->sizes[3];
        jBcol = workingset->sizes[4];
        ar = workingset->sizes[0];
        switch (workingset->probType) {
        case 2:
          b_v = 0.0;
          for (k = 0; k < 80; k++) {
            workingset->maxConstrWorkspace[k] = workingset->beq[k];
            workingset->maxConstrWorkspace[k] =
                -workingset->maxConstrWorkspace[k];
          }
          for (iAcol = 0; iAcol <= 9559; iAcol += 121) {
            c = 0.0;
            i = iAcol + 120;
            for (idx = iAcol + 1; idx <= i; idx++) {
              c += workingset->Aeq[idx - 1] * workspace[(idx - iAcol) + 320];
            }
            i = div_nde_s32_floor(iAcol, 121);
            workingset->maxConstrWorkspace[i] += c;
          }
          for (idx = 0; idx < 80; idx++) {
            workingset->maxConstrWorkspace[idx] =
                (workingset->maxConstrWorkspace[idx] - workspace[idx + 441]) +
                workspace[idx + 521];
            b_v = std::fmax(b_v, std::abs(workingset->maxConstrWorkspace[idx]));
          }
          break;
        default:
          b_v = 0.0;
          for (k = 0; k < 80; k++) {
            workingset->maxConstrWorkspace[k] = workingset->beq[k];
            workingset->maxConstrWorkspace[k] =
                -workingset->maxConstrWorkspace[k];
          }
          for (iAcol = 0; iAcol <= 9559; iAcol += 121) {
            c = 0.0;
            i = iAcol + workingset->nVar;
            for (idx = iAcol + 1; idx <= i; idx++) {
              c += workingset->Aeq[idx - 1] * workspace[(idx - iAcol) + 320];
            }
            i = div_nde_s32_floor(iAcol, 121);
            workingset->maxConstrWorkspace[i] += c;
          }
          for (idx = 0; idx < 80; idx++) {
            b_v = std::fmax(b_v, std::abs(workingset->maxConstrWorkspace[idx]));
          }
          break;
        }
        if (workingset->sizes[3] > 0) {
          for (idx = 0; idx < mLB; idx++) {
            iAcol = workingset->indexLB[idx];
            b_v = std::fmax(b_v, -workspace[iAcol + 320] -
                                     workingset->lb[iAcol - 1]);
          }
        }
        if (workingset->sizes[4] > 0) {
          for (idx = 0; idx < jBcol; idx++) {
            iAcol = workingset->indexUB[idx];
            b_v = std::fmax(b_v,
                            workspace[iAcol + 320] - workingset->ub[iAcol - 1]);
          }
        }
        if (workingset->sizes[0] > 0) {
          for (idx = 0; idx < ar; idx++) {
            b_v = std::fmax(
                b_v, std::abs(workspace[workingset->indexFixed[idx] + 320] -
                              workingset->ub[workingset->indexFixed[idx] - 1]));
          }
        }
        if ((v <= 2.2204460492503131E-16) || (v < b_v)) {
          if (0 <= nVar - 1) {
            std::copy(&workspace[0], &workspace[nVar], &xCurrent[0]);
          }
        } else if (0 <= nVar - 1) {
          std::copy(&workspace[321], &workspace[321 + nVar], &xCurrent[0]);
        }
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return nonDegenerateWset;
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (feasibleX0ForWorkingSet.cpp)
