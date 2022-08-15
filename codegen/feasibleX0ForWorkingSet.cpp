//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: feasibleX0ForWorkingSet.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Aug-2022 13:32:10
//

// Include Files
#include "feasibleX0ForWorkingSet.h"
#include "computeQ_.h"
#include "factorQR.h"
#include "maxConstraintViolation.h"
#include "mpc_fun_internal_types.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xzgeqp3.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : ::coder::array<double, 2U> &workspace
//                ::coder::array<double, 1U> &xCurrent
//                f_struct_T *workingset
//                d_struct_T *qrmanager
// Return Type  : boolean_T
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
boolean_T feasibleX0ForWorkingSet(::coder::array<double, 2U> &workspace,
                                  ::coder::array<double, 1U> &xCurrent,
                                  f_struct_T *workingset, d_struct_T *qrmanager)
{
  array<double, 2U> B;
  int mWConstr;
  int nVar;
  boolean_T nonDegenerateWset;
  mWConstr = workingset->nActiveConstr;
  nVar = workingset->nVar;
  nonDegenerateWset = true;
  if (mWConstr != 0) {
    double c;
    int ar;
    int br;
    int i;
    int i1;
    int iAcol;
    int idx;
    int iy;
    int jBcol;
    int mFixed;
    for (idx = 0; idx < mWConstr; idx++) {
      workspace[idx] = workingset->bwset[idx];
      workspace[idx + workspace.size(0)] = workingset->bwset[idx];
    }
    iAcol = workingset->ldA;
    if (mWConstr != 0) {
      iy = 0;
      i = workingset->ldA * (mWConstr - 1) + 1;
      for (jBcol = 1; iAcol < 0 ? jBcol >= i : jBcol <= i; jBcol += iAcol) {
        c = 0.0;
        i1 = (jBcol + nVar) - 1;
        for (br = jBcol; br <= i1; br++) {
          c += workingset->ATwset[br - 1] * xCurrent[br - jBcol];
        }
        workspace[iy] = workspace[iy] + -c;
        iy++;
      }
    }
    if (mWConstr >= nVar) {
      int ldq;
      int ldw;
      qrmanager->usedPivoting = false;
      qrmanager->mrows = mWConstr;
      qrmanager->ncols = nVar;
      for (iy = 0; iy < nVar; iy++) {
        iAcol = qrmanager->ldq * iy;
        for (jBcol = 0; jBcol < mWConstr; jBcol++) {
          qrmanager->QR[jBcol + iAcol] =
              workingset->ATwset[iy + workingset->ldA * jBcol];
        }
        qrmanager->jpvt[iy] = iy + 1;
      }
      if (mWConstr < nVar) {
        i = mWConstr;
      } else {
        i = nVar;
      }
      qrmanager->minRowCol = i;
      B.set_size(qrmanager->QR.size(0), qrmanager->QR.size(1));
      iAcol = qrmanager->QR.size(0) * qrmanager->QR.size(1);
      for (i1 = 0; i1 < iAcol; i1++) {
        B[i1] = qrmanager->QR[i1];
      }
      iAcol = qrmanager->QR.size(0);
      iy = qrmanager->QR.size(1);
      if (iAcol < iy) {
        iy = iAcol;
      }
      qrmanager->tau.set_size(iy);
      for (i1 = 0; i1 < iy; i1++) {
        qrmanager->tau[i1] = 0.0;
      }
      if (i >= 1) {
        internal::reflapack::qrf(B, mWConstr, nVar, i, qrmanager->tau);
      }
      qrmanager->QR.set_size(B.size(0), B.size(1));
      iAcol = B.size(0) * B.size(1);
      for (i = 0; i < iAcol; i++) {
        qrmanager->QR[i] = B[i];
      }
      QRManager::computeQ_(qrmanager, mWConstr);
      ldq = qrmanager->ldq;
      ldw = workspace.size(0);
      B.set_size(workspace.size(0), workspace.size(1));
      iAcol = workspace.size(0) * workspace.size(1);
      for (i = 0; i < iAcol; i++) {
        B[i] = workspace[i];
      }
      for (jBcol = 0; ldw < 0 ? jBcol >= ldw : jBcol <= ldw; jBcol += ldw) {
        i = jBcol + 1;
        i1 = jBcol + nVar;
        for (idx = i; idx <= i1; idx++) {
          workspace[idx - 1] = 0.0;
        }
      }
      br = -1;
      for (jBcol = 0; ldw < 0 ? jBcol >= ldw : jBcol <= ldw; jBcol += ldw) {
        ar = -1;
        i = jBcol + 1;
        i1 = jBcol + nVar;
        for (idx = i; idx <= i1; idx++) {
          c = 0.0;
          for (iy = 0; iy < mWConstr; iy++) {
            c += qrmanager->Q[(iy + ar) + 1] * B[(iy + br) + 1];
          }
          workspace[idx - 1] = workspace[idx - 1] + c;
          ar += ldq;
        }
        br += ldw;
      }
      for (mFixed = 0; mFixed < 2; mFixed++) {
        jBcol = ldw * mFixed - 1;
        for (idx = nVar; idx >= 1; idx--) {
          iy = ldq * (idx - 1) - 1;
          i = idx + jBcol;
          if (workspace[i] != 0.0) {
            workspace[i] = workspace[i] / qrmanager->QR[idx + iy];
            for (br = 0; br <= idx - 2; br++) {
              i1 = (br + jBcol) + 1;
              workspace[i1] =
                  workspace[i1] - workspace[i] * qrmanager->QR[(br + iy) + 1];
            }
          }
        }
      }
    } else {
      int ldq;
      int ldw;
      QRManager::factorQR(qrmanager, workingset->ATwset, nVar, mWConstr,
                          workingset->ldA);
      QRManager::computeQ_(qrmanager, qrmanager->minRowCol);
      ldq = qrmanager->ldq;
      ldw = workspace.size(0);
      for (mFixed = 0; mFixed < 2; mFixed++) {
        jBcol = ldw * mFixed;
        for (br = 0; br < mWConstr; br++) {
          iAcol = ldq * br;
          iy = br + jBcol;
          c = workspace[iy];
          for (idx = 0; idx < br; idx++) {
            c -= qrmanager->QR[idx + iAcol] * workspace[idx + jBcol];
          }
          workspace[iy] = c / qrmanager->QR[br + iAcol];
        }
      }
      B.set_size(workspace.size(0), workspace.size(1));
      iAcol = workspace.size(0) * workspace.size(1);
      for (i = 0; i < iAcol; i++) {
        B[i] = workspace[i];
      }
      for (jBcol = 0; ldw < 0 ? jBcol >= ldw : jBcol <= ldw; jBcol += ldw) {
        i = jBcol + 1;
        i1 = jBcol + nVar;
        for (idx = i; idx <= i1; idx++) {
          workspace[idx - 1] = 0.0;
        }
      }
      br = 0;
      for (jBcol = 0; ldw < 0 ? jBcol >= ldw : jBcol <= ldw; jBcol += ldw) {
        ar = -1;
        i = br + 1;
        i1 = br + mWConstr;
        for (mFixed = i; mFixed <= i1; mFixed++) {
          iy = jBcol + 1;
          iAcol = jBcol + nVar;
          for (idx = iy; idx <= iAcol; idx++) {
            workspace[idx - 1] =
                workspace[idx - 1] +
                B[mFixed - 1] * qrmanager->Q[(ar + idx) - jBcol];
          }
          ar += ldq;
        }
        br += ldw;
      }
    }
    idx = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (idx <= nVar - 1) {
        c = workspace[idx];
        if (std::isinf(c) || std::isnan(c)) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          c = workspace[idx + workspace.size(0)];
          if (std::isinf(c) || std::isnan(c)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            idx++;
          }
        }
      } else {
        double constrViolation_basicX;
        iAcol = nVar - 1;
        for (idx = 0; idx <= iAcol; idx++) {
          workspace[idx] = workspace[idx] + xCurrent[idx];
        }
        br = workingset->sizes[3];
        ar = workingset->sizes[4];
        mFixed = workingset->sizes[0];
        switch (workingset->probType) {
        case 2:
          c = 0.0;
          iAcol = workingset->sizes[2] - 1;
          jBcol = workingset->sizes[1] - 1;
          if (workingset->Aineq.size(0) != 0) {
            for (idx = 0; idx <= iAcol; idx++) {
              workingset->maxConstrWorkspace[idx] = workingset->bineq[idx];
            }
            internal::blas::xgemv(workingset->nVarOrig, workingset->sizes[2],
                                  workingset->Aineq, workingset->ldA, workspace,
                                  workingset->maxConstrWorkspace);
            for (idx = 0; idx <= iAcol; idx++) {
              workingset->maxConstrWorkspace[idx] =
                  workingset->maxConstrWorkspace[idx] -
                  workspace[workingset->nVarOrig + idx];
              c = std::fmax(c, workingset->maxConstrWorkspace[idx]);
            }
          }
          if (workingset->Aeq.size(0) != 0) {
            for (idx = 0; idx <= jBcol; idx++) {
              workingset->maxConstrWorkspace[idx] = workingset->beq[idx];
            }
            internal::blas::xgemv(workingset->nVarOrig, workingset->sizes[1],
                                  workingset->Aeq, workingset->ldA, workspace,
                                  workingset->maxConstrWorkspace);
            iAcol = workingset->nVarOrig + workingset->sizes[2];
            iy = iAcol + workingset->sizes[1];
            for (idx = 0; idx <= jBcol; idx++) {
              workingset->maxConstrWorkspace[idx] =
                  (workingset->maxConstrWorkspace[idx] -
                   workspace[iAcol + idx]) +
                  workspace[iy + idx];
              c = std::fmax(c, std::abs(workingset->maxConstrWorkspace[idx]));
            }
          }
          break;
        default:
          c = 0.0;
          iAcol = workingset->sizes[2] - 1;
          jBcol = workingset->sizes[1] - 1;
          if (workingset->Aineq.size(0) != 0) {
            for (idx = 0; idx <= iAcol; idx++) {
              workingset->maxConstrWorkspace[idx] = workingset->bineq[idx];
            }
            internal::blas::xgemv(workingset->nVar, workingset->sizes[2],
                                  workingset->Aineq, workingset->ldA, workspace,
                                  workingset->maxConstrWorkspace);
            for (idx = 0; idx <= iAcol; idx++) {
              c = std::fmax(c, workingset->maxConstrWorkspace[idx]);
            }
          }
          if (workingset->Aeq.size(0) != 0) {
            for (idx = 0; idx <= jBcol; idx++) {
              workingset->maxConstrWorkspace[idx] = workingset->beq[idx];
            }
            internal::blas::xgemv(workingset->nVar, workingset->sizes[1],
                                  workingset->Aeq, workingset->ldA, workspace,
                                  workingset->maxConstrWorkspace);
            for (idx = 0; idx <= jBcol; idx++) {
              c = std::fmax(c, std::abs(workingset->maxConstrWorkspace[idx]));
            }
          }
          break;
        }
        if (workingset->sizes[3] > 0) {
          for (idx = 0; idx < br; idx++) {
            iAcol = workingset->indexLB[idx] - 1;
            c = std::fmax(c, -workspace[iAcol] - workingset->lb[iAcol]);
          }
        }
        if (workingset->sizes[4] > 0) {
          for (idx = 0; idx < ar; idx++) {
            iAcol = workingset->indexUB[idx] - 1;
            c = std::fmax(c, workspace[iAcol] - workingset->ub[iAcol]);
          }
        }
        if (workingset->sizes[0] > 0) {
          for (idx = 0; idx < mFixed; idx++) {
            c = std::fmax(
                c, std::abs(workspace[workingset->indexFixed[idx] - 1] -
                            workingset->ub[workingset->indexFixed[idx] - 1]));
          }
        }
        constrViolation_basicX = WorkingSet::maxConstraintViolation(
            workingset, workspace, workspace.size(0) + 1);
        if ((c <= 2.2204460492503131E-16) || (c < constrViolation_basicX)) {
          for (idx = 0; idx < nVar; idx++) {
            xCurrent[idx] = workspace[idx];
          }
        } else {
          for (idx = 0; idx < nVar; idx++) {
            xCurrent[idx] = workspace[workspace.size(0) + idx];
          }
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

//
// File trailer for feasibleX0ForWorkingSet.cpp
//
// [EOF]
//
