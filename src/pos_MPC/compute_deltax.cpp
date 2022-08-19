//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// compute_deltax.cpp
//
// Code generation for function 'compute_deltax'
//

// Include files
#include "compute_deltax.h"
#include "PositionMPCStepFunction_internal_types.h"
#include "PositionMPCStepFunction_rtwutil.h"
#include "fullColLDL2_.h"
#include "partialColLDL3_.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void compute_deltax(const double H[25600], struct_T *solution,
                    f_struct_T *memspace, const g_struct_T *qrmanager,
                    c_struct_T *cholmanager, const b_struct_T *objective)
{
  int mNull_tmp;
  int nVar_tmp;
  nVar_tmp = qrmanager->mrows - 1;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    if (0 <= nVar_tmp) {
      std::memset(&solution->searchDir[0], 0, (nVar_tmp + 1) * sizeof(double));
    }
  } else {
    int br;
    for (br = 0; br <= nVar_tmp; br++) {
      solution->searchDir[br] = -objective->grad[br];
    }
    if (qrmanager->ncols <= 0) {
      if (objective->objtype == 3) {
        double temp;
        int A_maxDiag_idx;
        int lastColC;
        int nVars;
        temp = 1.4901161193847656E-6 * static_cast<double>(qrmanager->mrows);
        cholmanager->ndims = qrmanager->mrows;
        for (br = 0; br <= nVar_tmp; br++) {
          A_maxDiag_idx = (nVar_tmp + 1) * br;
          nVars = 240 * br;
          for (lastColC = 0; lastColC <= nVar_tmp; lastColC++) {
            cholmanager->FMat[nVars + lastColC] = H[A_maxDiag_idx + lastColC];
          }
        }
        if (qrmanager->mrows < 1) {
          A_maxDiag_idx = -1;
        } else {
          A_maxDiag_idx = 0;
          if (qrmanager->mrows > 1) {
            double smax;
            smax = std::abs(cholmanager->FMat[0]);
            for (lastColC = 2; lastColC <= nVar_tmp + 1; lastColC++) {
              double s;
              s = std::abs(cholmanager->FMat[(lastColC - 1) * 241]);
              if (s > smax) {
                A_maxDiag_idx = lastColC - 1;
                smax = s;
              }
            }
          }
        }
        cholmanager->regTol_ = std::fmax(
            std::abs(cholmanager->FMat[A_maxDiag_idx + 240 * A_maxDiag_idx]) *
                2.2204460492503131E-16,
            std::abs(temp));
        if (qrmanager->mrows > 128) {
          boolean_T exitg1;
          lastColC = 0;
          exitg1 = false;
          while ((!exitg1) && (lastColC < nVar_tmp + 1)) {
            nVars = 241 * lastColC + 1;
            A_maxDiag_idx = (nVar_tmp - lastColC) + 1;
            if (lastColC + 48 <= nVar_tmp + 1) {
              DynamicRegCholManager::partialColLDL3_(cholmanager, nVars,
                                                     A_maxDiag_idx, temp);
              lastColC += 48;
            } else {
              DynamicRegCholManager::fullColLDL2_(cholmanager, nVars,
                                                  A_maxDiag_idx, temp);
              exitg1 = true;
            }
          }
        } else {
          DynamicRegCholManager::fullColLDL2_(cholmanager, 1, qrmanager->mrows,
                                              temp);
        }
        if (cholmanager->ConvexCheck) {
          br = 0;
          int exitg2;
          do {
            exitg2 = 0;
            if (br <= nVar_tmp) {
              if (cholmanager->FMat[br + 240 * br] <= 0.0) {
                cholmanager->info = -br - 1;
                exitg2 = 1;
              } else {
                br++;
              }
            } else {
              cholmanager->ConvexCheck = false;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          int ar;
          int i;
          nVars = cholmanager->ndims - 2;
          if (cholmanager->ndims != 0) {
            for (lastColC = 0; lastColC <= nVars + 1; lastColC++) {
              A_maxDiag_idx = lastColC + lastColC * 240;
              i = nVars - lastColC;
              for (ar = 0; ar <= i; ar++) {
                br = (lastColC + ar) + 1;
                solution->searchDir[br] -=
                    solution->searchDir[lastColC] *
                    cholmanager->FMat[(A_maxDiag_idx + ar) + 1];
              }
            }
          }
          i = cholmanager->ndims;
          for (br = 0; br < i; br++) {
            solution->searchDir[br] /= cholmanager->FMat[br + 240 * br];
          }
          nVars = cholmanager->ndims;
          if (cholmanager->ndims != 0) {
            for (lastColC = nVars; lastColC >= 1; lastColC--) {
              A_maxDiag_idx = (lastColC - 1) * 240;
              temp = solution->searchDir[lastColC - 1];
              i = lastColC + 1;
              for (ar = nVars; ar >= i; ar--) {
                temp -= cholmanager->FMat[(A_maxDiag_idx + ar) - 1] *
                        solution->searchDir[ar - 1];
              }
              solution->searchDir[lastColC - 1] = temp;
            }
          }
        }
      }
    } else {
      int nullStartIdx;
      int nullStartIdx_tmp;
      nullStartIdx_tmp = 240 * qrmanager->ncols;
      nullStartIdx = nullStartIdx_tmp + 1;
      switch (objective->objtype) {
      case 5: {
        for (br = 0; br < mNull_tmp; br++) {
          memspace->workspace_double[br] =
              -qrmanager->Q[nVar_tmp + 240 * (qrmanager->ncols + br)];
        }
        if (qrmanager->mrows != 0) {
          int i;
          if (0 <= nVar_tmp) {
            std::memset(&solution->searchDir[0], 0,
                        (nVar_tmp + 1) * sizeof(double));
          }
          br = 0;
          i = (nullStartIdx_tmp + 240 * (mNull_tmp - 1)) + 1;
          for (int A_maxDiag_idx{nullStartIdx}; A_maxDiag_idx <= i;
               A_maxDiag_idx += 240) {
            int i1;
            i1 = A_maxDiag_idx + nVar_tmp;
            for (int nVars{A_maxDiag_idx}; nVars <= i1; nVars++) {
              int i2;
              i2 = nVars - A_maxDiag_idx;
              solution->searchDir[i2] +=
                  qrmanager->Q[nVars - 1] * memspace->workspace_double[br];
            }
            br++;
          }
        }
      } break;
      default: {
        double temp;
        int A_maxDiag_idx;
        int ar;
        int i;
        int i1;
        int i2;
        int lastColC;
        int nVars;
        switch (objective->objtype) {
        case 3: {
          int cr;
          int ic;
          nVars = qrmanager->mrows;
          if (qrmanager->mrows != 0) {
            br = nullStartIdx_tmp;
            lastColC = 561 * (mNull_tmp - 1);
            for (cr = 0; cr <= lastColC; cr += 561) {
              i = cr + 1;
              i1 = cr + nVars;
              if (i <= i1) {
                std::memset(&memspace->workspace_double[i + -1], 0,
                            ((i1 - i) + 1) * sizeof(double));
              }
            }
            for (cr = 0; cr <= lastColC; cr += 561) {
              ar = -1;
              i = br + 1;
              i1 = br + nVars;
              for (int ib{i}; ib <= i1; ib++) {
                i2 = cr + 1;
                A_maxDiag_idx = cr + nVars;
                for (ic = i2; ic <= A_maxDiag_idx; ic++) {
                  memspace->workspace_double[ic - 1] +=
                      qrmanager->Q[ib - 1] * H[(ar + ic) - cr];
                }
                ar += nVars;
              }
              br += 240;
            }
          }
          lastColC = 240 * (mNull_tmp - 1);
          for (cr = 0; cr <= lastColC; cr += 240) {
            i = cr + 1;
            i1 = cr + mNull_tmp;
            if (i <= i1) {
              std::memset(&cholmanager->FMat[i + -1], 0,
                          ((i1 - i) + 1) * sizeof(double));
            }
          }
          br = -1;
          for (cr = 0; cr <= lastColC; cr += 240) {
            ar = nullStartIdx_tmp;
            i = cr + 1;
            i1 = cr + mNull_tmp;
            for (ic = i; ic <= i1; ic++) {
              temp = 0.0;
              for (A_maxDiag_idx = 0; A_maxDiag_idx < nVars; A_maxDiag_idx++) {
                temp += qrmanager->Q[A_maxDiag_idx + ar] *
                        memspace->workspace_double[(A_maxDiag_idx + br) + 1];
              }
              cholmanager->FMat[ic - 1] += temp;
              ar += 240;
            }
            br += 561;
          }
        } break;
        }
        temp = 1.4901161193847656E-6 * static_cast<double>(mNull_tmp);
        cholmanager->ndims = mNull_tmp;
        A_maxDiag_idx = 0;
        if (mNull_tmp > 1) {
          double smax;
          smax = std::abs(cholmanager->FMat[0]);
          for (lastColC = 2; lastColC <= mNull_tmp; lastColC++) {
            double s;
            s = std::abs(cholmanager->FMat[(lastColC - 1) * 241]);
            if (s > smax) {
              A_maxDiag_idx = lastColC - 1;
              smax = s;
            }
          }
        }
        cholmanager->regTol_ = std::fmax(
            std::abs(cholmanager->FMat[A_maxDiag_idx + 240 * A_maxDiag_idx]) *
                2.2204460492503131E-16,
            temp);
        if (mNull_tmp > 128) {
          boolean_T exitg1;
          lastColC = 0;
          exitg1 = false;
          while ((!exitg1) && (lastColC < mNull_tmp)) {
            nVars = 241 * lastColC + 1;
            A_maxDiag_idx = mNull_tmp - lastColC;
            if (lastColC + 48 <= mNull_tmp) {
              DynamicRegCholManager::partialColLDL3_(cholmanager, nVars,
                                                     A_maxDiag_idx, temp);
              lastColC += 48;
            } else {
              DynamicRegCholManager::fullColLDL2_(cholmanager, nVars,
                                                  A_maxDiag_idx, temp);
              exitg1 = true;
            }
          }
        } else {
          DynamicRegCholManager::fullColLDL2_(cholmanager, 1, mNull_tmp, temp);
        }
        if (cholmanager->ConvexCheck) {
          br = 0;
          int exitg2;
          do {
            exitg2 = 0;
            if (br <= mNull_tmp - 1) {
              if (cholmanager->FMat[br + 240 * br] <= 0.0) {
                cholmanager->info = -br - 1;
                exitg2 = 1;
              } else {
                br++;
              }
            } else {
              cholmanager->ConvexCheck = false;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          if (qrmanager->mrows != 0) {
            if (0 <= mNull_tmp - 1) {
              std::memset(&memspace->workspace_double[0], 0,
                          mNull_tmp * sizeof(double));
            }
            i = (nullStartIdx_tmp + 240 * (mNull_tmp - 1)) + 1;
            for (A_maxDiag_idx = nullStartIdx; A_maxDiag_idx <= i;
                 A_maxDiag_idx += 240) {
              temp = 0.0;
              i1 = A_maxDiag_idx + nVar_tmp;
              for (nVars = A_maxDiag_idx; nVars <= i1; nVars++) {
                temp += qrmanager->Q[nVars - 1] *
                        objective->grad[nVars - A_maxDiag_idx];
              }
              i1 = div_nde_s32_floor((A_maxDiag_idx - nullStartIdx_tmp) - 1,
                                     240);
              memspace->workspace_double[i1] += -temp;
            }
          }
          nVars = cholmanager->ndims - 2;
          if (cholmanager->ndims != 0) {
            for (lastColC = 0; lastColC <= nVars + 1; lastColC++) {
              A_maxDiag_idx = lastColC + lastColC * 240;
              i = nVars - lastColC;
              for (ar = 0; ar <= i; ar++) {
                br = (lastColC + ar) + 1;
                memspace->workspace_double[br] -=
                    memspace->workspace_double[lastColC] *
                    cholmanager->FMat[(A_maxDiag_idx + ar) + 1];
              }
            }
          }
          i = cholmanager->ndims;
          for (br = 0; br < i; br++) {
            memspace->workspace_double[br] /= cholmanager->FMat[br + 240 * br];
          }
          nVars = cholmanager->ndims;
          if (cholmanager->ndims != 0) {
            for (lastColC = nVars; lastColC >= 1; lastColC--) {
              A_maxDiag_idx = (lastColC - 1) * 240;
              temp = memspace->workspace_double[lastColC - 1];
              i = lastColC + 1;
              for (ar = nVars; ar >= i; ar--) {
                temp -= cholmanager->FMat[(A_maxDiag_idx + ar) - 1] *
                        memspace->workspace_double[ar - 1];
              }
              memspace->workspace_double[lastColC - 1] = temp;
            }
          }
          if (qrmanager->mrows != 0) {
            if (0 <= nVar_tmp) {
              std::memset(&solution->searchDir[0], 0,
                          (nVar_tmp + 1) * sizeof(double));
            }
            br = 0;
            i = (nullStartIdx_tmp + 240 * (mNull_tmp - 1)) + 1;
            for (A_maxDiag_idx = nullStartIdx; A_maxDiag_idx <= i;
                 A_maxDiag_idx += 240) {
              i1 = A_maxDiag_idx + nVar_tmp;
              for (nVars = A_maxDiag_idx; nVars <= i1; nVars++) {
                i2 = nVars - A_maxDiag_idx;
                solution->searchDir[i2] +=
                    qrmanager->Q[nVars - 1] * memspace->workspace_double[br];
              }
              br++;
            }
          }
        }
      } break;
      }
    }
  }
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (compute_deltax.cpp)
