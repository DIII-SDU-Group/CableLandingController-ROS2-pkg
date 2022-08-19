//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeFirstOrderOpt.cpp
//
// Code generation for function 'computeFirstOrderOpt'
//

// Include files
#include "computeFirstOrderOpt.h"
#include "PositionMPCStepFunction_internal_types.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>

// Function Definitions
namespace pos_MPC {
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace parseoutput {
void computeFirstOrderOpt(struct_T *solution, const b_struct_T *objective,
                          int workingset_nVar,
                          const double workingset_ATwset[90321],
                          int workingset_nActiveConstr, double workspace[90321])
{
  double smax;
  int ix;
  int k;
  if (0 <= workingset_nVar - 1) {
    std::copy(&objective->grad[0], &objective->grad[workingset_nVar],
              &workspace[0]);
  }
  if (workingset_nActiveConstr != 0) {
    ix = 0;
    k = 161 * (workingset_nActiveConstr - 1) + 1;
    for (int iac{1}; iac <= k; iac += 161) {
      int i;
      i = (iac + workingset_nVar) - 1;
      for (int ia{iac}; ia <= i; ia++) {
        int i1;
        i1 = ia - iac;
        workspace[i1] += workingset_ATwset[ia - 1] * solution->lambda[ix];
      }
      ix++;
    }
  }
  ix = 1;
  smax = std::abs(workspace[0]);
  for (k = 2; k <= workingset_nVar; k++) {
    double s;
    s = std::abs(workspace[k - 1]);
    if (s > smax) {
      ix = k;
      smax = s;
    }
  }
  solution->firstorderopt = std::abs(workspace[ix - 1]);
}

} // namespace parseoutput
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder
} // namespace pos_MPC

// End of code generation (computeFirstOrderOpt.cpp)
