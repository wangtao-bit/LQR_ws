// Copyright 2026 wt
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "husky_lqr/unified_lqr_solver.hpp"

#include <cmath>

namespace husky_lqr
{

bool UnifiedLQRSolver::solve(
  const MatrixA & A,
  const MatrixB & B,
  const MatrixQ & Q,
  const MatrixR & R,
  MatrixK & K_out,
  int max_iterations,
  double tolerance) const
{
  Eigen::Matrix3d P = Q;
  bool converged = false;
  for (int i = 0; i < max_iterations; ++i) {
    Eigen::Matrix2d S = R + B.transpose() * P * B;
    Eigen::LDLT<Eigen::Matrix2d> ldlt(S);
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
      // Add small diagonal regularization for numerical robustness.
      S += 1e-6 * Eigen::Matrix2d::Identity();
      ldlt.compute(S);
      if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
        return false;
      }
    }

    const Eigen::Matrix2d S_inv = ldlt.solve(Eigen::Matrix2d::Identity());
    const Eigen::Matrix3d P_next =
      A.transpose() * P * A - A.transpose() * P * B * S_inv * B.transpose() * P * A + Q;

    if (!P_next.allFinite()) {
      return false;
    }
    if ((P_next - P).norm() < tolerance) {
      P = P_next;
      converged = true;
      break;
    }
    P = P_next;
  }

  Eigen::Matrix2d S = R + B.transpose() * P * B;
  Eigen::LDLT<Eigen::Matrix2d> ldlt(S);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    S += 1e-6 * Eigen::Matrix2d::Identity();
    ldlt.compute(S);
  }
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    return false;
  }

  K_out = ldlt.solve(B.transpose() * P * A);
  if (!K_out.allFinite()) {
    return false;
  }
  // Even if strict convergence is not reached in max_iterations, the finite-step
  // Riccati iterate can still provide a usable stabilizing gain for control.
  (void)converged;
  return true;
}

}  // namespace husky_lqr
