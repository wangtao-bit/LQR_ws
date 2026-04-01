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


#ifndef HUSKY_LQR__UNIFIED_LQR_SOLVER_HPP_
#define HUSKY_LQR__UNIFIED_LQR_SOLVER_HPP_

#include <Eigen/Dense>

namespace husky_lqr
{

class UnifiedLQRSolver
{
public:
  using MatrixA = Eigen::Matrix3d;
  using MatrixB = Eigen::Matrix<double, 3, 2>;
  using MatrixQ = Eigen::Matrix3d;
  using MatrixR = Eigen::Matrix2d;
  using MatrixK = Eigen::Matrix<double, 2, 3>;

  bool solve(
    const MatrixA & A,
    const MatrixB & B,
    const MatrixQ & Q,
    const MatrixR & R,
    MatrixK & K_out,
    int max_iterations = 200,
    double tolerance = 1e-8) const;
};

}  // namespace husky_lqr

#endif  // HUSKY_LQR__UNIFIED_LQR_SOLVER_HPP_
