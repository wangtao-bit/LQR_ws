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


#include "husky_lqr/lqr_tracker_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace husky_lqr
{

LQRTrackerNode::LQRTrackerNode()
: Node("lqr_tracker_node")
{
  control_rate_ = this->declare_parameter<double>("control_rate", 30.0);
  lookahead_dist_ = this->declare_parameter<double>("lookahead_dist", 0.5);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.25);
  min_lqr_speed_ = this->declare_parameter<double>("min_lqr_speed", 0.10);
  kappa_speed_eps_ = this->declare_parameter<double>("kappa_speed_eps", 1e-3);

  base_v_ref_ = this->declare_parameter<double>("base_v_ref", 0.6);
  v_min_ = this->declare_parameter<double>("v_min", 0.0);
  v_max_ = this->declare_parameter<double>("v_max", 1.0);
  w_max_ = this->declare_parameter<double>("w_max", 1.5);
  accel_limit_ = this->declare_parameter<double>("accel_limit", 0.6);
  decel_limit_ = this->declare_parameter<double>("decel_limit", 0.8);
  w_accel_limit_ = this->declare_parameter<double>("w_accel_limit", 1.8);

  q_ex_ = this->declare_parameter<double>("q_ex", 2.0);
  q_ey_ = this->declare_parameter<double>("q_ey", 8.0);
  q_etheta_ = this->declare_parameter<double>("q_etheta", 3.0);
  r_v_ = this->declare_parameter<double>("r_v", 1.0);
  r_w_ = this->declare_parameter<double>("r_w", 0.8);

  dt_ = 1.0 / std::max(1.0, control_rate_);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/plan", rclcpp::QoS(10),
    std::bind(&LQRTrackerNode::onPath, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(20),
    std::bind(&LQRTrackerNode::onOdom, this, std::placeholders::_1));

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(20));

  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(dt_),
    std::bind(&LQRTrackerNode::onControlTimer, this));

  RCLCPP_INFO(
    this->get_logger(), "lqr_tracker_node started (unified LQR for v and omega)");
}

void LQRTrackerNode::onPath(const nav_msgs::msg::Path::SharedPtr msg)
{
  ref_path_ = toRefPath(*msg);
  if (ref_path_.size() < 2) {
    RCLCPP_WARN(
      this->get_logger(), "Received /plan but not enough points: %zu", ref_path_.size());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Received /plan with %zu points", ref_path_.size());
}

void LQRTrackerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

bool LQRTrackerNode::hasValidState() const
{
  return latest_odom_ != nullptr && !ref_path_.empty();
}

std::vector<LQRTrackerNode::RefPoint> LQRTrackerNode::toRefPath(
  const nav_msgs::msg::Path & path) const
{
  std::vector<RefPoint> out;
  if (path.poses.size() < 2) {
    return out;
  }

  out.reserve(path.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & pose = path.poses[i].pose;
    RefPoint p{};
    p.x = pose.position.x;
    p.y = pose.position.y;

    size_t i_prev = (i == 0) ? i : i - 1;
    size_t i_next = (i + 1 >= path.poses.size()) ? i : i + 1;
    const auto & p_prev = path.poses[i_prev].pose.position;
    const auto & p_next = path.poses[i_next].pose.position;

    const double dx = p_next.x - p_prev.x;
    const double dy = p_next.y - p_prev.y;
    p.theta = std::atan2(dy, dx);

    p.kappa = 0.0;
    if (i > 0 && i + 1 < path.poses.size()) {
      const auto & p0 = path.poses[i - 1].pose.position;
      const auto & p1 = path.poses[i].pose.position;
      const auto & p2 = path.poses[i + 1].pose.position;

      const double a = std::hypot(p1.x - p0.x, p1.y - p0.y);
      const double b = std::hypot(p2.x - p1.x, p2.y - p1.y);
      const double c = std::hypot(p2.x - p0.x, p2.y - p0.y);
      const double cross =
        (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
      const double denom = a * b * c;
      if (denom > 1e-9) {
        p.kappa = 2.0 * cross / denom;
      }
    }

    out.push_back(p);
  }
  return out;
}

size_t LQRTrackerNode::findNearestIndex(double x, double y) const
{
  double best_d2 = std::numeric_limits<double>::max();
  size_t best_idx = 0;

  for (size_t i = 0; i < ref_path_.size(); ++i) {
    const double dx = x - ref_path_[i].x;
    const double dy = y - ref_path_[i].y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_idx = i;
    }
  }

  double accum = 0.0;
  for (size_t i = best_idx; i + 1 < ref_path_.size(); ++i) {
    const double ds = std::hypot(
      ref_path_[i + 1].x - ref_path_[i].x,
      ref_path_[i + 1].y - ref_path_[i].y);
    accum += ds;
    if (accum >= lookahead_dist_) {
      return i + 1;
    }
  }

  return best_idx;
}

LQRTrackerNode::Errors LQRTrackerNode::computeErrors(
  double x, double y, double yaw, const RefPoint & ref) const
{
  const double dx = x - ref.x;
  const double dy = y - ref.y;

  const double cos_r = std::cos(ref.theta);
  const double sin_r = std::sin(ref.theta);

  Errors e{};
  e.ex = cos_r * dx + sin_r * dy;
  e.ey = -sin_r * dx + cos_r * dy;
  e.etheta = normalizeAngle(yaw - ref.theta);
  return e;
}

void LQRTrackerNode::updateLQRGain(double v_ref)
{
  const double v_for_lqr = std::max(std::abs(v_ref), min_lqr_speed_);

  UnifiedLQRSolver::MatrixA A = UnifiedLQRSolver::MatrixA::Identity();
  UnifiedLQRSolver::MatrixB B = UnifiedLQRSolver::MatrixB::Zero();

  // Linearized discrete-time error model around reference motion.
  A(0, 2) = 0.0;
  A(1, 2) = v_for_lqr * dt_;

  B(0, 0) = dt_;
  B(2, 1) = dt_;

  UnifiedLQRSolver::MatrixQ Q = UnifiedLQRSolver::MatrixQ::Zero();
  Q(0, 0) = q_ex_;
  Q(1, 1) = q_ey_;
  Q(2, 2) = q_etheta_;

  UnifiedLQRSolver::MatrixR R = UnifiedLQRSolver::MatrixR::Zero();
  R(0, 0) = r_v_;
  R(1, 1) = r_w_;

  UnifiedLQRSolver::MatrixK K_candidate;
  const bool solved = solver_.solve(A, B, Q, R, K_candidate);
  if (solved) {
    K_ = K_candidate;
    has_valid_k_ = true;
    return;
  }

  if (has_valid_k_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "DARE solve failed, reusing previous valid gain");
  } else {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Failed to solve DARE and no valid gain is available");
  }
}

void LQRTrackerNode::onControlTimer()
{
  if (!hasValidState()) {
    return;
  }

  const auto & pose = latest_odom_->pose.pose;
  const double x = pose.position.x;
  const double y = pose.position.y;

  tf2::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  const size_t idx = findNearestIndex(x, y);
  const auto & ref = ref_path_[idx];
  const Errors e = computeErrors(x, y, yaw, ref);

  const auto & goal = ref_path_.back();
  const double dist_to_goal = std::hypot(x - goal.x, y - goal.y);
  const double goal_scale = clamp(dist_to_goal / std::max(goal_tolerance_, 1e-3), 0.0, 1.0);

  const double curve_limited_v = w_max_ / (std::abs(ref.kappa) + kappa_speed_eps_);
  const double v_ref = clamp(std::min(base_v_ref_, curve_limited_v) * goal_scale, v_min_, v_max_);
  const double omega_ref = ref.kappa * v_ref;

  updateLQRGain(v_ref);

  double delta_v = 0.0;
  double delta_w = 0.0;
  if (has_valid_k_) {
    Eigen::Vector3d x_e;
    x_e << e.ex, e.ey, e.etheta;
    const Eigen::Vector2d du = -K_ * x_e;
    delta_v = du(0);
    delta_w = du(1);
    no_gain_log_counter_ = 0;
  } else {
    // Safe fallback: keep linear speed on reference and use heading P-control.
    delta_v = 0.0;
    delta_w = -1.2 * e.etheta;
    if (no_gain_log_counter_ % 30 == 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "No valid LQR gain yet. Fallback active. e=(%.3f, %.3f, %.3f), v_ref=%.3f",
        e.ex, e.ey, e.etheta, v_ref);
    }
    ++no_gain_log_counter_;
  }

  double v_cmd = v_ref + delta_v;
  double w_cmd = omega_ref + delta_w;

  v_cmd = clamp(v_cmd, v_min_, v_max_);
  w_cmd = clamp(w_cmd, -w_max_, w_max_);

  const double accel_up = accel_limit_ * dt_;
  const double accel_down = decel_limit_ * dt_;
  if (v_cmd > last_v_cmd_ + accel_up) {
    v_cmd = last_v_cmd_ + accel_up;
  }
  if (v_cmd < last_v_cmd_ - accel_down) {
    v_cmd = last_v_cmd_ - accel_down;
  }

  const double max_dw = w_accel_limit_ * dt_;
  if (w_cmd > last_w_cmd_ + max_dw) {
    w_cmd = last_w_cmd_ + max_dw;
  }
  if (w_cmd < last_w_cmd_ - max_dw) {
    w_cmd = last_w_cmd_ - max_dw;
  }

  if (dist_to_goal < goal_tolerance_) {
    v_cmd = 0.0;
    w_cmd = 0.0;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = v_cmd;
  cmd.angular.z = w_cmd;
  cmd_pub_->publish(cmd);

  last_v_cmd_ = v_cmd;
  last_w_cmd_ = w_cmd;
}

double LQRTrackerNode::normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

double LQRTrackerNode::clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

}  // namespace husky_lqr

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<husky_lqr::LQRTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
