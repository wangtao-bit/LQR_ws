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


#ifndef HUSKY_LQR__LQR_TRACKER_NODE_HPP_
#define HUSKY_LQR__LQR_TRACKER_NODE_HPP_

#include <memory>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "husky_lqr/unified_lqr_solver.hpp"

namespace husky_lqr
{

class LQRTrackerNode : public rclcpp::Node
{
public:
  LQRTrackerNode();

private:
  struct RefPoint
  {
    double x;
    double y;
    double theta;
    double kappa;
  };

  struct Errors
  {
    double ex;
    double ey;
    double etheta;
  };

  void onPath(const nav_msgs::msg::Path::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onControlTimer();

  bool hasValidState() const;
  std::vector<RefPoint> toRefPath(const nav_msgs::msg::Path & path) const;
  size_t findNearestIndex(double x, double y) const;
  Errors computeErrors(double x, double y, double yaw, const RefPoint & ref) const;
  void updateLQRGain(double v_ref);
  static double normalizeAngle(double a);
  static double clamp(double v, double lo, double hi);

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  std::vector<RefPoint> ref_path_;

  UnifiedLQRSolver solver_;
  UnifiedLQRSolver::MatrixK K_;
  bool has_valid_k_ {false};

  double dt_;
  double control_rate_;
  double lookahead_dist_;
  double goal_tolerance_;
  double min_lqr_speed_;
  double kappa_speed_eps_;

  double base_v_ref_;
  double v_min_;
  double v_max_;
  double w_max_;
  double accel_limit_;
  double decel_limit_;
  double w_accel_limit_;

  double q_ex_;
  double q_ey_;
  double q_etheta_;
  double r_v_;
  double r_w_;
  int no_gain_log_counter_ {0};

  double last_v_cmd_ {0.0};
  double last_w_cmd_ {0.0};
};

}  // namespace husky_lqr

#endif  // HUSKY_LQR__LQR_TRACKER_NODE_HPP_
