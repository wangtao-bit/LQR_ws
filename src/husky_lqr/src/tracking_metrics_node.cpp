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

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

class TrackingMetricsNode : public rclcpp::Node
{
public:
  TrackingMetricsNode()
  : Node("tracking_metrics_node")
  {
    report_rate_ = this->declare_parameter<double>("report_rate", 1.0);

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", rclcpp::QoS(10),
      std::bind(&TrackingMetricsNode::onPath, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(20),
      std::bind(&TrackingMetricsNode::onOdom, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(0.1, report_rate_)),
      std::bind(&TrackingMetricsNode::onReport, this));
  }

private:
  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    path_ = msg;
    resetStats();
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!path_ || path_->poses.empty()) {
      return;
    }

    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    double best = std::numeric_limits<double>::max();
    size_t best_idx = 0;
    for (size_t i = 0; i < path_->poses.size(); ++i) {
      const auto & ps = path_->poses[i];
      const double dx = x - ps.pose.position.x;
      const double dy = y - ps.pose.position.y;
      const double dist = std::hypot(dx, dy);
      if (dist < best) {
        best = dist;
        best_idx = i;
      }
    }

    if (!std::isfinite(best)) {
      return;
    }

    ++count_;
    sum_sq_ += best * best;
    max_err_ = std::max(max_err_, best);
    furthest_idx_ = std::max(furthest_idx_, best_idx);

    if (path_->poses.size() >= 2) {
      size_t next_idx = best_idx + 1;
      if (next_idx >= path_->poses.size()) {
        next_idx = best_idx - 1;
      }
      const auto & p0 = path_->poses[best_idx].pose.position;
      const auto & p1 = path_->poses[next_idx].pose.position;
      const double path_yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
      const double e_theta = std::atan2(std::sin(yaw - path_yaw), std::cos(yaw - path_yaw));
      max_heading_err_ = std::max(max_heading_err_, std::abs(e_theta));
    }
  }

  void onReport()
  {
    if (count_ == 0) {
      return;
    }

    const double rms = std::sqrt(sum_sq_ / static_cast<double>(count_));
    const double completion = path_->poses.size() > 1 ?
      static_cast<double>(furthest_idx_) / static_cast<double>(path_->poses.size() - 1) : 0.0;
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 2000,
      "tracking metrics: samples=%zu, e_y_rms=%.3f, e_y_max=%.3f, "
      "e_theta_max=%.3f rad, completion=%.1f%%",
      count_, rms, max_err_, max_heading_err_, completion * 100.0);
  }

  void resetStats()
  {
    count_ = 0;
    sum_sq_ = 0.0;
    max_err_ = 0.0;
    max_heading_err_ = 0.0;
    furthest_idx_ = 0;
  }

  double report_rate_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path::SharedPtr path_;

  size_t count_ {0};
  double sum_sq_ {0.0};
  double max_err_ {0.0};
  double max_heading_err_ {0.0};
  size_t furthest_idx_ {0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackingMetricsNode>());
  rclcpp::shutdown();
  return 0;
}
