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
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class BowCoveragePathPublisher : public rclcpp::Node
{
public:
  BowCoveragePathPublisher()
  : Node("bow_coverage_path_publisher")
  {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    lane_spacing_ = this->declare_parameter<double>("lane_spacing", 0.8);
    length_ = this->declare_parameter<double>("length", 12.0);
    width_ = this->declare_parameter<double>("width", 4.0);
    turn_radius_ = this->declare_parameter<double>("turn_radius", 1.0);
    resolution_ = this->declare_parameter<double>("resolution", 0.05);
    publish_rate_ = this->declare_parameter<double>("publish_rate", 1.0);
    pass_count_ = this->declare_parameter<int>("pass_count", 0);
    closed_loop_ = this->declare_parameter<bool>("closed_loop", false);
    origin_x_ = this->declare_parameter<double>("origin_x", 0.0);
    origin_y_ = this->declare_parameter<double>("origin_y", 0.0);

    if (length_ <= 0.0) {
      length_ = 12.0;
    }
    if (width_ <= 0.0) {
      width_ = lane_spacing_;
    }
    if (lane_spacing_ <= 0.0) {
      lane_spacing_ = 0.8;
    }
    if (resolution_ <= 0.0) {
      resolution_ = 0.05;
    }
    if (turn_radius_ < lane_spacing_ * 0.5) {
      turn_radius_ = lane_spacing_ * 0.5;
    }

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/plan", rclcpp::QoS(10));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/bow_coverage_marker", rclcpp::QoS(10));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_)),
      std::bind(&BowCoveragePathPublisher::publishPath, this));

    RCLCPP_INFO(this->get_logger(), "bow_coverage_path_publisher started");
  }

private:
  void appendLine(
    std::vector<geometry_msgs::msg::PoseStamped> & poses,
    double x0, double y0, double x1, double y1) const
  {
    const double dist = std::hypot(x1 - x0, y1 - y0);
    const int steps = std::max(1, static_cast<int>(std::ceil(dist / resolution_)));
    const double yaw = std::atan2(y1 - y0, x1 - x0);

    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = frame_id_;
      p.pose.position.x = x0 + (x1 - x0) * t;
      p.pose.position.y = y0 + (y1 - y0) * t;
      p.pose.orientation.z = std::sin(yaw * 0.5);
      p.pose.orientation.w = std::cos(yaw * 0.5);
      if (!poses.empty()) {
        const auto & prev = poses.back().pose.position;
        if (std::hypot(prev.x - p.pose.position.x, prev.y - p.pose.position.y) < 1e-4) {
          continue;
        }
      }
      poses.push_back(p);
    }
  }

  std::vector<geometry_msgs::msg::PoseStamped> buildBowCoverage() const
  {
    std::vector<geometry_msgs::msg::PoseStamped> poses;

    int lane_count = std::max(2, static_cast<int>(std::floor(width_ / lane_spacing_)) + 1);
    if (pass_count_ > 1) {
      lane_count = pass_count_;
    }
    const double y_min = origin_y_;
    const double x_left = origin_x_;
    const double x_right = origin_x_ + length_;

    for (int lane = 0; lane < lane_count; ++lane) {
      const double y = y_min + lane * lane_spacing_;
      const bool forward = (lane % 2 == 0);

      if (forward) {
        appendLine(poses, x_left, y, x_right, y);
      } else {
        appendLine(poses, x_right, y, x_left, y);
      }

      if (lane == lane_count - 1) {
        break;
      }

      const double y_next = y_min + (lane + 1) * lane_spacing_;
      const double turn_x = forward ? x_right : x_left;
      appendLine(poses, turn_x, y, turn_x, y_next);
    }

    if (closed_loop_ && poses.size() > 2) {
      const auto & first = poses.front().pose.position;
      const auto & last = poses.back().pose.position;
      appendLine(poses, last.x, last.y, first.x, first.y);
    }

    return poses;
  }

  void publishPath()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = frame_id_;

    auto poses = buildBowCoverage();
    for (auto & p : poses) {
      p.header.stamp = path.header.stamp;
      path.poses.push_back(p);
    }

    path_pub_->publish(path);

    visualization_msgs::msg::Marker m;
    m.header = path.header;
    m.ns = "bow_coverage";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.05;
    m.color.a = 1.0;
    m.color.r = 0.1;
    m.color.g = 0.8;
    m.color.b = 1.0;
    for (const auto & p : path.poses) {
      m.points.push_back(p.pose.position);
    }
    marker_pub_->publish(m);
  }

  std::string frame_id_;
  double lane_spacing_;
  double length_;
  double width_;
  double turn_radius_;
  double resolution_;
  double publish_rate_;
  int pass_count_;
  bool closed_loop_;
  double origin_x_;
  double origin_y_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BowCoveragePathPublisher>());
  rclcpp::shutdown();
  return 0;
}
