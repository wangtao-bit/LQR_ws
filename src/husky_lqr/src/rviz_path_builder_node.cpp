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


#include "husky_lqr/rviz_path_builder_node.hpp"

#include <algorithm>
#include <cmath>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace husky_lqr
{

RVizPathBuilderNode::RVizPathBuilderNode()
: Node("rviz_path_builder_node")
{
  frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
  min_point_distance_ = this->declare_parameter<double>("min_point_distance", 0.10);
  interpolation_per_segment_ = this->declare_parameter<int>("interpolation_per_segment", 10);
  resample_step_ = this->declare_parameter<double>("resample_step", 0.10);

  point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/clicked_point", rclcpp::QoS(50),
    std::bind(&RVizPathBuilderNode::onClickedPoint, this, std::placeholders::_1));

  clear_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/clear_clicked_points", rclcpp::QoS(10),
    std::bind(&RVizPathBuilderNode::onClear, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/plan", rclcpp::QoS(10));
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/clicked_points_marker", rclcpp::QoS(10));

  RCLCPP_INFO(
    this->get_logger(), "rviz_path_builder_node started. Click in RViz Publish Point tool.");
}

void RVizPathBuilderNode::onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  geometry_msgs::msg::Point p = msg->point;

  if (!clicked_points_.empty()) {
    if (pointDistance(clicked_points_.back(), p) < min_point_distance_) {
      return;
    }
  }

  clicked_points_.push_back(p);
  publishPath();
  publishMarkers();
}

void RVizPathBuilderNode::onClear(const std_msgs::msg::Empty::SharedPtr)
{
  clicked_points_.clear();
  publishPath();
  publishMarkers();
  RCLCPP_INFO(this->get_logger(), "Cleared clicked points");
}

void RVizPathBuilderNode::publishPath()
{
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = frame_id_;

  if (clicked_points_.size() < 2) {
    path_pub_->publish(path);
    return;
  }

  auto points = smoothAndResample(
    clicked_points_, interpolation_per_segment_, resample_step_);

  path.poses.reserve(points.size());
  for (const auto & p : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = p;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  path_pub_->publish(path);
}

void RVizPathBuilderNode::publishMarkers()
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id_;
  m.header.stamp = this->now();
  m.ns = "clicked_points";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.12;
  m.scale.y = 0.12;
  m.scale.z = 0.12;
  m.color.a = 1.0;
  m.color.r = 0.2;
  m.color.g = 0.8;
  m.color.b = 0.2;
  m.points = clicked_points_;
  marker_pub_->publish(m);
}

std::vector<geometry_msgs::msg::Point> RVizPathBuilderNode::smoothAndResample(
  const std::vector<geometry_msgs::msg::Point> & raw_points,
  int interpolation_per_segment,
  double resample_step) const
{
  std::vector<geometry_msgs::msg::Point> dense;
  if (raw_points.size() < 2) {
    return dense;
  }

  if (raw_points.size() == 2) {
    dense = raw_points;
  } else {
    dense.reserve(raw_points.size() * std::max(2, interpolation_per_segment));
    for (size_t i = 0; i + 1 < raw_points.size(); ++i) {
      const auto & p1 = raw_points[i];
      const auto & p2 = raw_points[i + 1];
      const auto & p0 = (i == 0) ? p1 : raw_points[i - 1];
      const auto & p3 = (i + 2 < raw_points.size()) ? raw_points[i + 2] : p2;

      const int n = std::max(2, interpolation_per_segment);
      for (int k = 0; k < n; ++k) {
        const double t = static_cast<double>(k) / static_cast<double>(n);
        dense.push_back(catmullRom(p0, p1, p2, p3, t));
      }
    }
    dense.push_back(raw_points.back());
  }

  std::vector<geometry_msgs::msg::Point> sampled;
  sampled.reserve(dense.size());
  sampled.push_back(dense.front());

  double accum = 0.0;
  for (size_t i = 1; i < dense.size(); ++i) {
    const double ds = pointDistance(dense[i - 1], dense[i]);
    accum += ds;
    if (accum >= resample_step) {
      sampled.push_back(dense[i]);
      accum = 0.0;
    }
  }

  if (pointDistance(sampled.back(), dense.back()) > 1e-6) {
    sampled.push_back(dense.back());
  }
  return sampled;
}

geometry_msgs::msg::Point RVizPathBuilderNode::catmullRom(
  const geometry_msgs::msg::Point & p0,
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3,
  double t) const
{
  const double t2 = t * t;
  const double t3 = t2 * t;

  geometry_msgs::msg::Point out;
  out.x = 0.5 * ((2.0 * p1.x) +
    (-p0.x + p2.x) * t +
    (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2 +
    (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3);

  out.y = 0.5 * ((2.0 * p1.y) +
    (-p0.y + p2.y) * t +
    (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2 +
    (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3);

  out.z = 0.0;
  return out;
}

double RVizPathBuilderNode::pointDistance(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace husky_lqr

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<husky_lqr::RVizPathBuilderNode>());
  rclcpp::shutdown();
  return 0;
}
