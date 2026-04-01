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


#ifndef HUSKY_LQR__RVIZ_PATH_BUILDER_NODE_HPP_
#define HUSKY_LQR__RVIZ_PATH_BUILDER_NODE_HPP_

#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace husky_lqr
{

class RVizPathBuilderNode : public rclcpp::Node
{
public:
  RVizPathBuilderNode();

private:
  void onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void onClear(const std_msgs::msg::Empty::SharedPtr msg);
  void publishPath();
  void publishMarkers();

  std::vector<geometry_msgs::msg::Point> smoothAndResample(
    const std::vector<geometry_msgs::msg::Point> & raw_points,
    int interpolation_per_segment,
    double resample_step) const;

  geometry_msgs::msg::Point catmullRom(
    const geometry_msgs::msg::Point & p0,
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    const geometry_msgs::msg::Point & p3,
    double t) const;

  static double pointDistance(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b);

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::vector<geometry_msgs::msg::Point> clicked_points_;

  std::string frame_id_;
  double min_point_distance_;
  int interpolation_per_segment_;
  double resample_step_;
};

}  // namespace husky_lqr

#endif  // HUSKY_LQR__RVIZ_PATH_BUILDER_NODE_HPP_
