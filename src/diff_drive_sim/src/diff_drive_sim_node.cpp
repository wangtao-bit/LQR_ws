#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

class DiffDriveSimNode : public rclcpp::Node
{
public:
  DiffDriveSimNode()
  : Node("diff_drive_sim_node"),
    x_(0.0),
    y_(0.0),
    yaw_(0.0),
    linear_cmd_(0.0),
    angular_cmd_(0.0),
    left_wheel_pos_(0.0),
    right_wheel_pos_(0.0)
  {
    wheel_base_ = this->declare_parameter<double>("wheel_base", 0.50);
    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.10);
    update_rate_ = this->declare_parameter<double>("update_rate", 50.0);
    cmd_timeout_ = this->declare_parameter<double>("cmd_timeout", 0.50);
    odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_link");
    left_wheel_joint_name_ =
      this->declare_parameter<std::string>("left_wheel_joint_name", "left_wheel_joint");
    right_wheel_joint_name_ =
      this->declare_parameter<std::string>("right_wheel_joint_name", "right_wheel_joint");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&DiffDriveSimNode::on_cmd_vel, this, std::placeholders::_1));

    last_cmd_time_ = this->now();
    last_update_time_ = this->now();

    const auto period = std::chrono::duration<double>(1.0 / std::max(update_rate_, 1.0));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DiffDriveSimNode::on_timer, this));
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    linear_cmd_ = msg->linear.x;
    angular_cmd_ = msg->angular.z;
    last_cmd_time_ = this->now();
  }

  void on_timer()
  {
    const rclcpp::Time now = this->now();
    double dt = (now - last_update_time_).seconds();
    if (dt <= 0.0) {
      return;
    }
    last_update_time_ = now;

    double v = linear_cmd_;
    double w = angular_cmd_;
    if ((now - last_cmd_time_).seconds() > cmd_timeout_) {
      v = 0.0;
      w = 0.0;
    }

    x_ += v * std::cos(yaw_) * dt;
    y_ += v * std::sin(yaw_) * dt;
    yaw_ += w * dt;

    const double half_base = wheel_base_ * 0.5;
    const double v_left = v - w * half_base;
    const double v_right = v + w * half_base;
    const double w_left = v_left / std::max(wheel_radius_, 1e-6);
    const double w_right = v_right / std::max(wheel_radius_, 1e-6);

    left_wheel_pos_ += w_left * dt;
    right_wheel_pos_ += w_right * dt;

    publish_odometry(now, v, w);
    publish_tf(now);
    publish_joint_states(now, w_left, w_right);
  }

  void publish_odometry(const rclcpp::Time & stamp, double v, double w)
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    const double half_yaw = yaw_ * 0.5;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = std::sin(half_yaw);
    odom_msg.pose.pose.orientation.w = std::cos(half_yaw);

    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = w;

    odom_pub_->publish(odom_msg);
  }

  void publish_tf(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = stamp;
    transform_msg.header.frame_id = odom_frame_id_;
    transform_msg.child_frame_id = base_frame_id_;

    transform_msg.transform.translation.x = x_;
    transform_msg.transform.translation.y = y_;
    transform_msg.transform.translation.z = 0.0;

    const double half_yaw = yaw_ * 0.5;
    transform_msg.transform.rotation.x = 0.0;
    transform_msg.transform.rotation.y = 0.0;
    transform_msg.transform.rotation.z = std::sin(half_yaw);
    transform_msg.transform.rotation.w = std::cos(half_yaw);

    tf_broadcaster_->sendTransform(transform_msg);
  }

  void publish_joint_states(const rclcpp::Time & stamp, double w_left, double w_right)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    msg.name = {left_wheel_joint_name_, right_wheel_joint_name_};
    msg.position = {left_wheel_pos_, right_wheel_pos_};
    msg.velocity = {w_left, w_right};
    joint_state_pub_->publish(msg);
  }

  double x_;
  double y_;
  double yaw_;
  double linear_cmd_;
  double angular_cmd_;
  double left_wheel_pos_;
  double right_wheel_pos_;

  double wheel_base_;
  double wheel_radius_;
  double update_rate_;
  double cmd_timeout_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string left_wheel_joint_name_;
  std::string right_wheel_joint_name_;

  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_update_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffDriveSimNode>());
  rclcpp::shutdown();
  return 0;
}
