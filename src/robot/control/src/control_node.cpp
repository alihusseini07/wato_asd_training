#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode()
    : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 1.0);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.3);
  linear_speed_ = this->declare_parameter<double>("linear_speed", 0.5);
  max_angular_speed_ = this->declare_parameter<double>("max_angular_speed", 1.0);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ControlNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Control node started (pure pursuit)");
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::optional<geometry_msgs::msg::PoseStamped>
ControlNode::findLookaheadPoint() {
  double rx = robot_odom_->pose.pose.position.x;
  double ry = robot_odom_->pose.pose.position.y;

  // First waypoint at least lookahead_distance_ away from the robot.
  for (const auto& pose : current_path_->poses) {
    double d = std::hypot(pose.pose.position.x - rx, pose.pose.position.y - ry);
    if (d >= lookahead_distance_) return pose;
  }
  // None far enough: target the final waypoint (terminal approach).
  if (!current_path_->poses.empty()) return current_path_->poses.back();
  return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(
    const geometry_msgs::msg::PoseStamped& target) {
  geometry_msgs::msg::Twist cmd;

  double rx = robot_odom_->pose.pose.position.x;
  double ry = robot_odom_->pose.pose.position.y;
  double yaw = extractYaw(robot_odom_->pose.pose.orientation);

  // Transform the lookahead point into the robot frame.
  double dx = target.pose.position.x - rx;
  double dy = target.pose.position.y - ry;
  double x_local = std::cos(-yaw) * dx - std::sin(-yaw) * dy;
  double y_local = std::sin(-yaw) * dx + std::cos(-yaw) * dy;

  double ld = std::hypot(x_local, y_local);
  if (ld < 1e-6) return cmd;  // on top of target

  // Target is behind: spin in place toward it instead of driving away.
  if (x_local < 0.0) {
    cmd.linear.x = 0.0;
    cmd.angular.z = std::clamp(
        (y_local >= 0.0 ? 1.0 : -1.0) * max_angular_speed_,
        -max_angular_speed_, max_angular_speed_);
    return cmd;
  }

  // Pure pursuit curvature.
  double curvature = 2.0 * y_local / (ld * ld);
  double angular = linear_speed_ * curvature;
  angular = std::clamp(angular, -max_angular_speed_, max_angular_speed_);

  cmd.linear.x = linear_speed_;
  cmd.angular.z = angular;
  return cmd;
}

void ControlNode::controlLoop() {
  geometry_msgs::msg::Twist stop;  // zero velocity

  if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
    cmd_vel_pub_->publish(stop);
    return;
  }

  // Stop if within tolerance of the final waypoint.
  const auto& goal = current_path_->poses.back().pose.position;
  double dist_to_goal = std::hypot(
      goal.x - robot_odom_->pose.pose.position.x,
      goal.y - robot_odom_->pose.pose.position.y);
  if (dist_to_goal < goal_tolerance_) {
    cmd_vel_pub_->publish(stop);
    return;
  }

  auto target = findLookaheadPoint();
  if (!target) {
    cmd_vel_pub_->publish(stop);
    return;
  }

  cmd_vel_pub_->publish(computeVelocity(*target));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
