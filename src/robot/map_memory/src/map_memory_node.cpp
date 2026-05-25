#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  resolution_ = this->declare_parameter<double>("resolution", 0.1);
  width_ = this->declare_parameter<int>("width", 400);    // 400 * 0.1 = 40m
  height_ = this->declare_parameter<int>("height", 400);
  // Global map centred on the world origin.
  origin_x_ = this->declare_parameter<double>("origin_x", -(width_ * resolution_) / 2.0);
  origin_y_ = this->declare_parameter<double>("origin_y", -(height_ * resolution_) / 2.0);

  // Initialise an empty global map up front so the planner has something to
  // plan on even before the robot moves (per assignment tip).
  global_map_.info.resolution = resolution_;
  global_map_.info.width = width_;
  global_map_.info.height = height_;
  global_map_.info.origin.position.x = origin_x_;
  global_map_.info.origin.position.y = origin_y_;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(width_ * height_, -1);  // unknown

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10,
      std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&MapMemoryNode::updateMap, this));

  RCLCPP_INFO(this->get_logger(), "Map memory node started: %dx%d @ %.2fm/cell",
              width_, height_, resolution_);
}

void MapMemoryNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_received_ = true;
  if (!first_odom_) {
    integrateCostmap();
  }
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // Yaw from quaternion.
  const auto& q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Adopt the odom frame as the global map frame — never hardcoded.
  global_map_.header.frame_id = msg->header.frame_id;

  if (first_odom_) {
    first_odom_ = false;
  }
}

void MapMemoryNode::integrateCostmap() {
  const auto& cm = latest_costmap_;
  int cm_w = cm.info.width;
  int cm_h = cm.info.height;
  double cm_res = cm.info.resolution;
  double cm_ox = cm.info.origin.position.x;
  double cm_oy = cm.info.origin.position.y;

  double cos_y = std::cos(robot_yaw_);
  double sin_y = std::sin(robot_yaw_);

  for (int j = 0; j < cm_h; ++j) {
    for (int i = 0; i < cm_w; ++i) {
      int8_t val = cm.data[j * cm_w + i];
      if (val < 0) continue;  // unknown in local: retain global

      // Local cell centre in the robot frame.
      double lx = cm_ox + (i + 0.5) * cm_res;
      double ly = cm_oy + (j + 0.5) * cm_res;

      // Rotate + translate into the global frame.
      double gx = robot_x_ + lx * cos_y - ly * sin_y;
      double gy = robot_y_ + lx * sin_y + ly * cos_y;

      int mx = static_cast<int>((gx - origin_x_) / resolution_);
      int my = static_cast<int>((gy - origin_y_) / resolution_);
      if (mx < 0 || mx >= width_ || my < 0 || my >= height_) continue;

      int idx = my * width_ + mx;
      // Prioritise new data: keep the stronger obstacle reading.
      int8_t cur = global_map_.data[idx];
      if (cur < 0 || val > cur) global_map_.data[idx] = val;
    }
  }
}

void MapMemoryNode::updateMap() {
  global_map_.header.stamp = this->get_clock()->now();
  map_pub_->publish(global_map_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
