#include <chrono>
#include <cmath>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
    : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Parameters (overridable from config/params.yaml).
  resolution_ = this->declare_parameter<double>("resolution", 0.1);
  size_ = this->declare_parameter<int>("size", 200);  // 200 cells * 0.1 = 20m
  inflation_radius_ = this->declare_parameter<double>("inflation_radius", 1.0);
  max_cost_ = this->declare_parameter<int>("max_cost", 100);

  // Robot-centred grid: cell (0,0) is bottom-left, robot sits at the centre.
  origin_x_ = -(size_ * resolution_) / 2.0;
  origin_y_ = -(size_ * resolution_) / 2.0;

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10,
      std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  RCLCPP_INFO(this->get_logger(), "Costmap node started: %dx%d @ %.2fm/cell",
              size_, size_, resolution_);
}

void CostmapNode::markObstacle(double range, double angle,
                               std::vector<int8_t>& grid) {
  // Polar -> Cartesian in the laser frame.
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);

  // Cartesian -> grid indices.
  int gx = static_cast<int>((x - origin_x_) / resolution_);
  int gy = static_cast<int>((y - origin_y_) / resolution_);

  if (gx >= 0 && gx < size_ && gy >= 0 && gy < size_) {
    grid[gy * size_ + gx] = static_cast<int8_t>(max_cost_);
  }
}

void CostmapNode::inflateObstacles(std::vector<int8_t>& grid) {
  int radius_cells = static_cast<int>(inflation_radius_ / resolution_);
  // Snapshot of occupied cells so inflation doesn't cascade off itself.
  std::vector<std::pair<int, int>> occupied;
  for (int y = 0; y < size_; ++y) {
    for (int x = 0; x < size_; ++x) {
      if (grid[y * size_ + x] == static_cast<int8_t>(max_cost_)) {
        occupied.emplace_back(x, y);
      }
    }
  }

  for (const auto& [ox, oy] : occupied) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        int nx = ox + dx;
        int ny = oy + dy;
        if (nx < 0 || nx >= size_ || ny < 0 || ny >= size_) continue;
        double dist = std::sqrt(dx * dx + dy * dy) * resolution_;
        if (dist > inflation_radius_) continue;
        int cost = static_cast<int>(max_cost_ * (1.0 - dist / inflation_radius_));
        int idx = ny * size_ + nx;
        if (cost > grid[idx]) grid[idx] = static_cast<int8_t>(cost);
      }
    }
  }
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Step 1: initialise an empty (free) costmap.
  std::vector<int8_t> grid(size_ * size_, 0);

  // Step 2: mark obstacles from valid laser returns.
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    if (std::isnan(range) || range < scan->range_min || range > scan->range_max) {
      continue;
    }
    double angle = scan->angle_min + i * scan->angle_increment;
    markObstacle(range, angle, grid);
  }

  // Step 3: inflate.
  inflateObstacles(grid);

  // Step 4: publish. Frame is propagated from the incoming scan — never hardcoded.
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = scan->header.stamp;
  msg.header.frame_id = scan->header.frame_id;
  msg.info.resolution = resolution_;
  msg.info.width = size_;
  msg.info.height = size_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0;
  msg.data = grid;

  costmap_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
