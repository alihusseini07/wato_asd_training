#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

    // LaserScan callback: builds and publishes a local costmap.
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  private:
    // Convert a polar laser reading to a grid cell, marking it occupied.
    void markObstacle(double range, double angle, std::vector<int8_t>& grid);
    // Inflate occupied cells outwards within the inflation radius.
    void inflateObstacles(std::vector<int8_t>& grid);

    robot::CostmapCore costmap_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    // Costmap geometry (parameters).
    double resolution_;        // meters per cell
    int size_;                 // grid is size_ x size_ cells
    double origin_x_;          // world coord of cell (0,0), robot-centred
    double origin_y_;
    double inflation_radius_;  // meters
    int max_cost_;             // cost assigned to an occupied cell
};

#endif
