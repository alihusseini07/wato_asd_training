#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

// -------------------- Supporting Structures (from assignment) --------------------
struct CellIndex {
  int x;
  int y;
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
  bool operator==(const CellIndex& other) const {
    return (x == other.x && y == other.y);
  }
  bool operator!=(const CellIndex& other) const {
    return (x != other.x || y != other.y);
  }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex& idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode {
  CellIndex index;
  double f_score;  // f = g + h
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) {
    return a.f_score > b.f_score;  // min-heap on f_score
  }
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    bool goalReached();
    void planPath();
    // A* on current_map_. Returns world-frame waypoints (empty on failure).
    bool runAStar(double sx, double sy, double gx, double gy,
                  std::vector<std::pair<double, double>>& path_out);

    // World <-> grid helpers using current_map_ metadata.
    bool worldToGrid(double wx, double wy, int& gx, int& gy) const;
    void gridToWorld(int gx, int gy, double& wx, double& wy) const;

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::WAITING_FOR_GOAL;
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    bool goal_received_ = false;
    bool map_received_ = false;

    double goal_tolerance_;
    int obstacle_threshold_;  // grid value above which a cell is blocked
};

#endif
