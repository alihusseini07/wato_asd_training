#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "planner_node.hpp"

PlannerNode::PlannerNode()
    : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.5);
  obstacle_threshold_ = this->declare_parameter<int>("obstacle_threshold", 30);
  lethal_threshold_ = this->declare_parameter<int>("lethal_threshold", 80);
  cost_weight_ = this->declare_parameter<double>("cost_weight", 5.0);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10,
      std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PlannerNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Planner node started (WAITING_FOR_GOAL)");
}

bool PlannerNode::worldToGrid(double wx, double wy, int& gx, int& gy) const {
  double res = current_map_.info.resolution;
  gx = static_cast<int>((wx - current_map_.info.origin.position.x) / res);
  gy = static_cast<int>((wy - current_map_.info.origin.position.y) / res);
  return gx >= 0 && gx < static_cast<int>(current_map_.info.width) &&
         gy >= 0 && gy < static_cast<int>(current_map_.info.height);
}

void PlannerNode::gridToWorld(int gx, int gy, double& wx, double& wy) const {
  double res = current_map_.info.resolution;
  wx = current_map_.info.origin.position.x + (gx + 0.5) * res;
  wy = current_map_.info.origin.position.y + (gy + 0.5) * res;
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  map_received_ = true;
  // Replan on map updates while pursuing a goal (per assignment).
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "Goal received (%.2f, %.2f) -> PLANNING",
              goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_x_;
  double dy = goal_.point.y - robot_y_;
  return std::hypot(dx, dy) < goal_tolerance_;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached -> WAITING_FOR_GOAL");
      state_ = State::WAITING_FOR_GOAL;
      // Publish an empty path so the controller halts.
      nav_msgs::msg::Path empty;
      empty.header.stamp = this->get_clock()->now();
      empty.header.frame_id = current_map_.header.frame_id;
      path_pub_->publish(empty);
    } else {
      planPath();  // periodic replan / progress check
    }
  }
}

bool PlannerNode::runAStar(double sx, double sy, double gx, double gy,
                           int block_threshold,
                           std::vector<std::pair<double, double>>& path_out) {
  int start_x, start_y, goal_x, goal_y;
  if (!worldToGrid(sx, sy, start_x, start_y)) return false;
  if (!worldToGrid(gx, gy, goal_x, goal_y)) return false;

  int w = current_map_.info.width;
  int h = current_map_.info.height;
  const auto& data = current_map_.data;

  auto blocked = [&](int x, int y) {
    int8_t v = data[y * w + x];
    return v > block_threshold;  // unknown (-1) is treated as free
  };

  CellIndex start(start_x, start_y);
  CellIndex goal(goal_x, goal_y);

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_set<CellIndex, CellIndexHash> closed;

  auto heuristic = [&](const CellIndex& c) {
    return std::hypot(c.x - goal.x, c.y - goal.y);
  };

  g_score[start] = 0.0;
  open.emplace(start, heuristic(start));

  const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool found = false;
  while (!open.empty()) {
    AStarNode cur = open.top();
    open.pop();
    if (cur.index == goal) { found = true; break; }
    if (closed.count(cur.index)) continue;
    closed.insert(cur.index);

    for (int k = 0; k < 8; ++k) {
      int nx = cur.index.x + dx8[k];
      int ny = cur.index.y + dy8[k];
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      if (blocked(nx, ny)) continue;
      CellIndex nb(nx, ny);
      if (closed.count(nb)) continue;
      double step = (dx8[k] == 0 || dy8[k] == 0) ? 1.0 : 1.41421356;
      double tentative = g_score[cur.index] + step;
      auto it = g_score.find(nb);
      if (it == g_score.end() || tentative < it->second) {
        g_score[nb] = tentative;
        came_from[nb] = cur.index;
        open.emplace(nb, tentative + heuristic(nb));
      }
    }
  }

  if (!found) return false;

  // Reconstruct path from goal back to start.
  std::vector<CellIndex> cells;
  CellIndex c = goal;
  while (c != start) {
    cells.push_back(c);
    c = came_from[c];
  }
  cells.push_back(start);
  std::reverse(cells.begin(), cells.end());

  path_out.clear();
  for (const auto& cell : cells) {
    double wx, wy;
    gridToWorld(cell.x, cell.y, wx, wy);
    path_out.emplace_back(wx, wy);
  }
  return true;
}

void PlannerNode::planPath() {
  if (!goal_received_ || !map_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan: missing map or goal");
    return;
  }

  std::vector<std::pair<double, double>> waypoints;
  if (!runAStar(robot_x_, robot_y_, goal_.point.x, goal_.point.y, waypoints)) {
    RCLCPP_WARN(this->get_logger(), "A* failed to find a path; halting robot");
    // Publish an empty path so the controller stops instead of following a
    // stale path into an obstacle.
    nav_msgs::msg::Path empty;
    empty.header.stamp = this->get_clock()->now();
    empty.header.frame_id = current_map_.header.frame_id;
    path_pub_->publish(empty);
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = current_map_.header.frame_id;  // propagate map frame
  for (const auto& [wx, wy] : waypoints) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
