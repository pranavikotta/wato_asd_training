#include "planner_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot {

PlannerCore::PlannerCore(rclcpp::Logger logger, std::shared_ptr<robot::CostmapCore> costmap)
  : logger_(logger), costmap_(costmap), state_(PlannerState::WAITING_FOR_GOAL), has_goal_(false) {}

void PlannerCore::setCostmap(std::shared_ptr<CostmapCore> costmap) {
  costmap_ = costmap;
}

void PlannerCore::updateMap(const nav_msgs::msg::OccupancyGrid& map) {
  if (costmap_) {
    costmap_->updateMap(map);
  }
}

void PlannerCore::setGoal(const geometry_msgs::msg::PointStamped& goal_msg) {
  goal_ = goal_msg.point;
  has_goal_ = true;
  state_ = PlannerState::WAITING_FOR_ROBOT_TO_REACH_GOAL;
}

void PlannerCore::updateOdom(const nav_msgs::msg::Odometry& odom) {
  robot_pose_ = odom.pose.pose.position;
}

void PlannerCore::clearGoal() {
  has_goal_ = false;
  state_ = PlannerState::WAITING_FOR_GOAL;
}

bool PlannerCore::hasGoal() const {
  return has_goal_;
}

bool PlannerCore::isGoalReached() const {
  double dx = goal_.x - robot_pose_.x;
  double dy = goal_.y - robot_pose_.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5;  // goal tolerance
}

PlannerState PlannerCore::getState() const {
  return state_;
}

nav_msgs::msg::Path PlannerCore::planPath() {
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = rclcpp::Clock().now();

  CellIndex start_idx = worldToMapIndex(robot_pose_.x, robot_pose_.y);
  CellIndex goal_idx = worldToMapIndex(goal_.x, goal_.y);
  std::vector<CellIndex> path = runAStar(start_idx, goal_idx);

  for (const auto& idx : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position = mapIndexToWorld(idx);
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  return path_msg;
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const {
  return std::hypot(a.x - b.x, a.y - b.y);
}

std::vector<CellIndex> PlannerCore::runAStar(const CellIndex& start, const CellIndex& goal) {
  using OpenSet = std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>>;
  OpenSet open_list;

  std::unordered_map<CellIndex, double, CellIndexHash> g_scores;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  open_list.emplace(start, heuristic(start, goal));
  g_scores[start] = 0.0;

  while (!open_list.empty()) {
    CellIndex current = open_list.top().index;
    open_list.pop();

    if (current == goal) {
      std::vector<CellIndex> path;
      while (came_from.find(current) != came_from.end()) {
        path.push_back(current);
        current = came_from[current];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (const auto& neighbor : getNeighbors(current)) {
      double distance = heuristic(current, neighbor);
      double tentative_g = g_scores[current] + distance;

      if (g_scores.find(neighbor) == g_scores.end() || tentative_g < g_scores[neighbor]) {
        came_from[neighbor] = current;
        g_scores[neighbor] = tentative_g;
        double f = tentative_g + heuristic(neighbor, goal);
        open_list.emplace(neighbor, f);
      }
    }
  }

  RCLCPP_WARN(logger_, "A* failed to find a path.");
  return {};
}

std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex& cell) const {
  static const int dx[8] = {-1, -1, -1,  0, 0, 1, 1, 1};
  static const int dy[8] = {-1,  0,  1, -1, 1, -1, 0, 1};
  std::vector<CellIndex> neighbors;

  for (int i = 0; i < 8; ++i) {
    int nx = cell.x + dx[i];
    int ny = cell.y + dy[i];

    if (isValidCell(nx, ny)) {
      neighbors.emplace_back(nx, ny);
    }
  }

  return neighbors;
}

bool PlannerCore::inBounds(int x, int y) const {
  return x >= 0 && y >= 0 &&
         x < costmap_->getWidth() &&
         y < costmap_->getHeight();
}

CellIndex PlannerCore::worldToMapIndex(double wx, double wy) const {
  return costmap_ ? costmap_->worldToMap(wx, wy) : CellIndex(-1, -1);
}

geometry_msgs::msg::Point PlannerCore::mapIndexToWorld(const CellIndex& idx) const {
  return costmap_ ? costmap_->mapToWorld(idx.x, idx.y) : geometry_msgs::msg::Point();
}

bool PlannerCore::isValidCell(int x, int y) const {
  if (!costmap_ || !costmap_->inBounds(x, y)) return false;
  int cost = costmap_->getCost(x, y);
  return cost >= 0 && cost <= 100;  // Avoid unknown and high-cost obstacles
}

}

