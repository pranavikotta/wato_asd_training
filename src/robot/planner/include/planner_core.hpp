#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "costmap_core.hpp"

#include <memory>
#include <vector>
#include <unordered_map>
#include <utility>
#include <queue>

namespace robot {

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode {
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}

  bool operator>(const AStarNode& other) const {
    return f_score > other.f_score;
  }
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b) const{
    return a.f_score > b.f_score;
  }
};

enum class PlannerState {
  WAITING_FOR_GOAL,
  WAITING_FOR_ROBOT_TO_REACH_GOAL
};

class PlannerCore {
  public:
    PlannerCore(rclcpp::Logger logger, std::shared_ptr<robot::CostmapCore> costmap);

    void setCostmap(std::shared_ptr<CostmapCore> costmap);
    void updateMap(const nav_msgs::msg::OccupancyGrid& map);
    void setGoal(const geometry_msgs::msg::PointStamped& goal);
    void updateOdom(const nav_msgs::msg::Odometry& odom);
    void clearGoal();
    bool hasGoal() const;
    bool isGoalReached() const;
    PlannerState getState() const;
    nav_msgs::msg::Path planPath();

  private:
    rclcpp::Logger logger_;
    std::shared_ptr<CostmapCore> costmap_;
    PlannerState state_;

    double heuristic(const CellIndex& a, const CellIndex& b) const;
    std::vector<CellIndex> runAStar(const CellIndex& start, const CellIndex& goal);
    std::vector<CellIndex> getNeighbors(const CellIndex& cell) const;
    bool isValidCell(int x, int y) const;

    bool inBounds(int x, int y) const;
    CellIndex worldToMapIndex(double wx, double wy) const;
    geometry_msgs::msg::Point mapIndexToWorld(const CellIndex& idx) const;

    geometry_msgs::msg::Point goal_;
    geometry_msgs::msg::Point robot_pose_;
    bool has_goal_;
  };

}

#endif

