#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "planner_core.hpp"
#include "costmap_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    rclcpp::Logger logger_;  
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    std::shared_ptr<robot::CostmapCore> costmap_;
    std::unique_ptr<robot::PlannerCore> planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
};

#endif 



// #ifndef PLANNER_NODE_HPP_
// #define PLANNER_NODE_HPP_

// #include "rclcpp/rclcpp.hpp"

// #include "planner_core.hpp"

// class PlannerNode : public rclcpp::Node {
//   public:
//     PlannerNode();

//   private:
//     robot::PlannerCore planner_;
// };

// #endif 
