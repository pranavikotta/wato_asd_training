#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
 
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishMessage();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
 
  private:
    //costmap parameters
    double resolution_;
    int width_;
    int height_;
    double inflation_radius_;
    double max_cost_;  

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    robot::CostmapCore costmap_;
};
 
#endif

// #ifndef COSTMAP_NODE_HPP_
// #define COSTMAP_NODE_HPP_

// #include "rclcpp/rclcpp.hpp"

// #include "costmap_core.hpp"

// class CostmapNode : public rclcpp::Node {
//   public:
//     CostmapNode();

//   private:
//     robot::CostmapCore costmap_;
// };

// #endif 