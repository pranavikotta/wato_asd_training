#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode()
: Node("costmap"),
  costmap_pub_(this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10)),
  costmap_(this->get_logger(), costmap_pub_)
{
  resolution_ = 0.1;
  width_ = 100;
  height_ = 100;
  inflation_radius_ = 1.0;
  max_cost_ = 100;

  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1)
  );

  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}


void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      // Step 1: Initialize costmap
      costmap_.initializeCostmap();
 
      // Step 2: Convert LaserScan to grid and mark obstacles
      for (size_t i = 0; i < msg->ranges.size(); ++i) {
          double angle = msg->angle_min + i * msg->angle_increment;
          double range = msg->ranges[i];
          if (range < msg->range_max && range > msg->range_min) {
              // Calculate grid coordinates
              int x_grid, y_grid;
              costmap_.convertToGrid(range, angle, x_grid, y_grid);
              costmap_.markObstacle(x_grid, y_grid);
          }
      }
   
      // Step 3: Inflate obstacles
      costmap_.inflateObstacles();
   
      // Step 4: Publish costmap
      costmap_.publishCostmap(this->now());
  
  RCLCPP_INFO(this->get_logger(), "Received LaserScan with %ld ranges", msg->ranges.size());
}

void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}


// #include <chrono>
// #include <memory>

// #include "costmap_node.hpp"

// CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {}

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<CostmapNode>());
//   rclcpp::shutdown();
//   return 0;
// }