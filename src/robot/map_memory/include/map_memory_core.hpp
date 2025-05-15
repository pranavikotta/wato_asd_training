#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    void initializeGlobalMap();
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap, const geometry_msgs::msg::Pose& robot_pose);
    nav_msgs::msg::OccupancyGrid getGlobalMap() const;
    
  private:
    rclcpp::Logger logger_;

    nav_msgs::msg::OccupancyGrid global_map_;
    bool is_initialized_ = false;

    void transformCostmapToGlobalFrame(const nav_msgs::msg::OccupancyGrid& costmap,
    const geometry_msgs::msg::Pose& robot_pose, nav_msgs::msg::OccupancyGrid& global_costmap);
};

}  

#endif

// #ifndef MAP_MEMORY_CORE_HPP_
// #define MAP_MEMORY_CORE_HPP_

// #include "rclcpp/rclcpp.hpp"

// namespace robot
// {

// class MapMemoryCore {
//   public:
//     explicit MapMemoryCore(const rclcpp::Logger& logger);

//   private:
//     rclcpp::Logger logger_;
// };

// }  

// #endif  
