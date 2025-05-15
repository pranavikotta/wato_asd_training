#include "map_memory_core.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

void MapMemoryCore::initializeGlobalMap() {
    global_map_.header.frame_id = "map";  // or "sim_world" if you're simulating

    global_map_.info.resolution = 0.6;
    global_map_.info.width = 50;
    global_map_.info.height = 50;

    // Center the map at (0, 0)
    global_map_.info.origin.position.x = -0.5 * global_map_.info.width * global_map_.info.resolution;
    global_map_.info.origin.position.y = -0.5 * global_map_.info.height * global_map_.info.resolution;
    global_map_.info.origin.position.z = 0.0;
    global_map_.info.origin.orientation.w = 1.0;

    global_map_.data.resize(global_map_.info.width * global_map_.info.height, -1); // unknown by default

    is_initialized_ = true;
}

void MapMemoryCore::transformCostmapToGlobalFrame(const nav_msgs::msg::OccupancyGrid& costmap,
  const geometry_msgs::msg::Pose& robot_pose, nav_msgs::msg::OccupancyGrid& global_costmap) {

    double resolution = costmap.info.resolution;
    double robot_x = robot_pose.position.x;
    double robot_y = robot_pose.position.y;
  
    tf2::Quaternion q(
      robot_pose.orientation.x,
      robot_pose.orientation.y,
      robot_pose.orientation.z,
      robot_pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  
    for (size_t y = 0; y < costmap.info.height; ++y) {
      for (size_t x = 0; x < costmap.info.width; ++x) {
        size_t idx = y * costmap.info.width + x;
  
        double local_x = (x - costmap.info.width / 2.0) * resolution;
        double local_y = (y - costmap.info.height / 2.0) * resolution;
  
        double global_x = robot_x + local_x * std::cos(yaw) - local_y * std::sin(yaw);
        double global_y = robot_y + local_x * std::sin(yaw) + local_y * std::cos(yaw);
  
        int gx = static_cast<int>((global_x - global_costmap.info.origin.position.x) / resolution);
        int gy = static_cast<int>((global_y - global_costmap.info.origin.position.y) / resolution);
  
        if (gx >= 0 && gx < static_cast<int>(global_costmap.info.width) &&
            gy >= 0 && gy < static_cast<int>(global_costmap.info.height)) {
          size_t global_idx = gy * global_costmap.info.width + gx;
  
          int8_t value = costmap.data[idx];
          if (value != -1) {
            global_costmap.data[global_idx] = value;
          }
        }
      }
    }
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap, const geometry_msgs::msg::Pose& robot_pose) {
    if (!is_initialized_) {
        initializeGlobalMap();  // Now uses static map size and centered origin
    }

    nav_msgs::msg::OccupancyGrid transformed_costmap = global_map_; // Copy size and metadata
    transformCostmapToGlobalFrame(costmap, robot_pose, transformed_costmap);

    for (size_t y = 0; y < transformed_costmap.info.height; ++y) {
        for (size_t x = 0; x < transformed_costmap.info.width; ++x) {
            size_t idx = y * transformed_costmap.info.width + x;

            int8_t new_val = transformed_costmap.data[idx];
            int8_t& current_val = global_map_.data[idx];

            if (new_val != -1) { // only update with known values
                if (current_val == -1) {
                    current_val = new_val;
                } else {
                    current_val = std::max(current_val, new_val);
                }
            }
        }
    }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const {
    return global_map_;  // Simply return the global map
}

} 


// #include "map_memory_core.hpp"

// namespace robot
// {

// MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
//   : logger_(logger) {}

// } 
