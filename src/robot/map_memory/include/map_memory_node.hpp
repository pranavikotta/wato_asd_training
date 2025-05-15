#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    // geometry_msgs::msg::Pose latest_pose_;

    bool costmap_updated_ = false;
    bool should_update_map_ = false;

    // double last_x_ = 0.0;
    // double last_y_ = 0.0;
    // const double distance_threshold_ = 1.5;

    robot::MapMemoryCore map_memory_;
};

#endif


// // #ifndef MAP_MEMORY_NODE_HPP_
// // #define MAP_MEMORY_NODE_HPP_

// // #include "rclcpp/rclcpp.hpp"

// // #include "map_memory_core.hpp"

// // class MapMemoryNode : public rclcpp::Node {
// //   public:
// //     MapMemoryNode();

// //   private:
// //     robot::MapMemoryCore map_memory_;
// // };

// // #endif 
