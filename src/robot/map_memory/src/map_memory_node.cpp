#include "map_memory_node.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

MapMemoryNode::MapMemoryNode()
: Node("map_memory"),
  map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  // Initialize TF2 buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscriptions
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr /* msg */) {
  // No need to track pose manually anymore
  should_update_map_ = true;
}

void MapMemoryNode::updateMap() {
  if (!should_update_map_ || !costmap_updated_) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  if (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero)) {
  RCLCPP_WARN(this->get_logger(), "Transform not yet available: map -> base_link");
  return;
  }
  try {
    // Attempt to get the robot pose in the map frame
    transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
    return;
  }

  // Convert the transform to a pose
  geometry_msgs::msg::Pose robot_pose;
  robot_pose.position.x = transform.transform.translation.x;
  robot_pose.position.y = transform.transform.translation.y;
  robot_pose.position.z = transform.transform.translation.z;
  robot_pose.orientation = transform.transform.rotation;

  // Integrate the costmap at the current robot pose
  map_memory_.integrateCostmap(latest_costmap_, robot_pose);

  // Publish the updated global map
  map_pub_->publish(map_memory_.getGlobalMap());

  // Reset flags
  should_update_map_ = false;
  costmap_updated_ = false;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}



// // #include "map_memory_node.hpp"
// // #include "geometry_msgs/msg/pose_stamped.hpp"
// // #include <tf2_ros/buffer_interface.h>
// // #include <tf2_ros/transform_listener.h>

// // // , tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())), tf_listener_(tf_buffer_, *this)

// // MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
// //   tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
// //   tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
// //   costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
// //     "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
// //   odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
// //       "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

// //   map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

// //   timer_ = this->create_wall_timer(
// //       std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
// // }

// // void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
// //   latest_costmap_ = *msg;
// //   costmap_updated_ = true;
// // }

// // void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
// //   // latest_pose_ = msg->pose.pose;
  
// //   // double x = msg->pose.pose.position.x;
// //   // double y = msg->pose.pose.position.y;

// //   // double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));
// //   // if (distance >= distance_threshold_) {
// //   //     last_x_ = x;
// //   //     last_y_ = y;
// //   //     should_update_map_ = true;
// //   // }
// //   should_update_map_ = true;
// // }

// // void MapMemoryNode::updateMap() {
// //   geometry_msgs::msg::TransformStamped transform;
// // try {
// //     transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
// // } catch (tf2::TransformException &ex) {
// //     RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
// //     return;
// // }

// // // Convert transform to pose
// // geometry_msgs::msg::Pose robot_pose;
// // robot_pose.position.x = transform.transform.translation.x;
// // robot_pose.position.y = transform.transform.translation.y;
// // robot_pose.position.z = transform.transform.translation.z;
// // robot_pose.orientation = transform.transform.rotation;

// // // Now use this in integration
// // map_memory_.integrateCostmap(latest_costmap_, robot_pose);
// // map_pub_->publish(map_memory_.getGlobalMap());
// // should_update_map_ = false;
// //   if (should_update_map_ && costmap_updated_) {
// //     geometry_msgs::msg::Pose robot_pose;
// //     robot_pose.position.x = last_x_;
// //     robot_pose.position.y = last_y_;

// //     map_memory_.integrateCostmap(latest_costmap_, latest_pose_);

// //     map_pub_->publish(map_memory_.getGlobalMap());
// //     should_update_map_ = false;
// //   }
// // }

// // int main(int argc, char ** argv)
// // {
// //   rclcpp::init(argc, argv);
// //   rclcpp::spin(std::make_shared<MapMemoryNode>());
// //   rclcpp::shutdown();
// //   return 0;
// // }



// // // #include "map_memory_node.hpp"

// // // MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {}

// // // int main(int argc, char ** argv)
// // // {
// // //   rclcpp::init(argc, argv);
// // //   rclcpp::spin(std::make_shared<MapMemoryNode>());
// // //   rclcpp::shutdown();
// // //   return 0;
// // // }
