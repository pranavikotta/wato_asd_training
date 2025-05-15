#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>

namespace robot
{

class ControlCore {
  public:

    ControlCore(const rclcpp::Logger& logger, double lookahead_distance, double linear_speed, double goal_tolerance);

    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
        const nav_msgs::msg::Path & path,
        const geometry_msgs::msg::Point & current_point,
        const geometry_msgs::msg::Quaternion & current_orientation);

    geometry_msgs::msg::Twist computeVelocity(
        const geometry_msgs::msg::Pose &robot_pose,
        const geometry_msgs::msg::PoseStamped &target);

    double goalTolerance() const { return goal_tolerance_; }
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);
  
  private:
    rclcpp::Logger logger_;
    
    double lookahead_distance_;
    double linear_speed_;
    double goal_tolerance_;
};

} 

#endif 



// #ifndef CONTROL_CORE_HPP_
// #define CONTROL_CORE_HPP_

// #include "rclcpp/rclcpp.hpp"

// namespace robot
// {

// class ControlCore {
//   public:
//     // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
//     ControlCore(const rclcpp::Logger& logger);
  
//   private:
//     rclcpp::Logger logger_;
// };

// } 

// #endif 
