#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "control_core.hpp"

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Publisher for velocity commands
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Timer for control loop
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Control loop function
    void controlLoop();

    // Data for current path and odometry
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;
};

#endif



// #ifndef CONTROL_NODE_HPP_
// #define CONTROL_NODE_HPP_

// #include "rclcpp/rclcpp.hpp"

// #include "control_core.hpp"

// class ControlNode : public rclcpp::Node {
//   public:
//     ControlNode();

//   private:
//     robot::ControlCore control_;
// };

// #endif
