#include "control_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_((this->get_logger()), 1.0, 0.5, 0.1) {
    // Initialize subscribers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
            current_path_ = msg;
        });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            robot_odom_ = msg;
        });

    // Publisher for velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer for periodic control loop
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]() { controlLoop(); });
}
void ControlNode::controlLoop() {
    if (!current_path_ || !robot_odom_) {
        return;
    }

    const auto &robot_position = robot_odom_->pose.pose.position;
    const auto &goal_pose = current_path_->poses.back().pose.position;

    double distance_to_goal = control_.computeDistance(robot_position, goal_pose);
    if (distance_to_goal <=  control_.goalTolerance()) {
        RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping robot.");
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);  // zero velocities
        return;
    }

    auto lookahead_point = control_.findLookaheadPoint(*current_path_, robot_position, robot_odom_->pose.pose.orientation);

    if (!lookahead_point) {
        RCLCPP_WARN(this->get_logger(), "No valid lookahead point found");
        return;
    }

    auto cmd_vel = control_.computeVelocity(robot_odom_->pose.pose, *lookahead_point);
    cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}


// #include "control_node.hpp"

// ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {}

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<ControlNode>());
//   rclcpp::shutdown();
//   return 0;
// }
