#include "control_core.hpp"
#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger, double lookahead_distance, double linear_speed, double goal_tolerance)
    : logger_(logger),
      lookahead_distance_(lookahead_distance),
      linear_speed_(linear_speed),
      goal_tolerance_(goal_tolerance) {}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
        const nav_msgs::msg::Path & path,
        const geometry_msgs::msg::Point & current_point,
        const geometry_msgs::msg::Quaternion & current_orientation) {

    double robot_yaw = extractYaw(current_orientation);

    for (const auto &pose : path.poses) {
        double dx = pose.pose.position.x - current_point.x;
        double dy = pose.pose.position.y - current_point.y;
        double distance = computeDistance(current_point, pose.pose.position);

        if (distance < lookahead_distance_) continue;

        double angle_to_point = std::atan2(dy, dx);
        double angle_diff = angle_to_point - robot_yaw;

        // Normalize angle_diff to [-π, π]
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        if (std::abs(angle_diff) < M_PI / 2.0) {
            return pose;
        }
    }

    RCLCPP_WARN(logger_, "No valid lookahead point found in front of the robot");
    return std::nullopt;
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const geometry_msgs::msg::Pose &robot_pose, const geometry_msgs::msg::PoseStamped &target) {

    double robot_yaw = extractYaw(robot_pose.orientation);

    double angle_to_target = std::atan2(
        target.pose.position.y - robot_pose.position.y,
        target.pose.position.x - robot_pose.position.x);

    double angle_diff = angle_to_target - robot_yaw;

    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = angle_diff;

    return cmd_vel;
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    // Calculate Euclidean distance between two points
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    // Extract yaw angle from quaternion
    return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y), quat.w * quat.w - quat.x * quat.x - quat.y * quat.y + quat.z * quat.z);
}

}  



// #include "control_core.hpp"

// namespace robot
// {

// ControlCore::ControlCore(const rclcpp::Logger& logger) 
//   : logger_(logger) {}

// }  
