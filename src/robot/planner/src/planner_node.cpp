#include "planner_node.hpp"

PlannerNode::PlannerNode()
  : Node("planner"), logger_(this->get_logger()) {

  occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occupancy_grid", 10);
  costmap_ = std::make_shared<robot::CostmapCore>(logger_, occupancy_grid_pub_);
  planner_ = std::make_unique<robot::PlannerCore>(logger_, costmap_);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  RCLCPP_INFO(logger_, "Received map: %d x %d", msg->info.width, msg->info.height);
  costmap_->updateMap(*msg);
  planner_->updateMap(*msg);
  
  if (planner_->hasGoal()) {
    auto path = planner_->planPath();
    path_pub_->publish(path);
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  planner_->setGoal(*msg);
  auto path = planner_->planPath();
  path_pub_->publish(path);
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  planner_->updateOdom(*msg);
}

void PlannerNode::timerCallback() {
  if (planner_->getState() == robot::PlannerState::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (planner_->isGoalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      planner_->clearGoal();
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning path...");
      auto path = planner_->planPath();
      path_pub_->publish(path);
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}



// #include "planner_node.hpp"

// PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {}

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<PlannerNode>());
//   rclcpp::shutdown();
//   return 0;
// }
