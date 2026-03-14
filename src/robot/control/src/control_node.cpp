#include "control_node.hpp"

ControlNode::ControlNode()
    : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  // Initialize parameters
  lookahead_distance_ = this->declare_parameter("lookahead_distance", 1.0);
  goal_tolerance_ = this->declare_parameter("goal_tolerance", 0.1);
  linear_speed_ = this->declare_parameter("linear_speed", 0.5);
  std::string path_topic = this->declare_parameter("path_topic", "/path");
  std::string odom_topic =
      this->declare_parameter("odom_topic", "/odom/filtered");
  std::string cmd_vel_topic =
      this->declare_parameter("cmd_vel_topic", "/cmd_vel");

  // Subscribers and Publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic, 10, [this](const nav_msgs::msg::Path::ConstSharedPtr msg) {
        current_path_ = msg;
      });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        robot_odom_ = msg;
      });
  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

  // Timer
  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                           [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  // Skip control if no path or odometry data is available
  if (!current_path_ || !robot_odom_) {
    return;
  }

  // If an empty path is received, stop the robot to avoid stale commands
  if (current_path_->poses.empty()) {
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  // Check if we are close enough to the goal
  auto goal = current_path_->poses.back().pose.position;
  auto robot_pos = robot_odom_->pose.pose.position;
  double distance_to_goal = control_.computeDistance(robot_pos, goal);
  if (distance_to_goal < goal_tolerance_) {
    // We are at the goal, stop the robot
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  // Find the lookahead point
  auto lookahead_point = control_.findLookaheadPoint(current_path_, robot_odom_,
                                                     lookahead_distance_);

  // Compute velocity command. Target is the lookahead point.
  auto cmd_vel =
      control_.computeVelocity(lookahead_point, robot_odom_, linear_speed_);

  // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
