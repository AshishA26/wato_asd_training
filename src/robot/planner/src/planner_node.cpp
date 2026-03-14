#include "planner_node.hpp"

PlannerNode::PlannerNode()
    : Node("planner"), planner_(robot::PlannerCore(this->get_logger())),
      state_(State::WAITING_FOR_GOAL) {
  // Declare parameters
  std::string map_topic = this->declare_parameter("map_topic", "/map");
  std::string goal_topic = this->declare_parameter("goal_topic", "/goal_point");
  std::string odom_topic =
      this->declare_parameter("odom_topic", "/odom/filtered");
  std::string path_topic = this->declare_parameter("path_topic", "/path");
  goal_tolerance_ = this->declare_parameter("goal_tolerance", 0.5);
  cell_threshold_ = this->declare_parameter("cell_threshold", 50);

  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, 10,
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      goal_topic, 10,
      std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
  // Timer
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(500),
                              std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = std::make_shared<geometry_msgs::msg::Pose>(msg->pose.pose);
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Replanning due to timeout or progress...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached() {
  // Cannot determine if goal is reached without goal or robot pose
  if (!goal_received_ || !robot_pose_) {
    return false;
  }

  // Calculate the distance between robots position and the goal
  double dx = goal_->point.x - robot_pose_->position.x;
  double dy = goal_->point.y - robot_pose_->position.y;
  return std::sqrt(dx * dx + dy * dy) <
         goal_tolerance_; // Threshold for reaching the goal
}

void PlannerNode::planPath() {
  if (!goal_received_ || !current_map_ || current_map_->data.empty() ||
      !robot_pose_) {
    RCLCPP_WARN(this->get_logger(),
                "Cannot plan path: Missing map, goal, or robot pose!");
    return;
  }

  // Initialize start and goal points
  const robot::CellIndex start_point =
      planner_.convertWorldToGrid(robot_pose_->position, current_map_);
  const robot::CellIndex goal_point =
      planner_.convertWorldToGrid(goal_->point, current_map_);

  // Check if goal is valid
  if (!planner_.isTraversable(goal_point, current_map_, cell_threshold_) ||
      !planner_.isTraversable(start_point, current_map_)) {
    RCLCPP_WARN(this->get_logger(),
                "Goal or start position is not traversable!");
    return;
  }

  // Call the planner core to compute the path
  nav_msgs::msg::Path::SharedPtr path =
      planner_.planPath(current_map_, start_point, goal_point, cell_threshold_);

  if (!path || path->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Planner failed.");
    return;
  }

  // Copy and set header fields
  nav_msgs::msg::Path path_to_publish = *path;
  path_to_publish.header.stamp = this->get_clock()->now();
  path_to_publish.header.frame_id = current_map_->header.frame_id;

  // Publish the path
  path_pub_->publish(path_to_publish);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
