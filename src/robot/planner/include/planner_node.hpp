#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();

private:
  robot::PlannerCore planner_;

  enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
  State state_;

  // Subscribers and Publisher
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // The occupancy grid map
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;

  // The goal point
  geometry_msgs::msg::PointStamped::SharedPtr goal_;

  // The robot's current pose
  geometry_msgs::msg::Pose::SharedPtr robot_pose_;

  bool goal_received_ = false;
  double goal_tolerance_; // Tolerance to consider goal reached

  // Callback functions
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();
  void planPath();
  bool goalReached();
};

#endif
