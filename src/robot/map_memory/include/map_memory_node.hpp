#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

private:
  robot::MapMemoryCore map_memory_; // MapMemoryCore instance
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback for costmap updates
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  // Callback for odometry updates
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // Timer-based map update
  void updateMap();

  // Robot state tracking
  double last_x_ = 0.0;
  double last_y_ = 0.0;
  robot::RobotTransform robot_world_transform_;
  double distance_threshold_;

  // Flags to track if we have received data
  bool costmap_updated_ = false;
  bool odom_initialized_ = false;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_costmap_;
  bool should_update_map_ = false;
};

#endif
