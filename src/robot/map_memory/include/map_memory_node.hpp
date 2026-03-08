#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"

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
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  // Callback for odometry updates
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Timer-based map update
  void updateMap();

  std::string frame_id_ = "sim_world";
  double resolution_ = 0.1;
  int width_ = 200;
  int height_ = 200;

  // Robot state tracking
  double last_x_ = 0.0;
  double last_y_ = 0.0;
  robot::RobotTransform robot_transform_;
  const double distance_threshold_ = 1.5;

  // Flags to track if we have received data
  bool costmap_updated_ = false;
  bool odom_initialized_ = false;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  bool should_update_map_ = false;
};

#endif
