#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

private:
  robot::CostmapCore costmap_; // CostmapCore instance
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  // Callback function to process laser scan messages and update the costmap
  void laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan);

  // Function to publish the costmap as an OccupancyGrid message
  void publishCostmap();

  // Define costmap parameters.
  double resolution_;
  int width_;
  int height_;
  double inflation_radius_;
  int8_t default_cell_value_; // Default cost value for free space
  int8_t max_cost_inflated_cell_;
  int8_t cost_occupied_;
  std::string frame_id_;
};

#endif