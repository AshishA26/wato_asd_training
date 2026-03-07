#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Callback function to process laser scan messages and update the costmap
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Function to publish the costmap as an OccupancyGrid message
    void publishCostmap();

  private:
    robot::CostmapCore costmap_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_; // Subscription to LIDAR data
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_; // Publisher for the costmap
};

#endif 