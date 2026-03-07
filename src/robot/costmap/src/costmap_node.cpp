#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // LaserScan message contains:
  // - angle_min, angle_max: start and end angles of the scan
  // - angle_increment: angular resolution
  // - ranges: array of distance measurements

  // Define costmap parameters. 
  double resolution = 0.1; // 0.1 meters per cell, with 200x200 cells, gives a 20m x 20m area
  int width = 200;
  int height = 200;
  double inflation_radius = 1.0; // 1 meter inflation radius around obstacles
  int8_t default_cell_value = 0; // Default cost value for free space
  int8_t max_cost_inflated_cell = 100; // Maximum cost value for a inflated cell
  int8_t cost_occupied = 100; // Cost value for occupied cells

  // Step 1: Initialize costmap
  costmap_.initializeCostmap(resolution, width, height, default_cell_value);

  // Step 2: Convert LaserScan to grid and mark obstacles
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
      double angle = scan->angle_min + i * scan->angle_increment;
      double range = scan->ranges[i];

      // Check if the range is valid
      if (range < scan->range_max && range > scan->range_min) {
          // Calculate grid coordinates
          int x_grid, y_grid;
          costmap_.convertToGrid(range, angle, x_grid, y_grid);
          costmap_.markObstacle(x_grid, y_grid, cost_occupied);
      }
  }

  // Step 3: Inflate obstacles
  costmap_.inflateObstacles(inflation_radius, max_cost_inflated_cell, cost_occupied);

  // Step 4: Publish costmap (no need for a separate timer callback since we can 
  // publish immediately after processing the scan)
  publishCostmap();
}

void CostmapNode::publishCostmap() {
  // Convert the 2D costmap array into a nav_msgs::msg::OccupancyGrid message
  // Populate the OccupancyGrid message fields (header, info) based on the costmap params
  auto grid_msg = nav_msgs::msg::OccupancyGrid();
  grid_msg.header.frame_id = "robot/chassis/lidar"; // Use the same frame as the LaserScan
  grid_msg.header.stamp = this->get_clock()->now();
  grid_msg.info.resolution = static_cast<float>(costmap_.getResolution());
  grid_msg.info.width = static_cast<uint32_t>(costmap_.getWidth());
  grid_msg.info.height = static_cast<uint32_t>(costmap_.getHeight());

  // OccupancyGrid.info.origin is in the bottom-left of the grid. So to align the map center with the world (0,0),
  // the map must be shifted down and left (i.e. negative x and y) by half the map size
  grid_msg.info.origin.position.x = -(costmap_.getWidth() / 2) * costmap_.getResolution();
  grid_msg.info.origin.position.y = -(costmap_.getHeight() / 2) * costmap_.getResolution();
  grid_msg.info.origin.orientation.w = 1.0;

  // Get the costmap data as a flattened 1D array and assign it to the OccupancyGrid message
  grid_msg.data = costmap_.getCostmapData();

  // Publish message to "/costmap" topic
  costmap_pub_->publish(grid_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}