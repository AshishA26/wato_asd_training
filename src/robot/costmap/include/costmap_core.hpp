#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace robot {

class CostmapCore {
public:
  explicit CostmapCore(const rclcpp::Logger &logger);

  // Initialize costmap parameters and reset grid to default values
  void initializeCostmap(double resolution, int width, int height,
                         int8_t default_cell_value);

  // Convert polar coordinates to grid coordinates
  void convertToGrid(double range, double angle, int &x_grid, int &y_grid);

  // Mark a cell as occupied based on grid coordinates
  void markObstacle(int x, int y, int8_t cost_occupied);

  // Inflate obstacles in the costmap based on the inflation radius
  void inflateObstacles(double inflation_radius, int8_t max_cost_inflated_cell,
                        int8_t cost_occupied);

  // Get the costmap data as a flattened 1D array
  std::vector<int8_t> getCostmapData() const;

  double getResolution() const; // Get grid resolution
  int getWidth() const;         // Get grid width
  int getHeight() const;        // Get grid height

private:
  rclcpp::Logger logger_;

  // 2D array representing occupancy grid
  std::vector<std::vector<int8_t>> occupancy_grid_;

  double resolution_; // Resolution in meters per cell
  int width_;         // Width of the occupancy grid in cells
  int height_;        // Height of the occupancy grid in cells
};

} // namespace robot

#endif