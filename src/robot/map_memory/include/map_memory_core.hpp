#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <cstdint>
#include <vector>

namespace robot {

struct RobotTransform {
  double x = 0.0;
  double y = 0.0;
  double rot_x = 0.0;
  double rot_y = 0.0;
  double rot_z = 0.0;
  double rot_w = 1.0;
};

class MapMemoryCore {
public:
  explicit MapMemoryCore(const rclcpp::Logger &logger);

  // Initialize the global map
  void initializeGlobalMap(const std::string &frame_id, double resolution,
                           int width, int height, double origin_x,
                           double origin_y, int8_t cell_threshold,
                           int8_t unknown_cell_value);

  // Integrate a costmap into the global map
  void integrateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap,
                        const RobotTransform &robot_world_transform);

  const nav_msgs::msg::OccupancyGrid::SharedPtr getGlobalMap() const;

private:
  rclcpp::Logger logger_;
  nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
  int8_t cell_threshold_;     // Threshold for marking cells as occupied
  int8_t unknown_cell_value_; // Value for unknown cells
};

} // namespace robot

#endif
