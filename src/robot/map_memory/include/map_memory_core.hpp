#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

// #include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "tf2_ros/buffer.h"
// #include "tf2_ros/transform_listener.h"
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
                           double origin_y, int8_t default_cell_value);

  // Integrate a costmap into the global map
  void integrateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap,
                        const RobotTransform &robot_transform);

  const nav_msgs::msg::OccupancyGrid::SharedPtr getGlobalMap() const;

private:
  rclcpp::Logger logger_;
  nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
  int8_t default_cell_value_; // Default value for unknown cells

  // TF2 components
  // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace robot

#endif
