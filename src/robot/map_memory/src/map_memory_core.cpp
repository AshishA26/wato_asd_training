#include "map_memory_core.hpp"

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger &logger) : logger_(logger), default_cell_value_(-1) {
    global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
}

void MapMemoryCore::initializeGlobalMap(const std::string &frame_id,
                                        double resolution, int width,
                                        int height, double origin_x,
                                        double origin_y, int8_t default_cell_value) {
  global_map_->header.frame_id = frame_id;
  global_map_->info.resolution = resolution;
  global_map_->info.width = width;                // [cells]
  global_map_->info.height = height;              // [cells]
  global_map_->info.origin.position.x = origin_x; // [meters]
  global_map_->info.origin.position.y = origin_y; // [meters]
  global_map_->info.origin.orientation.w = 1.0;   // No rotation

  // Initialize the global map data with unknown values
  global_map_->data.resize(width * height, default_cell_value);
  default_cell_value_ = default_cell_value;
}

void MapMemoryCore::integrateCostmap(
    const nav_msgs::msg::OccupancyGrid::SharedPtr costmap,
    const RobotTransform &robot_transform) {

  for (size_t i = 0; i < costmap->data.size(); ++i) {
    // Overwrite global map with costmap values. Only update cells that are
    // known.
    if (costmap->data[i] != default_cell_value_) {
      global_map_->data[i] = costmap->data[i];
    }
  }
}

const nav_msgs::msg::OccupancyGrid::SharedPtr
MapMemoryCore::getGlobalMap() const {
  return global_map_;
}

} // namespace robot
