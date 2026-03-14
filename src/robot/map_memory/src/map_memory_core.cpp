#include "map_memory_core.hpp"

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger &logger)
    : logger_(logger), cell_threshold_(50), unknown_cell_value_(-1) {
  global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
}

void MapMemoryCore::initializeGlobalMap(const std::string &frame_id,
                                        double resolution, int width,
                                        int height, double origin_x,
                                        double origin_y, int8_t cell_threshold,
                                        int8_t unknown_cell_value) {
  global_map_->header.frame_id = frame_id;
  global_map_->info.resolution = resolution;
  global_map_->info.width = width;                // [cells]
  global_map_->info.height = height;              // [cells]
  global_map_->info.origin.position.x = origin_x; // [meters]
  global_map_->info.origin.position.y = origin_y; // [meters]
  global_map_->info.origin.orientation.w = 1.0;   // No rotation

  // Initialize the global map data with unknown values
  global_map_->data.resize(width * height, unknown_cell_value);
  cell_threshold_ = cell_threshold;
  unknown_cell_value_ = unknown_cell_value;
}

void MapMemoryCore::integrateCostmap(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap,
    const RobotTransform &robot_world_transform) {

  // Get yaw angle from quaternion
  double yaw = atan2(
      2.0 * (robot_world_transform.rot_w * robot_world_transform.rot_z +
             robot_world_transform.rot_x * robot_world_transform.rot_y),
      1.0 - 2.0 * (robot_world_transform.rot_y * robot_world_transform.rot_y +
                   robot_world_transform.rot_z * robot_world_transform.rot_z));
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);

  // Get costmap parameters
  int costmap_height = static_cast<int>(costmap->info.height);
  int costmap_width = static_cast<int>(costmap->info.width);
  double costmap_resolution = static_cast<double>(costmap->info.resolution);
  double costmap_origin_x = costmap->info.origin.position.x;
  double costmap_origin_y = costmap->info.origin.position.y;

  // Get global map parameters
  int global_map_height = static_cast<int>(global_map_->info.height);
  int global_map_width = static_cast<int>(global_map_->info.width);
  double global_map_resolution =
      static_cast<double>(global_map_->info.resolution);
  double global_map_origin_x = global_map_->info.origin.position.x;
  double global_map_origin_y = global_map_->info.origin.position.y;

  // Iterate through each cell in the incoming costmap
  for (int cy = 0; cy < costmap_height; ++cy) {
    for (int cx = 0; cx < costmap_width; ++cx) {
      // Get costmap cell index and value
      int costmap_index = cy * costmap_width + cx;
      int8_t cell_value = costmap->data[costmap_index];

      // Skip unknown cells - keep global map value
      if (cell_value == unknown_cell_value_) {
        continue;
      }

      // Convert costmap cell to position in costmap frame (meters)
      double costmap_x = costmap_origin_x + (cx * costmap_resolution);
      double costmap_y = costmap_origin_y + (cy * costmap_resolution);

      // Apply rotation using rotation matrix, then translation. Converts robot
      // relative costmap coordinates to global map coordinates.
      double world_x =
          robot_world_transform.x + costmap_x * cos_yaw - costmap_y * sin_yaw;
      double world_y =
          robot_world_transform.y + costmap_x * sin_yaw + costmap_y * cos_yaw;

      // Convert world position to global map cell coordinates
      int gx = static_cast<int>(
          std::floor((world_x - global_map_origin_x) / global_map_resolution));
      int gy = static_cast<int>(
          std::floor((world_y - global_map_origin_y) / global_map_resolution));

      // Check bounds and update global map
      if (gx >= 0 && gx < global_map_width && gy >= 0 &&
          gy < global_map_height) {
        int global_index = gy * global_map_width + gx;

        if (cell_value >= cell_threshold_) {
          // There is a high probability of an obstacle, so we mark it as
          // occupied in the global map and keep the highest cost.
          global_map_->data[global_index] =
              std::max(global_map_->data[global_index], cell_value);
        } else if (global_map_->data[global_index] < cell_threshold_) {
          // Cell is either free or inflated. Needed to clear ghosting and
          // prevent false positives.
          global_map_->data[global_index] = cell_value;
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid::ConstSharedPtr
MapMemoryCore::getGlobalMap() const {
  return global_map_;
}

} // namespace robot
