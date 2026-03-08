#include <chrono>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory"),
      map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // Initialize subscribers
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10,
      std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Initialize publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize timer for periodic map updates
  timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                   std::bind(&MapMemoryNode::updateMap, this));

  // Initialize global map
  map_memory_.initializeGlobalMap(frame_id_, resolution_, width_, height_,
                                  -(width_ * resolution_) / 2,
                                  -(height_ * resolution_) / 2);
}

void MapMemoryNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Store the latest costmap
  latest_costmap_ = msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // Compute distance traveled
  double distance =
      std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));

  // Update map if the robot has moved beyond the threshold
  if (distance >= distance_threshold_) {
    last_x_ = x;
    last_y_ = y;
    robot_transform_ = {.x = last_x_,
                        .y = last_y_,
                        .rot_x = msg->pose.pose.orientation.x,
                        .rot_y = msg->pose.pose.orientation.y,
                        .rot_z = msg->pose.pose.orientation.z,
                        .rot_w = msg->pose.pose.orientation.w};
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_) {

    map_memory_.integrateCostmap(latest_costmap_, robot_transform_);

    nav_msgs::msg::OccupancyGrid::SharedPtr global_map =
        map_memory_.getGlobalMap();
    global_map->header.stamp = this->get_clock()->now();
    global_map->header.frame_id = frame_id_;

    map_pub_->publish(*global_map);
    should_update_map_ = false;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
