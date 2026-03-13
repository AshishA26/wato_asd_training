#include <chrono>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory"),
      map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // Declare parameters with default values
  double resolution = this->declare_parameter("resolution", 0.1);
  int width = this->declare_parameter("width", 1000);
  int height = this->declare_parameter("height", 1000);
  distance_threshold_ = this->declare_parameter("distance_threshold", 1.5);
  int8_t cell_threshold = this->declare_parameter("cell_threshold", 50);
  int8_t unknown_cell_value = this->declare_parameter("unknown_cell_value", -1);
  std::string frame_id = this->declare_parameter("frame_id", "sim_world");
  std::string costmap_topic =
      this->declare_parameter("costmap_topic", "/costmap");
  std::string odom_topic =
      this->declare_parameter("odom_topic", "/odom/filtered");
  std::string map_topic = this->declare_parameter("map_topic", "/map");

  // Initialize subscribers
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic, 10,
      std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Initialize publisher
  map_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, 10);
  // Initialize timer for periodic map updates
  timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                   std::bind(&MapMemoryNode::updateMap, this));

  // Initialize global map
  map_memory_.initializeGlobalMap(
      frame_id, resolution, width, height, -(width * resolution) / 2,
      -(height * resolution) / 2, cell_threshold, unknown_cell_value);
}

void MapMemoryNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
  // Store the latest costmap
  latest_costmap_ = msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // Always update current transform
  robot_world_transform_.x = x;
  robot_world_transform_.y = y;
  robot_world_transform_.rot_x = msg->pose.pose.orientation.x;
  robot_world_transform_.rot_y = msg->pose.pose.orientation.y;
  robot_world_transform_.rot_z = msg->pose.pose.orientation.z;
  robot_world_transform_.rot_w = msg->pose.pose.orientation.w;

  // Initialize on first odom message (needed to trigger first map update)
  if (!odom_initialized_) {
    last_x_ = x;
    last_y_ = y;
    odom_initialized_ = true;
    should_update_map_ = true; // Trigger first update
    return;
  }

  // Compute distance traveled
  double distance =
      std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));

  // Update map if the robot has moved beyond the threshold
  if (distance >= distance_threshold_) {
    last_x_ = x;
    last_y_ = y;
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_) {

    // Integrate the latest costmap into the global map
    map_memory_.integrateCostmap(latest_costmap_, robot_world_transform_);

    // Get the updated global map
    nav_msgs::msg::OccupancyGrid::SharedPtr global_map =
        map_memory_.getGlobalMap();

    // Copy and update the header for publishing
    nav_msgs::msg::OccupancyGrid map_to_publish = *global_map;
    map_to_publish.header.stamp = this->get_clock()->now();

    // Publish the updated global map
    map_pub_->publish(map_to_publish);
    should_update_map_ = false;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
