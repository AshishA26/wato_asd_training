#include "planner_core.hpp"

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger &logger) : logger_(logger) {
    path_ = std::make_shared<nav_msgs::msg::Path>();
}

void PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
    const geometry_msgs::msg::Pose::SharedPtr &robot_pose,
    const geometry_msgs::msg::PointStamped::SharedPtr &goal) {
        
    }

nav_msgs::msg::Path::SharedPtr PlannerCore::getPath() const { return path_; }

} // namespace robot
