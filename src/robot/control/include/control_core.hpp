#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

namespace robot {

class ControlCore {
public:
  ControlCore(const rclcpp::Logger &logger);

  // Logic to find the lookahead point on the path
  geometry_msgs::msg::PoseStamped
  findLookaheadPoint(const nav_msgs::msg::Path::ConstSharedPtr &path,
                     const nav_msgs::msg::Odometry::ConstSharedPtr &odom,
                     double lookahead_distance);

  // Logic to compute velocity commands
  geometry_msgs::msg::Twist
  computeVelocity(const geometry_msgs::msg::PoseStamped &target,
                  const nav_msgs::msg::Odometry::ConstSharedPtr &odom,
                  double linear_speed);

  double computeDistance(const geometry_msgs::msg::Point &a,
                         const geometry_msgs::msg::Point &b);

  double extractYaw(const geometry_msgs::msg::Quaternion &quat);

private:
  rclcpp::Logger logger_;
};

} // namespace robot

#endif
