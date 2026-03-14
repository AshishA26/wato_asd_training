#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>

namespace robot {

class ControlCore {
public:
  ControlCore(const rclcpp::Logger &logger);
  std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();

  geometry_msgs::msg::Twist
  computeVelocity(const geometry_msgs::msg::PoseStamped &target);

  double computeDistance(const geometry_msgs::msg::Point &a,
                         const geometry_msgs::msg::Point &b);

  double extractYaw(const geometry_msgs::msg::Quaternion &quat);

private:
  rclcpp::Logger logger_;
};

} // namespace robot

#endif
