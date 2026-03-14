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

private:
  rclcpp::Logger logger_;
};

} // namespace robot

#endif
