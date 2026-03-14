#include "control_core.hpp"

namespace robot {

ControlCore::ControlCore(const rclcpp::Logger &logger) : logger_(logger) {}

geometry_msgs::msg::PoseStamped
ControlCore::findLookaheadPoint(const nav_msgs::msg::Path::SharedPtr &path,
                                const nav_msgs::msg::Odometry::SharedPtr &odom,
                                double lookahead_distance) {
  if (!path || path->poses.empty() || !odom) {
    return geometry_msgs::msg::PoseStamped();
  }

  // Get the robot's current position
  auto robot_pos = odom->pose.pose.position;

  // Iterate through the points in the path to find the lookahead point
  for (const auto &pose : path->poses) {
    double dist = computeDistance(robot_pos, pose.pose.position);
    if (dist >= lookahead_distance) {
      return pose;
    }
  }

  // Return the last point if no lookahead point is found
  return path->poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const geometry_msgs::msg::PoseStamped &target,
    const nav_msgs::msg::Odometry::SharedPtr &odom, double linear_speed) {
  geometry_msgs::msg::Twist cmd_vel;

  // Get robot's current position and orientation
  double robot_x = odom->pose.pose.position.x;
  double robot_y = odom->pose.pose.position.y;
  double robot_yaw = extractYaw(odom->pose.pose.orientation);

  // Get target position (world frame)
  double target_x = target.pose.position.x;
  double target_y = target.pose.position.y;

  // Apply rotation using rotation matrix. Converts world
  // coordinates of target point to robot-centric coordinates
  double dx = target_x - robot_x;
  double dy = target_y - robot_y;
  double cos_yaw = cos(-robot_yaw); // Negative for inverse rotation
  double sin_yaw = sin(-robot_yaw);
  double local_x = dx * cos_yaw - dy * sin_yaw;
  double local_y = dx * sin_yaw + dy * cos_yaw;

  // Pure pursuit formula: k = 2y / L^2, and angular velocity is w = v * k
  // Where:
  // - y is the y offset to the target in the robot's frame
  // - L is the distance to the target in the robot's frame
  // - v is the desired linear speed
  // - k is the curvature to the target
  // - w is the angular velocity to steer towards the target

  // Get L^2 (distance to target squared)
  double L2 = local_x * local_x + local_y * local_y;
  if (L2 == 0) {
    return cmd_vel; // Avoid division by zero
  }

  // Compute curvature and angular velocity
  double curvature = (2.0 * local_y) / L2;
  cmd_vel.linear.x = linear_speed;
  cmd_vel.angular.z = curvature * linear_speed;

  return cmd_vel;
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point &a,
                                    const geometry_msgs::msg::Point &b) {
  // Distance calculation between two points
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  // Quaternion to yaw conversion
  return atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
               1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
}
} // namespace robot
