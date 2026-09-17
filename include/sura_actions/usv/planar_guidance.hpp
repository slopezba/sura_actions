#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace sura_actions::usv
{
using Pose2D = geometry_msgs::msg::Pose2D;

inline double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline bool finitePose(const Pose2D & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.theta);
}

inline geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

inline bool quaternionYaw(const geometry_msgs::msg::Quaternion & q, double & yaw)
{
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (!std::isfinite(norm) || norm < 1.e-6) {
    return false;
  }
  const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
  yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return true;
}

struct GuidanceLimits
{
  double forward_speed{0.20};
  double yaw_rate{0.25};
  double position_tolerance{0.30};
  double yaw_tolerance{0.12};
  double slowdown_distance{1.2};
  double heading_stop{0.70};
  double yaw_gain{0.8};
};

struct GuidanceOutput
{
  geometry_msgs::msg::Twist velocity{};
  double distance{0.0};
  double yaw_error{0.0};
  bool aligning{false};
  bool reached{false};
};

// A pure planar controller: distance and heading govern forward motion;
// final orientation is handled at rest once XY is within tolerance.
inline GuidanceOutput guide(
  const Pose2D & current, const Pose2D & target, const GuidanceLimits & limits,
  bool enforce_final_yaw)
{
  GuidanceOutput out;
  const double dx = target.x - current.x;
  const double dy = target.y - current.y;
  out.distance = std::hypot(dx, dy);
  out.aligning = out.distance <= limits.position_tolerance;
  const double desired_yaw = out.aligning ? target.theta : std::atan2(dy, dx);
  out.yaw_error = normalizeAngle(desired_yaw - current.theta);
  out.reached = out.aligning &&
    (!enforce_final_yaw || std::abs(out.yaw_error) <= limits.yaw_tolerance);
  if (out.reached) {
    return out;
  }
  out.velocity.angular.z = std::clamp(
    limits.yaw_gain * out.yaw_error, -limits.yaw_rate, limits.yaw_rate);
  if (!out.aligning) {
    const double alignment = std::clamp(
      1.0 - std::abs(out.yaw_error) / limits.heading_stop, 0.0, 1.0);
    out.velocity.linear.x = limits.forward_speed * alignment *
      std::min(1.0, out.distance / limits.slowdown_distance);
  }
  return out;
}

inline double goalValue(double value, double fallback)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument("Goal limits must be finite and non-negative (0 = default)");
  }
  return value == 0.0 ? fallback : value;
}
}  // namespace sura_actions::usv
