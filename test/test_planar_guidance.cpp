#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <unistd.h>

#include "gtest/gtest.h"
#include "sura_actions/usv/path_io.hpp"

using namespace sura_actions::usv;  // NOLINT

static Pose2D pose(double x, double y, double yaw = 0.0)
{
  Pose2D p;
  p.x = x; p.y = y; p.theta = yaw;
  return p;
}

TEST(PlanarGuidance, WrapAndForwardOnly)
{
  EXPECT_NEAR(normalizeAngle(3.0 * M_PI), M_PI, 1.e-12);
  EXPECT_NEAR(normalizeAngle(-2.0 * M_PI + 0.1), 0.1, 1.e-12);
  const GuidanceLimits limits;
  for (int i = -100; i <= 100; ++i) {
    const auto out = guide(pose(0, 0, i * 0.1), pose(2, 1, -1), limits, true);
    EXPECT_GE(out.velocity.linear.x, 0.0);
    EXPECT_LE(out.velocity.linear.x, limits.forward_speed);
    EXPECT_LE(std::abs(out.velocity.angular.z), limits.yaw_rate);
    EXPECT_DOUBLE_EQ(out.velocity.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(out.velocity.linear.z, 0.0);
    EXPECT_DOUBLE_EQ(out.velocity.angular.x, 0.0);
    EXPECT_DOUBLE_EQ(out.velocity.angular.y, 0.0);
  }
}

TEST(PlanarGuidance, SlowdownAndHeadingStop)
{
  const GuidanceLimits limits;
  const auto far = guide(pose(0, 0), pose(3, 0), limits, true);
  const auto near = guide(pose(0, 0), pose(0.6, 0), limits, true);
  EXPECT_DOUBLE_EQ(far.velocity.linear.x, 0.20);
  EXPECT_NEAR(near.velocity.linear.x, 0.10, 1.e-12);
  const auto behind = guide(pose(0, 0), pose(-2, 0), limits, true);
  EXPECT_DOUBLE_EQ(behind.velocity.linear.x, 0.0);
  EXPECT_GT(std::abs(behind.velocity.angular.z), 0.0);
}

TEST(PlanarGuidance, FinalYawAndIntermediateYaw)
{
  const GuidanceLimits limits;
  auto out = guide(pose(0, 0), pose(0, 0, 1.0), limits, true);
  EXPECT_TRUE(out.aligning);
  EXPECT_FALSE(out.reached);
  EXPECT_DOUBLE_EQ(out.velocity.linear.x, 0.0);
  EXPECT_GT(out.velocity.angular.z, 0.0);
  EXPECT_TRUE(guide(pose(0, 0), pose(0, 0, 1.0), limits, false).reached);
  EXPECT_TRUE(guide(pose(0, 0), pose(0, 0), limits, true).reached);
  // A drift outside tolerance must resume approach instead of accepting yaw alone.
  EXPECT_FALSE(guide(pose(1, 0), pose(0, 0), limits, true).aligning);
}

TEST(PlanarGuidance, InvalidInputs)
{
  EXPECT_THROW(goalValue(std::numeric_limits<double>::infinity(), 1.0), std::invalid_argument);
  EXPECT_THROW(goalValue(std::numeric_limits<double>::quiet_NaN(), 1.0), std::invalid_argument);
  EXPECT_THROW(goalValue(-1, 1.0), std::invalid_argument);
  EXPECT_DOUBLE_EQ(goalValue(0, 0.2), 0.2);
  EXPECT_FALSE(finitePose(pose(0, std::numeric_limits<double>::quiet_NaN())));
  double yaw;
  geometry_msgs::msg::Quaternion q;
  q.w = 0.0;
  EXPECT_FALSE(quaternionYaw(q, yaw));
  q = yawQuaternion(1.2);
  q.z *= 2; q.w *= 2;
  ASSERT_TRUE(quaternionYaw(q, yaw));
  EXPECT_NEAR(yaw, 1.2, 1.e-12);
}

TEST(PlanarGuidance, ConvergesWithUnicycleKinematics)
{
  const GuidanceLimits limits;
  for (const auto & target : {pose(2, 1, 1.5), pose(-2, 0, -1.0), pose(0, 2, -2.0)}) {
    auto current = pose(0, 0);
    bool reached = false;
    for (int step = 0; step < 4000; ++step) {
      const auto output = guide(current, target, limits, true);
      if (output.reached) {reached = true; break;}
      current.x += 0.05 * output.velocity.linear.x * std::cos(current.theta);
      current.y += 0.05 * output.velocity.linear.x * std::sin(current.theta);
      current.theta = normalizeAngle(current.theta + 0.05 * output.velocity.angular.z);
    }
    EXPECT_TRUE(reached);
    EXPECT_LE(std::hypot(current.x - target.x, current.y - target.y), limits.position_tolerance);
    EXPECT_LE(std::abs(normalizeAngle(current.theta - target.theta)), limits.yaw_tolerance);
  }
}

TEST(PlanarPathIO, RoundtripAndAuvCompatibility)
{
  const auto file = std::filesystem::temp_directory_path() /
    ("sura_usv_path_" + std::to_string(getpid()) + ".xml");
  std::string error;
  std::vector<Pose2D> points{pose(0, 0), pose(2, 3, 1)};
  ASSERT_TRUE(savePath(file.string(), "world_ned", points, error)) << error;
  std::ifstream input(file);
  const std::string text((std::istreambuf_iterator<char>(input)), {});
  EXPECT_EQ(text.find(" z="), std::string::npos);
  std::vector<Pose2D> loaded;
  ASSERT_TRUE(loadPath(file.string(), "world_ned", loaded, error)) << error;
  ASSERT_EQ(loaded.size(), 2U);
  EXPECT_DOUBLE_EQ(loaded[1].theta, 1.0);
  EXPECT_FALSE(loadPath(file.string(), "other_frame", loaded, error));
  {
    std::ofstream output(file);
    output << "<path frame_id='world_ned'><waypoint index='0' x='1' y='2' "
      "z='-5' yaw='0.4'/></path>";
  }
  ASSERT_TRUE(loadPath(file.string(), "world_ned", loaded, error)) << error;
  ASSERT_EQ(loaded.size(), 1U);
  EXPECT_DOUBLE_EQ(loaded[0].x, 1.0);
  {
    std::ofstream output(file);
    output << "<path><waypoint index='2' x='1' y='2' yaw='0'/></path>";
  }
  EXPECT_FALSE(loadPath(file.string(), "world_ned", loaded, error));
  EXPECT_EQ(loaded.size(), 1U);  // Failed loads do not replace the editor path.
  {
    std::ofstream output(file);
    output << "<path/>";
  }
  EXPECT_FALSE(loadPath(file.string(), "world_ned", loaded, error));
  std::filesystem::remove(file);
}
