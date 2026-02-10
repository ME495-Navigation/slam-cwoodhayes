/// @file
/// 
/// \brief contains ros_catch2 test scaffold for odometry API

#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"

#include <queue>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

TEST_CASE("turtle_control odometry API test", "[integration]")
{
  auto node = rclcpp::Node::make_shared("integration_test_node");

  // TODO: add odometry integration checks here.
  CHECK(false);
}
