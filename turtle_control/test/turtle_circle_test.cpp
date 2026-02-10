/// @file
/// 
/// \brief contains ros_catch2 test for circle cmd_vel frequency

#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

TEST_CASE("circle publishes cmd_vel at expected frequency", "[integration]")
{
  auto node = rclcpp::Node::make_shared("integration_test_node_circle");

  node->declare_parameter<double>("expected_frequency", 10.0);
  node->declare_parameter<int>("sample_count", 20);
  node->declare_parameter<double>("timeout_seconds", 5.0);

  const auto expected_frequency =
    node->get_parameter("expected_frequency").get_parameter_value().get<double>();
  const auto sample_count =
    node->get_parameter("sample_count").get_parameter_value().get<int>();
  const auto timeout_seconds =
    node->get_parameter("timeout_seconds").get_parameter_value().get<double>();

  auto recv_times = std::make_shared<std::vector<std::chrono::steady_clock::time_point>>();
  auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    [recv_times](const geometry_msgs::msg::Twist::SharedPtr) {
      recv_times->push_back(std::chrono::steady_clock::now());
    });

  const auto timeout = std::chrono::duration<double>(timeout_seconds);
  auto wait_start = std::chrono::steady_clock::now();
  auto wait_rate = rclcpp::WallRate(200ms);

  while (cmd_vel_sub->get_publisher_count() == 0) {
    if (std::chrono::steady_clock::now() - wait_start > timeout) {
      FAIL("Timed out waiting for cmd_vel publisher");
    }
    rclcpp::spin_some(node);
    wait_rate.sleep();
  }

  wait_start = std::chrono::steady_clock::now();
  while (static_cast<int>(recv_times->size()) < sample_count) {
    if (std::chrono::steady_clock::now() - wait_start > timeout) {
      FAIL("Timed out waiting for cmd_vel samples");
    }
    rclcpp::spin_some(node);
    wait_rate.sleep();
  }

  REQUIRE(recv_times->size() >= 2);
  const auto duration = recv_times->back() - recv_times->front();
  const auto elapsed = std::chrono::duration<double>(duration).count();
  const auto measured_frequency = (recv_times->size() - 1) / elapsed;

  const auto tolerance = std::max(1.0, expected_frequency * 0.1);
  CHECK_THAT(measured_frequency, Catch::Matchers::WithinAbs(expected_frequency, tolerance));
}
