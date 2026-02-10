/// @file
/// 
/// \brief contains ros_catch2 test for circle cmd_vel frequency

#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <chrono>

#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

TEST_CASE("circle publishes cmd_vel at expected frequency", "[integration]")
{
  auto node = rclcpp::Node::make_shared("integration_test_node_circle");

  node->declare_parameter<double>("expected_frequency", 10.0);
  node->declare_parameter<double>("measurement_seconds", 2.0);
  node->declare_parameter<double>("timeout_seconds", 5.0);

  const auto expected_frequency =
    node->get_parameter("expected_frequency").get_parameter_value().get<double>();
  const auto measurement_seconds =
    node->get_parameter("measurement_seconds").get_parameter_value().get<double>();
  const auto timeout_seconds =
    node->get_parameter("timeout_seconds").get_parameter_value().get<double>();

  auto msg_count = std::make_shared<std::atomic<int>>(0);
  auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    [msg_count](const geometry_msgs::msg::Twist::SharedPtr) {
      msg_count->fetch_add(1, std::memory_order_relaxed);
    });

  const auto timeout = std::chrono::duration<double>(timeout_seconds);
  auto wait_start = std::chrono::steady_clock::now();
  auto wait_rate = rclcpp::WallRate(5ms);

  while (cmd_vel_sub->get_publisher_count() == 0) {
    if (std::chrono::steady_clock::now() - wait_start > timeout) {
      FAIL("Timed out waiting for cmd_vel publisher");
    }
    rclcpp::spin_some(node);
    wait_rate.sleep();
  }

  auto warmup_end = std::chrono::steady_clock::now() + 200ms;
  while (std::chrono::steady_clock::now() < warmup_end) {
    rclcpp::spin_some(node);
    wait_rate.sleep();
  }
  msg_count->store(0, std::memory_order_relaxed);

  const auto measurement_duration = std::chrono::duration<double>(measurement_seconds);
  const auto measure_start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - measure_start < measurement_duration) {
    rclcpp::spin_some(node);
    wait_rate.sleep();
  }

  const auto elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - measure_start).count();
  const auto measured_frequency = msg_count->load(std::memory_order_relaxed) / elapsed;

  const auto tolerance = std::max(1.0, expected_frequency * 0.2);
  CHECK_THAT(measured_frequency, Catch::Matchers::WithinAbs(expected_frequency, tolerance));
}
