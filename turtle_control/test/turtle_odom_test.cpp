/// @file
/// 
/// \brief contains ros_catch2 test scaffold for odometry API

#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"

#include <queue>

#include "nav_msgs/msg/odometry.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/angle.hpp"
#include "turtle_control/srv/set_pose.hpp"

using namespace std::chrono_literals;

TEST_CASE("turtle_control odometry API test", "[integration]")
{
  auto node = rclcpp::Node::make_shared("integration_test_node");

  auto odom_recv_queue = std::make_shared<std::queue<nav_msgs::msg::Odometry::SharedPtr>>();
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    [odom_recv_queue](const nav_msgs::msg::Odometry::SharedPtr msg) {
      odom_recv_queue->push(msg);
    });
  auto sensor_data_pub =
    node->create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);

  auto client = node->create_client<turtle_control::srv::SetPose>("set_initial_pose");

  REQUIRE(client->wait_for_service(2s));

  auto request = std::make_shared<turtle_control::srv::SetPose::Request>();
  request->x = 1.0;
  request->y = 2.0;
  request->theta = 0.5;

  auto future = client->async_send_request(request);
  const auto status = rclcpp::spin_until_future_complete(node, future, 2s);

  REQUIRE(status == rclcpp::FutureReturnCode::SUCCESS);
  const auto response = future.get();
  CHECK(response->success);

  // check that the service call actually set the pose
  // using the odometry topic
  const auto timeout = 2s;
  auto wait_start = std::chrono::steady_clock::now();
  auto wait_rate = rclcpp::WallRate(50ms);

  while (sensor_data_pub->get_subscription_count() == 0 ||
         odom_sub->get_publisher_count() == 0) {
    if (std::chrono::steady_clock::now() - wait_start > timeout) {
      FAIL("Timed out waiting for pub/sub matching on sensor_data or odom");
    }
    rclcpp::spin_some(node);
    wait_rate.sleep();
  }

  auto sensor = nuturtlebot_msgs::msg::SensorData();
  sensor.left_encoder = 0;
  sensor.right_encoder = 0;
  sensor_data_pub->publish(sensor);

  wait_start = std::chrono::steady_clock::now();

  while (odom_recv_queue->empty()) {
    if (std::chrono::steady_clock::now() - wait_start > timeout) {
      FAIL("Timed out waiting for odom message");
    }
    rclcpp::spin_some(node);
    wait_rate.sleep();
  }

  const auto odom_msg = odom_recv_queue->front();
  CHECK_THAT(odom_msg->pose.pose.position.x, Catch::Matchers::WithinAbs(request->x, 1e-3));
  CHECK_THAT(odom_msg->pose.pose.position.y, Catch::Matchers::WithinAbs(request->y, 1e-3));

  const auto expected_quat = turtlelib::angle_to_2d_planar_quaternion(request->theta);
  CHECK_THAT(odom_msg->pose.pose.orientation.x, Catch::Matchers::WithinAbs(expected_quat[0], 1e-3));
  CHECK_THAT(odom_msg->pose.pose.orientation.y, Catch::Matchers::WithinAbs(expected_quat[1], 1e-3));
  CHECK_THAT(odom_msg->pose.pose.orientation.z, Catch::Matchers::WithinAbs(expected_quat[2], 1e-3));
  CHECK_THAT(odom_msg->pose.pose.orientation.w, Catch::Matchers::WithinAbs(expected_quat[3], 1e-3));
}
