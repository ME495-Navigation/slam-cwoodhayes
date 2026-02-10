/// @file
/// \brief contains ros_catch2 test for turtle_control_node

#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"

#include <queue>

#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

// ####################### Begin_Citation[11] ##################### --- IGNORE ---
// The following code is adapted from the example integration test provided in the catch_ros2 package:

TEST_CASE("turtle_control test - cmd_vel to wheel_cmd", "[integration]") {
  // Create a simple client node to check if the auxiliary node
  // has a service available
  auto node = rclcpp::Node::make_shared("integration_test_node");

  // Declare a parameter on the node
  // (the default catch_ros2 node main will allow ROS arguments
  // like parameters to be passed to nodes in test files)
  node->declare_parameter<double>("test_duration");
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  // create a publisher for cmd_vel to command the turtle_control node,
  // which will then publish wheel commands that we can listen for to verify the node is working
  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  
  // and a subscriber to listen for wheel commands published by turtle_control
  auto wheel_cmd_recv_queue = std::make_shared<std::queue<nuturtlebot_msgs::msg::WheelCommands::SharedPtr>>();
  auto wheel_cmd_sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd", 10,
    [wheel_cmd_recv_queue](const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) {
      RCLCPP_INFO(rclcpp::get_logger("integration_test_node"), "Received wheel command: left=%d, right=%d",
                  msg->left_velocity, msg->right_velocity);
      wheel_cmd_recv_queue->push(msg);
    });

  // HELPER FUNCTIONS
  auto wait_for_subs_and_pubs = [&]() {
    auto timeout = std::chrono::duration<double>(TEST_DURATION);
    auto wait_start = std::chrono::steady_clock::now();
    auto wait_rate = rclcpp::WallRate(50ms);

    while (cmd_vel_pub->get_subscription_count() == 0 || wheel_cmd_sub->get_publisher_count() == 0) {
      if (std::chrono::steady_clock::now() - wait_start > timeout) {
        FAIL("Timed out waiting for pub/sub matching on cmd_vel or wheel_cmd");
      }
      rclcpp::spin_some(node);
      wait_rate.sleep();
    }
  };

  auto wait_for_wheel_cmd = [&]() {
    auto timeout = std::chrono::duration<double>(TEST_DURATION);
    auto wait_start = std::chrono::steady_clock::now();
    auto wait_rate = rclcpp::WallRate(50ms);

    while (wheel_cmd_recv_queue->empty()) {
      if (std::chrono::steady_clock::now() - wait_start > timeout) {
        FAIL("Timed out waiting for wheel_cmd message");
      }
      rclcpp::spin_some(node);
      wait_rate.sleep();
    }
  };

  // test that verifies that cmd_vel commands with pure translation result in 
  // the appropriate wheel_cmd being published.
  SECTION("pure translation cmd_vel -> wheel commands") {
    wait_for_subs_and_pubs();

    // publish a cmd_vel with pure translation
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = 0.5; // 0.5 m/s forward
    cmd.angular.z = 0.0; // no rotation
    cmd_vel_pub->publish(cmd);

    wait_for_wheel_cmd();

    CHECK(wheel_cmd_recv_queue->size() == 1);
    // check that wheel commands are approx equal (due to straight line)
    auto received_cmd = wheel_cmd_recv_queue->front();
    CHECK_THAT(received_cmd->left_velocity, Catch::Matchers::WithinAbs(received_cmd->right_velocity, 3));
  }

  // test that verifies that cmd_vel commands with pure rotation result in
  // the appropriate wheel_cmd being published.
  SECTION("pure rotation cmd_vel -> wheel commands") {
    wait_for_subs_and_pubs();
    // publish a cmd_vel with pure rotation
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = 0.0; // no forward motion
    cmd.angular.z = 1.0; // 1 rad/s counterclockwise
    cmd_vel_pub->publish(cmd);

    wait_for_wheel_cmd();

    CHECK(wheel_cmd_recv_queue->size() == 1);
    // check that wheel commands are approx equal and opposite (due to rotation in place)
    auto received_cmd = wheel_cmd_recv_queue->front();
    CHECK_THAT(received_cmd->left_velocity, Catch::Matchers::WithinAbs(-received_cmd->right_velocity, 3)); 
  }

  // test that verifies that encoder data on sensors is converted to joint_states properly
}
// ####################### End_Citation[11] ##################### --- IGNORE ---

TEST_CASE("turtle_control test - sensor_data to joint_states", "[integration]") {
  auto node = rclcpp::Node::make_shared("integration_test_node");

  node->declare_parameter<double>("test_duration", 2.0);
  const auto test_duration =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  auto sensor_data_pub = node->create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);
  auto joint_state_recv_queue = std::make_shared<std::queue<sensor_msgs::msg::JointState::SharedPtr>>();
  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    [joint_state_recv_queue](const sensor_msgs::msg::JointState::SharedPtr msg) {
      RCLCPP_INFO(
        rclcpp::get_logger("integration_test_node"),
        "Received joint state with %zu positions", msg->position.size());
      joint_state_recv_queue->push(msg);
    });

  auto wait_for_subs_and_pubs = [&]() {
    const auto timeout = std::chrono::duration<double>(test_duration);
    auto wait_start = std::chrono::steady_clock::now();
    auto wait_rate = rclcpp::WallRate(50ms);

    while (sensor_data_pub->get_subscription_count() == 0 ||
           joint_state_sub->get_publisher_count() == 0) {
      if (std::chrono::steady_clock::now() - wait_start > timeout) {
        FAIL("Timed out waiting for pub/sub matching on sensor_data or joint_states");
      }
      rclcpp::spin_some(node);
      wait_rate.sleep();
    }
  };

  auto wait_for_joint_state = [&]() {
    const auto timeout = std::chrono::duration<double>(test_duration);
    auto wait_start = std::chrono::steady_clock::now();
    auto wait_rate = rclcpp::WallRate(50ms);

    while (joint_state_recv_queue->empty()) {
      if (std::chrono::steady_clock::now() - wait_start > timeout) {
        FAIL("Timed out waiting for joint_states message");
      }
      rclcpp::spin_some(node);
      wait_rate.sleep();
    }
  };

  SECTION("sensor_data -> joint_states") {
    wait_for_subs_and_pubs();

    auto sensor = nuturtlebot_msgs::msg::SensorData();
    sensor.left_encoder = 100;
    sensor.right_encoder = 200;
    sensor_data_pub->publish(sensor);

    wait_for_joint_state();

    // basic checks
    CHECK(joint_state_recv_queue->size() == 1);
    const auto received_state = joint_state_recv_queue->front();
    CHECK(received_state->position.size() >= 2);
    CHECK(received_state->name.size() >= 2);
    CHECK(received_state->position.at(0) != received_state->position.at(1));
  }
}