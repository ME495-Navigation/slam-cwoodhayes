/// @file
/// \brief contains ros_catch2 test for turtle_control_node

#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"

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

TEST_CASE("example_integration_test", "[integration]") {
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
  auto wheel_cmd_sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd", 10,
    [](const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) {
      RCLCPP_INFO(rclcpp::get_logger("integration_test_node"), "Received wheel command: left=%d, right=%d",
                  msg->left_velocity, msg->right_velocity);
    });

  // Keep test running only for the length of the "test_duration" parameter
  // (in seconds)

  // test that verifies that cmd_vel commands with pure translation result in 
  // the appropriate wheel_cmd being published.
  SECTION("pure translation cmd_vel -> wheel commands") {
    // publish a cmd_vel with pure translation
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = 0.5; // 0.5 m/s forward
    cmd.angular.z = 0.0; // no rotation
    cmd_vel_pub->publish(cmd);

    // tick so that the turtle_control node processes the cmd_vel and publishes a wheel_cmd
    rclcpp::spin_some(node);

    // TODO get wheel commands and check that the wheels are moving at the same rate.
    CHECK(false); // replace with actual check
  }
}
// ####################### End_Citation[11] ##################### --- IGNORE ---