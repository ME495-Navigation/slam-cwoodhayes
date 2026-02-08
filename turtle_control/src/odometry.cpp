/// \file
/// \brief contains odometry node.

#include "nav_msgs/msg/odometry.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <functional>

/// \brief Node for controlling the turtle in turtlesim.
///
/// Subscribes:
/// - cmd_vel (geometry_msgs/msg/Twist)
/// - sensor_data (nuturtlebot_msgs/msg/SensorData)
///
/// Publishes:
/// - wheel_cmd (nuturtlebot_msgs/msg/WheelCommands)
/// - joint_states (sensor_msgs/msg/JointState)
class Odometry : public rclcpp::Node
{
public:
  /// @brief constructor
  Odometry() : Node("odometry")
  {
    auto qos = rclcpp::QoS(10);

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", qos, std::bind(&Odometry::joint_states_cb, this, std::placeholders::_1));

    // fetch necessary robot parameters from diff_params.yaml
    /**
    body_id: The name of the body frame of the robot. Defaults to base_footprint.
    odom_id: The name of the odometry frame. Defaults to odom.
    wheel_left: The name of the left wheel joint. If not specified, log an error and exit.
    wheel_right: The name of the right wheel joint. If not specified, log an error and exit.
    **/
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Name of the body frame of the robot";
      declare_parameter("body_id", "base_footprint", desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Name of the odometry frame";
      declare_parameter("odom_id", "odom", desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Name of the left wheel joint";
      declare_parameter("wheel_left", "", desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Name of the right wheel joint";
      declare_parameter("wheel_right", "", desc);
    }
    // we also need some params from diff_params.yaml
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Radius of the wheels";
      declare_parameter("wheel_radius", 0.1, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Distance between the wheels";
      declare_parameter("track_width", 0.5, desc);
    }
    // construct DiffDrive object with parameters
    // diff_drive_ = std::make_unique<turtlelib::DiffDrive>(wheel_radius, track_width);

    RCLCPP_INFO(get_logger(), "odometry node constructed.");
  }

private:
  /// @brief Callback function for joint_states topic. Computes and publishes tbot3 odometry.
  void joint_states_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto odom_msg = nav_msgs::msg::Odometry();
    // TODO
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<turtlelib::DiffDrive> diff_drive_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
