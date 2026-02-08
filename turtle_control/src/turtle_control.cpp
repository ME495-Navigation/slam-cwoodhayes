/// \file
/// \brief contains turtle control node.


#include "rclcpp/rclcpp.hpp"

#include <functional>

#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


/// \brief Node for controlling the turtle in turtlesim.
///
/// Subscribes:
/// - cmd_vel (geometry_msgs/msg/Twist)
/// - sensor_data (nuturtlebot_msgs/msg/SensorData)
///
/// Publishes:
/// - wheel_cmd (nuturtlebot_msgs/msg/WheelCommands)
/// - joint_states (sensor_msgs/msg/JointState)
class TurtleControl : public rclcpp::Node
{
public:
  /// @brief constructor
  TurtleControl() : Node("turtle_control")
  {
    auto qos = rclcpp::QoS(10);
    wheel_cmd_pub_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", qos);
    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", qos);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", qos,
      std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));

    sensor_data_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", qos,
      std::bind(&TurtleControl::sensor_data_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "turtle_control node constructed.");
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr /*msg*/)
  {
    auto wheel_cmd = nuturtlebot_msgs::msg::WheelCommands{};
    wheel_cmd.left_velocity = 0;
    wheel_cmd.right_velocity = 0;
    wheel_cmd_pub_->publish(wheel_cmd);
  }

  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr /*msg*/)
  {
    auto joint_states = sensor_msgs::msg::JointState{};
    joint_states.header.stamp = now();
    joint_states.name = {"left_wheel", "right_wheel"};
    joint_states.position = {0.0, 0.0};
    joint_states.velocity = {0.0, 0.0};
    joint_states_pub_->publish(joint_states);
  }

  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
