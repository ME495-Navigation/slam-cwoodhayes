/// \file
/// \brief contains turtle control node.


#include "rclcpp/rclcpp.hpp"

#include <functional>

#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"


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
    
    // fetch necessary robot parameters from diff_params.yaml
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
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Maximum motor command";
      declare_parameter("motor_cmd_max", 265, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Motor command per rad/sec";
      declare_parameter("motor_cmd_per_rad_sec", 0.024, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Encoder ticks per rad";
      declare_parameter("encoder_ticks_per_rad", 651.8986, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Collision radius";
      declare_parameter("collision_radius", 0.11, desc);
    }

    auto wheel_radius = get_parameter("wheel_radius").as_double();
    auto track_width = get_parameter("track_width").as_double();
    auto motor_cmd_max = get_parameter("motor_cmd_max").as_int();
    auto motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    auto encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    auto collision_radius = get_parameter("collision_radius").as_double();

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
