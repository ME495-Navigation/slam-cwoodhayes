/// \file circle.cpp
/// \brief contains a simple node that moves the turtle in a circle using cmd_vel.
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"
#include "turtle_control/srv/circle_control.hpp"

/// \brief Node for moving the turtle in a circle by publishing to cmd_vel.
///
/// Publishes:
/// - cmd_vel (geometry_msgs/msg/Twist)
///
/// Services:
/// - circle_control (turtle_control/srv/CircleControl)
/// - reverse (std_srvs/srv/Empty)
/// - stop (std_srvs/srv/Empty)
class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {
    auto qos = rclcpp::QoS(10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

    auto desc = rcl_interfaces::msg::ParameterDescriptor();
    desc.description = "Frequency to publish cmd_vel messages at";
    declare_parameter("frequency", 100.0, desc);
    auto frequency = get_parameter("frequency").as_double();
    if (frequency <= 0.0) {
      RCLCPP_WARN(get_logger(), "frequency must be > 0.0, defaulting to 100Hz");
      frequency = 100.0;
    }
    const auto period = std::chrono::duration<double>(1.0 / frequency);
    timer_ = create_wall_timer(period, std::bind(&Circle::timer_callback, this));

    circle_control_srv_ = create_service<turtle_control::srv::CircleControl>(
      "circle_control",
      std::bind(&Circle::circle_control_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    reverse_srv_ = create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&Circle::reverse_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    stop_srv_ = create_service<std_srvs::srv::Empty>(
      "stop",
      std::bind(&Circle::stop_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    // start off not circling
    angular_velocity_ = 0.0;
    circle_radius_ = 0.0;
    is_circling_ = false;

    RCLCPP_INFO(get_logger(), "circle node constructed.");
  }

private:
  void timer_callback()
  {
    auto msg = geometry_msgs::msg::Twist();
    // by kevination twist math:
    if (is_circling_) {
      const auto direction = is_forward_ ? 1.0 : -1.0;
      msg.linear.x = circle_radius_ * angular_velocity_ * direction;
      msg.angular.z = angular_velocity_ * direction;
    }
    cmd_vel_pub_->publish(msg);
  }

  void circle_control_callback(
    const std::shared_ptr<turtle_control::srv::CircleControl::Request> request,
    std::shared_ptr<turtle_control::srv::CircleControl::Response> response)
  {
    auto info_msg = std::format("Received circle control request (radius: {}, velocity: {})",
      request->radius, request->velocity);
    RCLCPP_INFO(get_logger(), info_msg.c_str());

    angular_velocity_ = request->velocity;
    circle_radius_ = request->radius;
    is_circling_ = true;

    response->success = true;
  }

  void reverse_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    is_forward_ = !is_forward_;
  }

  void stop_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    is_circling_ = false;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<turtle_control::srv::CircleControl>::SharedPtr circle_control_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;

  // angular velocity in rad/s
  double angular_velocity_ = 0.0;
  double circle_radius_ = 1.0;
  // forward is 1, backward is -1
  bool is_forward_ = true;
  bool is_circling_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
