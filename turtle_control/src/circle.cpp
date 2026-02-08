
/// \file circle.cpp
/// \brief contains a simple node that moves the turtle in a circle using cmd_vel.
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

/// @brief Node for moving the turtle in a circle by publishing to cmd_vel.
class Circle : public rclcpp::Node
{
public:
  Circle() : Node("circle")
  {
    auto qos = rclcpp::QoS(10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
    timer_ =
      create_wall_timer(std::chrono::milliseconds(100), std::bind(&Circle::timer_callback, this));
    
    RCLCPP_INFO(get_logger(), "circle node constructed.");
  }

private:
  void timer_callback()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.5;
    msg.angular.z = 0.5;
    cmd_vel_pub_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}