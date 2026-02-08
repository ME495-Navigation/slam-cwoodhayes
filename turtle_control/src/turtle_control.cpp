/// \file
/// \brief contains turtle control node.


#include "rclcpp/rclcpp.hpp"


/// \brief Node for controlling the turtle in turtlesim.
class TurtleControl : public rclcpp::Node
{
public:
  /// @brief constructor
  TurtleControl() : Node("turtle_control")
  {
    RCLCPP_INFO(get_logger(), "turtle_control node constructed.");
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
