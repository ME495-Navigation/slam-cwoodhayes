#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

using namespace std::chrono_literals;

/// @brief Simulator node.
class NUSimulator : public rclcpp::Node
{
public:
    /// @brief Node constructor 
    NUSimulator()
        : Node("nusimulator"), count_(0)
    {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Sim rate in Hz";
        this->declare_parameter("rate", 100.0, param_desc);
        double rate = this->get_parameter("rate").as_double();
        
        publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate), 
            std::bind(&NUSimulator::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "nusimulator node constructed.");
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::UInt64();
        message.data = count_++;
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NUSimulator>());
    rclcpp::shutdown();
    return 0;
}