/// \file
/// \brief contains SLAM simulator node.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

/// @brief Simulator node.
class NUSimulator : public rclcpp::Node
{
public:
    /// @brief Node constructor 
    NUSimulator()
        : Node("nusimulator"), count_(0)
    {
        // declare parameters
        // - rate
        auto rate_desc = rcl_interfaces::msg::ParameterDescriptor();
        rate_desc.description = "Sim rate in Hz";
        this->declare_parameter("rate", 100.0, rate_desc);
        double rate = this->get_parameter("rate").as_double();

        // - start pose
        auto x0_desc = rcl_interfaces::msg::ParameterDescriptor();
        x0_desc.description = "Initial x position";
        this->declare_parameter("x0", 0.0, x0_desc);
        
        auto y0_desc = rcl_interfaces::msg::ParameterDescriptor();
        y0_desc.description = "Initial y position";
        this->declare_parameter("y0", 0.0, y0_desc);
        
        auto theta0_desc = rcl_interfaces::msg::ParameterDescriptor();
        theta0_desc.description = "Initial theta angle";
        this->declare_parameter("theta0", 0.0, theta0_desc);
        
        publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate), 
            std::bind(&NUSimulator::timer_callback, this));
        
        reset_service_ = this->create_service<std_srvs::srv::Empty>(
            "~/reset",
            std::bind(&NUSimulator::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "nusimulator node constructed.");
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::UInt64();
        message.data = count_++;
        publisher_->publish(message);
    }

    void reset_callback(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        count_ = 0;
        RCLCPP_INFO(this->get_logger(), "Simulation reset: timestep set to 0");
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NUSimulator>());
    rclcpp::shutdown();
    return 0;
}