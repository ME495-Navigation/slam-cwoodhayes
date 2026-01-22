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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"

using namespace std::chrono_literals;

/// @brief Simulator node.
class NUSimulator : public rclcpp::Node
{
public:
    /// @brief Node constructor 
    NUSimulator()
        : Node("nusimulator"), count_(0), gt_pose_()
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
        gt_pose_ = get_pose0();
        
        publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate), 
            std::bind(&NUSimulator::timer_callback, this));
        
        reset_service_ = this->create_service<std_srvs::srv::Empty>(
            "~/reset",
            std::bind(&NUSimulator::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        RCLCPP_INFO(this->get_logger(), "nusimulator node constructed.");
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::UInt64();
        message.data = count_++;
        publisher_->publish(message);
        
        // broadcast transform from nusim/world to red/base_footprint
        auto transform = geometry_msgs::msg::TransformStamped();
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "nusim/world";
        transform.child_frame_id = "red/base_footprint";
        
        // set translation from groundtruth pose
        transform.transform.translation.x = gt_pose_.translation().x;
        transform.transform.translation.y = gt_pose_.translation().y;
        transform.transform.translation.z = 0.0;
        
        // set rotation, convert angle to quaternion restricted to 2d plane
        double theta = gt_pose_.rotation();
        double half_theta = theta / 2.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = std::sin(half_theta);
        transform.transform.rotation.w = std::cos(half_theta);
        
        tf_broadcaster_->sendTransform(transform);
    }

    void reset_callback(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        count_ = 0;
        gt_pose_ = get_pose0();
        const auto msg = std::format("Simulation reset: timestep set to 0, robot reset to initial pose ({}).", gt_pose_);
        RCLCPP_INFO(this->get_logger(), msg.c_str());
    }

    /// @brief get the initial pose of the robot from parameters
    /// @return tf for the initial pose (x0, y0, theta0)
    turtlelib::Transform2D get_pose0() {
        double x = this->get_parameter("x0").as_double();
        double y = this->get_parameter("y0").as_double();
        double theta = this->get_parameter("theta0").as_double();
        return {{x, y}, theta};
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /// @brief simulation timestep
    size_t count_;
    /// @brief ground truth pose of the robot
    turtlelib::Transform2D gt_pose_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NUSimulator>());
    rclcpp::shutdown();
    return 0;
}