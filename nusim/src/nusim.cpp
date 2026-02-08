/// \file
/// \brief contains SLAM simulator node.

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

/// @brief Simulator node.
class NUSimulator : public rclcpp::Node
{
public:
  /// @brief Node constructor
  NUSimulator()
  : Node("nusimulator"), count_(0), gt_pose_()
  {
    auto rate_desc = rcl_interfaces::msg::ParameterDescriptor();
    rate_desc.description = "Sim rate in Hz";
    declare_parameter("rate", 100.0, rate_desc);
    auto rate = get_parameter("rate").as_double();

    // - start pose
    auto x0_desc = rcl_interfaces::msg::ParameterDescriptor();
    x0_desc.description = "Initial x position";
    declare_parameter("x0", 0.0, x0_desc);

    auto y0_desc = rcl_interfaces::msg::ParameterDescriptor();
    y0_desc.description = "Initial y position";
    declare_parameter("y0", 0.0, y0_desc);

    auto theta0_desc = rcl_interfaces::msg::ParameterDescriptor();
    theta0_desc.description = "Initial theta angle";
    declare_parameter("theta0", 0.0, theta0_desc);
    gt_pose_ = get_pose0();

    // - arena walls
    auto ax_desc = rcl_interfaces::msg::ParameterDescriptor();
    ax_desc.description = "length of the arena in the world X direction";
    declare_parameter("arena_x_length", 10.0, ax_desc);

    auto ay_desc = rcl_interfaces::msg::ParameterDescriptor();
    ay_desc.description = "length of the arena in the world Y direction";
    declare_parameter("arena_y_length", 10.0, ay_desc);

    // - cylindrical obstacles
    auto obs_x_desc = rcl_interfaces::msg::ParameterDescriptor();
    obs_x_desc.description = "X coordinates of obstacles";
    declare_parameter("obstacles.x", std::vector<double>{}, obs_x_desc);

    auto obs_y_desc = rcl_interfaces::msg::ParameterDescriptor();
    obs_y_desc.description = "Y coordinates of obstacles";
    declare_parameter("obstacles.y", std::vector<double>{}, obs_y_desc);
    auto obs_r_desc = rcl_interfaces::msg::ParameterDescriptor();
    obs_r_desc.description = "Radius of obstacles";
    declare_parameter("obstacles.r", 0.0, obs_r_desc);

    // Validate that x and y have same length
    auto obs_x = get_parameter("obstacles.x").as_double_array();
    auto obs_y = get_parameter("obstacles.y").as_double_array();
    if (obs_x.size() != obs_y.size()) {
      RCLCPP_ERROR(get_logger(), "obstacles.x and obstacles.y must have the same length");
      throw std::runtime_error("obstacles.x and obstacles.y must have the same length");
    }

    publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), std::bind(&NUSimulator::timer_callback, this));

    reset_service_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&NUSimulator::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // create arena walls publisher
    rclcpp::QoS qos(10);
    qos.transient_local();
    walls_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls", qos);

    // create obstacles publisher
    obstacles_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/real_obstacles", qos);
    RCLCPP_INFO(get_logger(), "nusimulator node constructed.");

    publish_arena();
    publish_cyl_obstacles();
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = count_++;
    publisher_->publish(message);

    // broadcast transform from nusim/world to red/base_footprint
    auto transform = geometry_msgs::msg::TransformStamped();
    transform.header.stamp = get_clock()->now();
    transform.header.frame_id = "nusim/world";
    transform.child_frame_id = "red/base_footprint";

    // set translation from groundtruth pose
    transform.transform.translation.x = gt_pose_.translation().x;
    transform.transform.translation.y = gt_pose_.translation().y;
    transform.transform.translation.z = 0.0;

    // set rotation, convert angle to quaternion restricted to 2d plane
    const auto theta = gt_pose_.rotation();
    const auto quat = turtlelib::angle_to_2d_planar_quaternion(theta);
    transform.transform.rotation.x = quat[0];
    transform.transform.rotation.y = quat[1];
    transform.transform.rotation.z = quat[2];
    transform.transform.rotation.w = quat[3];

    tf_broadcaster_->sendTransform(transform);
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    count_ = 0;
    gt_pose_ = get_pose0();
    const auto msg = std::format(
      "Simulation reset: timestep set to 0, robot reset to initial pose "
      "({}).",
      gt_pose_);
    RCLCPP_INFO(get_logger(), msg.c_str());
  }

  /// @brief get the initial pose of the robot from parameters
  /// @return tf for the initial pose (x0, y0, theta0)
  turtlelib::Transform2D get_pose0()
  {
    double x = get_parameter("x0").as_double();
    double y = get_parameter("y0").as_double();
    const auto theta = get_parameter("theta0").as_double();
    return {{x, y}, theta};
  }

  /// @brief publish the walls of the arena according to the parameters
  void publish_arena()
  {
    double x_len = get_parameter("arena_x_length").as_double();
    double y_len = get_parameter("arena_y_length").as_double();
    const double wall_height = 0.25;
    const double wall_thickness = 0.05;

    auto marker_array = visualization_msgs::msg::MarkerArray();
    int marker_id = 0;

    // Calculate the outer bounds (arena edges)
    double x_min = -x_len / 2.0;
    double x_max = x_len / 2.0;
    double y_min = -y_len / 2.0;
    double y_max = y_len / 2.0;

    // wall definitions
    std::vector<std::tuple<double, double, double, double>> walls = {
      // bottom
      {0.0, y_min - wall_thickness / 2.0, x_len + 2.0 * wall_thickness, wall_thickness},
      // top
      {0.0, y_max + wall_thickness / 2.0, x_len + 2.0 * wall_thickness, wall_thickness},
      // left
      {x_min - wall_thickness / 2.0, 0.0, wall_thickness, y_len},
      // right
      {x_max + wall_thickness / 2.0, 0.0, wall_thickness, y_len}};

    for (const auto & [x_pos, y_pos, x_scale, y_scale] : walls) {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "nusim/world";
      marker.header.stamp = get_clock()->now();
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.ns = "red";

      marker.pose.position.x = x_pos;
      marker.pose.position.y = y_pos;
      marker.pose.position.z = wall_height / 2.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = x_scale;
      marker.scale.y = y_scale;
      marker.scale.z = wall_height;

      // walls must be red
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      marker_array.markers.push_back(marker);
    }

    walls_publisher_->publish(marker_array);
  }

  /// @brief publish cylindrical obstacles as configured on startup.
  void publish_cyl_obstacles()
  {
    auto obs_x = get_parameter("obstacles.x").as_double_array();
    auto obs_y = get_parameter("obstacles.y").as_double_array();
    double obs_r = get_parameter("obstacles.r").as_double();
    const double cyl_height = 0.25;

    auto marker_array = visualization_msgs::msg::MarkerArray();

    for (size_t i = 0; i < obs_x.size(); ++i) {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "nusim/world";
      marker.header.stamp = get_clock()->now();
      marker.ns = "red";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = obs_x[i];
      marker.pose.position.y = obs_y[i];
      marker.pose.position.z = cyl_height / 2.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 2.0 * obs_r;  // diameter
      marker.scale.y = 2.0 * obs_r;  // diameter
      marker.scale.z = cyl_height;

      // red color
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      marker_array.markers.push_back(marker);
    }

    obstacles_publisher_->publish(marker_array);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /// @brief simulation timestep
  size_t count_;
  /// @brief ground truth pose of the robot
  turtlelib::Transform2D gt_pose_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NUSimulator>());
  rclcpp::shutdown();
  return 0;
}
