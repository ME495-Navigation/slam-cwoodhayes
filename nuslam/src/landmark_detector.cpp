/// \file
/// \brief Landmark detection node that processes laser scan data and publishes detected landmarks.

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "turtlelib/circle_fit.hpp"

#include <memory>

/// \brief Node that detects landmarks from laser scan data.
///
/// Subscribes:
///   - red/scan (sensor_msgs::msg/LaserScan): Laser scan data from the simulator
///
/// Publishes:
///   - /landmarks (visualization_msgs::msg/MarkerArray): Detected landmarks as cylinders
///
/// Parameters:
///   - obstacles.h (double): Height of cylinder markers for detected obstacles
///   - distance_threshold (double): Maximum distance between consecutive points in a LiDAR cluster
class LandmarkDetector : public rclcpp::Node
{
public:
  /// @brief Node constructor
  LandmarkDetector()
  : Node("landmark_detector")
  {
    // Declare obstacle height parameter
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Height of cylinder obstacles";
      declare_parameter("obstacles.h", 0.25, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Maximum distance between consecutive points in a LiDAR cluster";
      declare_parameter("distance_threshold", 0.01, desc);
    }

    cylinder_height_ = get_parameter("obstacles.h").as_double();
    auto distance_threshold = get_parameter("distance_threshold").as_double();
    detector_ = std::make_unique<turtlelib::CylinderDetector>(distance_threshold);

    // Create subscription to laser scan
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "red/scan", 10,
      std::bind(&LandmarkDetector::scan_callback, this, std::placeholders::_1));

    // Create publisher for landmarks
    landmarks_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/landmarks", rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(), "Landmark detector node started.");
  }

private:
  /// @brief Callback for laser scan messages
  /// @param msg The laser scan message
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto marker_array = visualization_msgs::msg::MarkerArray();

    // Convert float ranges to double
    std::vector<double> ranges;
    ranges.reserve(msg->ranges.size());
    for (const auto range : msg->ranges) {
      ranges.push_back(static_cast<double>(range));
    }

    // Detect circles from the LiDAR scan
    auto detected_circles = detector_->detect(ranges, msg->angle_increment);

    auto debugmsg = std::format(
      "Detected {} circles in laser scan ({})", 
      detected_circles.size(), ranges.size());
    RCLCPP_INFO(get_logger(), debugmsg.c_str());

    // Create markers for each detected circle
    for (size_t i = 0; i < detected_circles.size(); ++i) {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "red/base_footprint";
      marker.header.stamp = get_clock()->now();
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;

      // Set position from detected circle center
      marker.pose.position.x = detected_circles[i].center.x;
      marker.pose.position.y = detected_circles[i].center.y;
      marker.pose.position.z = cylinder_height_ / 2.0;
      marker.pose.orientation.w = 1.0;

      // Set scale (diameter and height)
      const auto diameter = 2.0 * detected_circles[i].radius;
      marker.scale.x = diameter;
      marker.scale.y = diameter;
      marker.scale.z = cylinder_height_;

      // Set blue color
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;

      marker_array.markers.push_back(marker);
    }

    landmarks_publisher_->publish(marker_array);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmarks_publisher_;
  std::unique_ptr<turtlelib::CylinderDetector> detector_;
  double cylinder_height_{};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandmarkDetector>());
  rclcpp::shutdown();
  return 0;
}
