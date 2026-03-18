/// \file
/// \brief Landmark detection node that processes laser scan data and publishes detected landmarks.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

/// \brief Node that detects landmarks from laser scan data.
///
/// Subscribes:
///   - red/scan (sensor_msgs::msg/LaserScan): Laser scan data from the simulator
///
/// Publishes:
///   - /landmarks (visualization_msgs::msg/MarkerArray): Detected landmarks as markers
class LandmarkDetector : public rclcpp::Node
{
public:
  /// @brief Node constructor
  LandmarkDetector()
  : Node("landmark_detector")
  {
    // Create subscription to laser scan
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "red/scan", 10,
      std::bind(&LandmarkDetector::scan_callback, this, std::placeholders::_1));

    // Create publisher for landmarks
    landmarks_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/detected_obstacles", rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(), "Landmark detector node started.");
  }

private:
  /// @brief Callback for laser scan messages
  /// @param msg The laser scan message
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto marker_array = visualization_msgs::msg::MarkerArray();

    // Create a dummy blue marker at (1, 2)
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "red/base_footprint";
    marker.header.stamp = get_clock()->now();
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set position (dummy landmark at 1, 2)
    marker.pose.position.x = 1.0;
    marker.pose.position.y = 2.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set scale
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set blue color
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    marker_array.markers.push_back(marker);
    landmarks_publisher_->publish(marker_array);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmarks_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandmarkDetector>());
  rclcpp::shutdown();
  return 0;
}
