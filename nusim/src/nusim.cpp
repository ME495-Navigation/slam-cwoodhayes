/// \file
/// \brief contains SLAM simulator node.

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/angle.hpp"
#include "turtlelib/noise_models.hpp"
#include "turtlelib/obstacles.hpp"
#include "turtlelib/lidar.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <format>
#include <vector>
#include <algorithm>
#include <cstdint>
#include <queue>

using namespace std::chrono_literals;

/// @class NUSimulator
/// @brief ROS2 node that simulates a differential drive robot (TurtleBot3) in a 2D environment.
///
/// This simulator maintains ground truth pose and wheel states, publishes sensor data and transforms,
/// and responds to wheel command inputs.
///
/// @details Publishers:
///   - `~/timestep` (std_msgs::msg::UInt64): Current simulation timestep counter
///   - `~/real_walls` (visualization_msgs::msg::MarkerArray): Arena boundary walls for visualization
///   - `~/real_obstacles` (visualization_msgs::msg::MarkerArray): Cylindrical obstacles for visualization
///   - `red/sensor_data` (nuturtlebot_msgs::msg::SensorData): Simulated sensor output (ie encoder readings) for the robot
///   - `/fake_sensor` (visualization_msgs::msg::MarkerArray): Obstacle detections relative to robot at 5Hz with Gaussian noise
///
/// @details Subscribers:
///   - `red/wheel_cmd` (nuturtlebot_msgs::msg::WheelCommands): Motor commands to the robot
///
/// @details Services:
///   - `~/reset` (std_srvs::srv::Empty): Resets simulation timestep and robot pose to initial state
///
/// @details Broadcasts:
///   - Transform from "nusim/world" to "red/base_footprint" with ground truth pose at each timestep
class NUSimulator : public rclcpp::Node
{
public:
  /// @brief Node constructor

  NUSimulator()
  : Node("nusimulator"), count_(0), gt_pose_()
  {
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Sim rate in Hz";
      declare_parameter("rate", 100.0, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Initial x position";
      declare_parameter("x0", 0.0, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Initial y position";
      declare_parameter("y0", 0.0, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Initial theta angle";
      declare_parameter("theta0", 0.0, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "length of the arena in the world X direction";
      declare_parameter("arena_x_length", 10.0, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "length of the arena in the world Y direction";
      declare_parameter("arena_y_length", 10.0, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "X coordinates of obstacles";
      declare_parameter("obstacles.x", std::vector<double>{}, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Y coordinates of obstacles";
      declare_parameter("obstacles.y", std::vector<double>{}, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Radius of obstacles";
      declare_parameter("obstacles.r", 0.0, desc);
    }

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
      desc.description = "Turtlebot3 collision radius";
      declare_parameter("collision_radius", 0.11, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "If true, draw the landmarks and do nothing else (for when connected to the actual robot)";
      declare_parameter("draw_only", false, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Maximum number of poses in the ground truth path";
      declare_parameter("max_path_size", 1000, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Gaussian noise variance to add to wheel velocity commands";
      declare_parameter("input_noise", 0.02, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Wheel slip fraction -- bounds of uniform distribution to add slip noise proportional to wheel velocity";
      declare_parameter("slip_fraction", 0.1, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Variance of zero-mean Gaussian noise to add to obstacle sensor measurements";
      declare_parameter("basic_sensor_variance", 0.0, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Maximum range (in meters) for obstacle detection by the fake sensor";
      declare_parameter("max_range", 5.0, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Minimum range of the LiDAR in meters";
      declare_parameter("lidar_range_min", 0.12, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Maximum range of the LiDAR in meters";
      declare_parameter("lidar_range_max", 3.5, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Angular resolution of the LiDAR in radians";
      declare_parameter("lidar_angle_increment", 0.01745329, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Range resolution of the LiDAR in meters";
      declare_parameter("lidar_resolution", 0.015, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Variance of zero-mean Gaussian noise to add to LiDAR range measurements";
      declare_parameter("lidar_noise_variance", 0.0, desc);
    }

    auto wheel_radius = get_parameter("wheel_radius").as_double();
    auto track_width = get_parameter("track_width").as_double();
    motor_cmd_max_ = get_parameter("motor_cmd_max").as_int();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    collision_radius_ = get_parameter("collision_radius").as_double();
    max_path_size_ = get_parameter("max_path_size").as_int();
    auto input_noise = get_parameter("input_noise").as_double();
    auto slip_fraction = get_parameter("slip_fraction").as_double();
    basic_sensor_variance_ = get_parameter("basic_sensor_variance").as_double();
    max_range_ = get_parameter("max_range").as_double();
    noisy_diff_drive_ = std::make_unique<NoisyDiffDrive>(wheel_radius, track_width, input_noise, slip_fraction);

    // Initialize LiDAR simulator
    auto lidar_range_min = get_parameter("lidar_range_min").as_double();
    auto lidar_range_max = get_parameter("lidar_range_max").as_double();
    auto lidar_angle_increment = get_parameter("lidar_angle_increment").as_double();
    auto lidar_resolution = get_parameter("lidar_resolution").as_double();
    auto lidar_noise_variance = get_parameter("lidar_noise_variance").as_double();
    lidar_ = std::make_unique<turtlelib::Lidar>(lidar_range_min, lidar_range_max, lidar_angle_increment, lidar_resolution, lidar_noise_variance);

    gt_pose_ = get_pose0();
    sim_rate_ = get_parameter("rate").as_double();
    // Validate that x and y have same length
    auto obs_x = get_parameter("obstacles.x").as_double_array();
    auto obs_y = get_parameter("obstacles.y").as_double_array();
    auto obs_r = get_parameter("obstacles.r").as_double();
    if (obs_x.size() != obs_y.size()) {
      RCLCPP_ERROR(get_logger(), "obstacles.x and obstacles.y must have the same length");
      throw std::runtime_error("obstacles.x and obstacles.y must have the same length");
    }

    draw_only_ = get_parameter("draw_only").as_bool();
    if (draw_only_) {
      RCLCPP_INFO(get_logger(), "Draw only mode enabled: nusimulator will only publish the arena walls and obstacles; and will not simulate robot movement or publish sensor data.");
    }
    else {
      RCLCPP_INFO(get_logger(), "Simulation mode enabled: nusimulator will simulate robot movement and publish sensor data.");
      timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / sim_rate_), std::bind(&NUSimulator::timer_callback, this));
      count_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
      reset_service_ = create_service<std_srvs::srv::Empty>(
        "~/reset",
        std::bind(&NUSimulator::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
      wheel_cmd_sub_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
        "red/wheel_cmd", 10,
        std::bind(&NUSimulator::wheel_cmd_callback, this, std::placeholders::_1));

      // sensor data publisher - publishes CLEAN encoder data (ideal, no noise)
      // turtle_control reads this and publishes to blue/joint_states for odometry
      sensor_data_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);

      // joint states for red robot (ground truth with noise)
      joint_states_publisher_ = create_publisher<sensor_msgs::msg::JointState>("red/joint_states", 10);

      path_publisher_ = create_publisher<nav_msgs::msg::Path>("red/path", 10);

      // laser scan publisher
      laser_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("red/scan", 10);
      
      // fake sensor publisher at 5Hz
      fake_sensor_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/fake_sensor", rclcpp::QoS(10));
      
      fake_sensor_timer_ = create_wall_timer(
        200ms, std::bind(&NUSimulator::fake_sensor_callback, this));
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // publishers for landmarks
    // publish these with root namespace cuz we change the name of this node sometimes and we want landmarks to always be at the same topic

    // create arena walls publisher
    rclcpp::QoS qos(10);
    qos.transient_local();
    walls_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/real_walls", qos);

    // create obstacles publisher
    obstacles_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/real_obstacles", qos);

    publish_arena();
    gt_obs_ = std::make_unique<Obstacles>(Obstacles{obs_x, obs_y, obs_r});
    publish_cyl_obstacles(obs_x, obs_y, obs_r);
    
    // Initialize random number generator for sensor noise
    rand_gen_ = std::mt19937(std::random_device{}());
    
    RCLCPP_INFO(get_logger(), "nusimulator node constructed.");
  }

private:
  static constexpr auto rad_to_ticks(double radians, double ticks_per_rad) -> std::int64_t
  {
    return static_cast<std::int64_t>(radians * ticks_per_rad);
  }

  void timer_callback()
  {
    auto count_msg = std_msgs::msg::UInt64();
    count_msg.data = count_++;

    // update wheels + robot pose
    auto dt = 1.0 / sim_rate_;
    noisy_diff_drive_->noisy_fk(wheel_vel_left_, wheel_vel_right_, dt);
    auto [noisy_wheel_angle_left, noisy_wheel_angle_right] = noisy_diff_drive_->get_gt_wheel_angles();
    auto [encoder_wheel_angle_left, encoder_wheel_angle_right] = noisy_diff_drive_->get_encoder_wheel_angles();
    gt_pose_ = noisy_diff_drive_->get_gt_pose();

    // check for collisions
    auto new_gt_pose_ = gt_obs_->collide(gt_pose_, collision_radius_);
    noisy_diff_drive_->set_gt_pose(new_gt_pose_);
    if (gt_obs_->did_collide()) {
      auto msg = std::format("Collision ({}) Pose: {:D.3f} -> {:D.3f}.", count_msg.data, gt_pose_, new_gt_pose_);
      RCLCPP_DEBUG(get_logger(), msg.c_str());
    }
    gt_pose_ = new_gt_pose_;

    // publish encoder sensor data (noise only, no slip - what encoders actually read)
    // turtle_control will convert this to blue/joint_states for clean odometry
    auto sensor_msg = nuturtlebot_msgs::msg::SensorData();
    sensor_msg.left_encoder = rad_to_ticks(encoder_wheel_angle_left, encoder_ticks_per_rad_);
    sensor_msg.right_encoder = rad_to_ticks(encoder_wheel_angle_right, encoder_ticks_per_rad_);
    sensor_msg.stamp = get_clock()->now();

    // publish NOISY joint states for red robot (actual ground truth motion with noise + slip)
    auto joint_states = sensor_msgs::msg::JointState{};
    joint_states.header.stamp = get_clock()->now();
    joint_states.name = {"wheel_left_joint", "wheel_right_joint"};
    joint_states.position = {noisy_wheel_angle_left, noisy_wheel_angle_right};

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

    // send everything out
    tf_broadcaster_->sendTransform(transform);
    count_publisher_->publish(count_msg);
    sensor_data_publisher_->publish(sensor_msg);  // clean data -> turtle_control -> blue/joint_states -> odometry
    joint_states_publisher_->publish(joint_states);  // noisy data for red robot visualization

    // publish a path for the ground truth robot position with max 500 poses
    auto pose_stamped = geometry_msgs::msg::PoseStamped();
    pose_stamped.header.stamp = get_clock()->now();
    pose_stamped.header.frame_id = "nusim/world";
    pose_stamped.pose.position.x = gt_pose_.translation().x;
    pose_stamped.pose.position.y = gt_pose_.translation().y;
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.pose.orientation.x = quat[0];
    pose_stamped.pose.orientation.y = quat[1];
    pose_stamped.pose.orientation.z = quat[2];
    pose_stamped.pose.orientation.w = quat[3];

    gt_path_buffer_.push_back(pose_stamped);
    if (gt_path_buffer_.size() > max_path_size_) {
      gt_path_buffer_.pop_front();
    }

    auto path_msg = nav_msgs::msg::Path();
    path_msg.header.stamp = get_clock()->now();
    path_msg.header.frame_id = "nusim/world";
    path_msg.poses = std::vector<geometry_msgs::msg::PoseStamped>(
        gt_path_buffer_.begin(), gt_path_buffer_.end());
    path_publisher_->publish(path_msg);
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    count_ = 0;
    gt_pose_ = get_pose0();
    gt_path_buffer_.clear();
    noisy_diff_drive_ = std::make_unique<NoisyDiffDrive>(
      get_parameter("wheel_radius").as_double(),
      get_parameter("track_width").as_double(),
      get_parameter("input_noise").as_double(),
      get_parameter("slip_fraction").as_double());
    const auto msg = std::format(
      "Simulation reset: timestep set to 0, robot reset to initial pose "
      "({}).",
      gt_pose_);
    RCLCPP_INFO(get_logger(), msg.c_str());
  }

  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
  {
    /*
     Each received wheel_cmd command sets the wheel velocities of the robot until the next wheel_cmd command is received.
     The wheel_cmd messages are integer values between -265 and 265 and are proportional to the maximum rotational velocity of the motor (see Specifications and A.8).
     */
    wheel_vel_left_ = std::clamp(msg->left_velocity, -motor_cmd_max_,
      motor_cmd_max_) * motor_cmd_per_rad_sec_;
    wheel_vel_right_ = std::clamp(msg->right_velocity, -motor_cmd_max_,
      motor_cmd_max_) * motor_cmd_per_rad_sec_;
  }

  /// @brief get the initial pose of the robot from parameters
  /// @return tf for the initial pose (x0, y0, theta0)
  turtlelib::Transform2D get_pose0()
  {
    const auto x = get_parameter("x0").as_double();
    const auto y = get_parameter("y0").as_double();
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
    RCLCPP_INFO(get_logger(), "Published arena walls.");
  }

  /// @brief publish cylindrical obstacles as configured on startup.
  void publish_cyl_obstacles(const std::vector<double>& obs_x, const std::vector<double>& obs_y, const double obs_r)
  {
    const auto cyl_height = 0.25;

    auto marker_array = visualization_msgs::msg::MarkerArray();

    for (size_t i = 0; i < obs_x.size(); ++i) {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "nusim/world";
      marker.header.stamp = get_clock()->now();
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

      marker.ns = "red";
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      marker_array.markers.push_back(marker);
      RCLCPP_INFO(get_logger(), std::format("Added obstacle at ({}, {}) with radius {}", obs_x[i], obs_y[i], obs_r).c_str());
    }

    RCLCPP_INFO(get_logger(), "Published arena obstacles.");
    obstacles_publisher_->publish(marker_array);
  }
  
  /// @brief Callback for fake sensor that publishes obstacles relative to robot with noise
  void fake_sensor_callback() {
    const auto cyl_height = 0.25;
    auto marker_array = visualization_msgs::msg::MarkerArray();
    
    // Get transform from world to robot frame (inverse of robot pose in world)
    const auto world_to_robot = gt_pose_.inv();
    
    // Create Gaussian noise distribution
    std::normal_distribution<double> noise_dist(0.0, std::sqrt(basic_sensor_variance_));
    
    for (size_t i = 0; i < gt_obs_->x.size(); ++i) {
      // obstacle in world frame
      const auto obs_world = turtlelib::Point2D{gt_obs_->x[i], gt_obs_->y[i]};
      // obstacle in robot frame
      const auto obs_robot = world_to_robot(obs_world);
      
      // Add Gaussian noise to the measurements
      const auto noisy_x = obs_robot.x + noise_dist(rand_gen_);
      const auto noisy_y = obs_robot.y + noise_dist(rand_gen_);
      
      // Calculate distance from robot
      const auto distance = std::sqrt(noisy_x * noisy_x + noisy_y * noisy_y);
      
      // Create marker
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "red/base_footprint";
      marker.header.stamp = rclcpp::Time(0);
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.ns = "fake_sensor";
      
      // Check if within range
      if (distance > max_range_) {
        // Set action to DELETE if out of range
        marker.action = visualization_msgs::msg::Marker::DELETE;
      } else {
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = noisy_x;
        marker.pose.position.y = noisy_y;
        marker.pose.position.z = cyl_height / 2.0;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 2.0 * gt_obs_->r;  // diameter
        marker.scale.y = 2.0 * gt_obs_->r;  // diameter
        marker.scale.z = cyl_height;
        
        // Yellow color for sensor measurements
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
      }
      
      marker_array.markers.push_back(marker);
    }
    
    fake_sensor_publisher_->publish(marker_array);
    publish_lidar();
  }
  
  /// @brief Publish a sensor_messages/LaserScan displaying the simulated laser scan data (in red)
  void publish_lidar() {
    // Simulate a laserscan
    auto ranges = lidar_->simulate_scan(gt_pose_, *gt_obs_);

    auto scan_msg = sensor_msgs::msg::LaserScan();
    scan_msg.header.stamp = get_clock()->now();
    scan_msg.header.frame_id = "red/base_scan";
    scan_msg.angle_min = 0;
    scan_msg.angle_max = 2.0 * M_PI;
    scan_msg.angle_increment = lidar_->get_angle_increment();
    scan_msg.range_min = lidar_->get_min_range();
    scan_msg.range_max = lidar_->get_max_range();
    scan_msg.time_increment = 0.0;
    
    // Convert double ranges to float for LaserScan message
    scan_msg.ranges.reserve(ranges.size());
    for (const auto range : ranges) {
      scan_msg.ranges.push_back(static_cast<float>(range));
    }

    laser_scan_publisher_->publish(scan_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr fake_sensor_timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr count_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<Obstacles> gt_obs_;
  std::unique_ptr<turtlelib::Lidar> lidar_;

  bool draw_only_ = false;

  /// @brief simulation timestep
  size_t count_;
  double sim_rate_;
  /// @brief ground truth pose of the robot
  turtlelib::Transform2D gt_pose_;
  /// @brief ground truth path of the robot (deque with configurable max size)
  std::deque<geometry_msgs::msg::PoseStamped> gt_path_buffer_;
  size_t max_path_size_;
  double wheel_vel_left_ = 0.0;
  double wheel_vel_right_ = 0.0;

  // robot parameters
  int motor_cmd_max_{};
  double motor_cmd_per_rad_sec_{};
  double encoder_ticks_per_rad_{};
  double collision_radius_{};
  std::unique_ptr<NoisyDiffDrive> noisy_diff_drive_;
  
  // sensor parameters
  double basic_sensor_variance_{};
  double max_range_{};
  std::mt19937 rand_gen_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NUSimulator>());
  rclcpp::shutdown();
  return 0;
}
