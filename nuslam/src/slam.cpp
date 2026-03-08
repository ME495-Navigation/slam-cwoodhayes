/// \file
/// \brief contains SLAM node (based on odometry node in turtle_control).

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/angle.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/dd_slam.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtle_control/srv/set_pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "armadillo"

#include <functional>
#include <numeric>
#include <ranges>
#include <format>
#include <queue>

/// \brief Node for SLAM + odometry estimation of the robot.
///
/// Subscribes:
/// - joint_states (sensor_msgs/msg/JointState): Current wheel joint positions
/// - landmark observations topic (visualization_msgs/msg/MarkerArray): Landmark observations in robot frame
///
/// Publishes:
/// - odom (nav_msgs/msg/Odometry): Robot odometry pose and twist
/// - blue/path (nav_msgs/msg/Path): Robot path based on odometry measurements
/// - green/path (nav_msgs/msg/Path): Robot path based on SLAM pose estimates
/// - green/joint_states (sensor_msgs/msg/JointState): Wheel joint states for green robot visualization
///
/// Services:
/// - set_initial_pose (turtle_control/srv/SetPose): Sets the initial pose for odometry
///
/// Broadcasts:
/// - map -> odom transform via tf2 (for SLAM pose estimates)
/// - odom -> body_id transform via tf2 (ie odom -> blue base_footprint)
/// - odom -> slam_body_id transform via tf2 (ie odom -> green base_footprint)
class SLAMNode : public rclcpp::Node
{
public:
  /// @brief constructor
  SLAMNode()
  : Node("odometry")
  {
    auto qos = rclcpp::QoS(10);

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", qos, std::bind(&SLAMNode::joint_states_cb, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("blue/path", qos);
    slam_path_pub_ = create_publisher<nav_msgs::msg::Path>("green/path", qos);
    green_joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("green/joint_states", qos);

    initial_pose_srv_ = create_service<turtle_control::srv::SetPose>(
      "set_initial_pose",
      std::bind(&SLAMNode::set_initial_pose_cb, this, std::placeholders::_1,
      std::placeholders::_2));

    // fetch necessary robot parameters from diff_params.yaml
    /**
    body_id: The name of the body frame of the robot. Defaults to base_footprint.
    odom_id: The name of the odometry frame. Defaults to odom.
    wheel_left: The name of the left wheel joint. If not specified, log an error and exit.
    wheel_right: The name of the right wheel joint. If not specified, log an error and exit.
    **/
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Name of the body frame of the robot";
      declare_parameter("body_id", "base_footprint", desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Name of the body frame of the SLAM estimate";
      declare_parameter("slam_body_id", "base_footprint_slam", desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Name of the odometry frame";
      declare_parameter("odom_id", "odom", desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Name of the left wheel joint";
      declare_parameter("wheel_left", "", desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Name of the right wheel joint";
      declare_parameter("wheel_right", "", desc);
    }
    // we also need some params from diff_params.yaml
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
      desc.description = "Maximum number of poses in the odometry path";
      declare_parameter("max_path_size", 1000, desc);
    }
    {
      auto desc = rcl_interfaces::msg::ParameterDescriptor();
      desc.description = "Topic for landmark observations (MarkerArray), defaults to simulated sensor topic";
      declare_parameter("landmark_observations_topic", "/fake_sensor", desc);
    }
    body_id_ = get_parameter("body_id").as_string();
    odom_id_ = get_parameter("odom_id").as_string();
    map_id_ = "map"; 
    wheel_left_ = get_parameter("wheel_left").as_string();
    wheel_right_ = get_parameter("wheel_right").as_string();
    slam_body_id_ = get_parameter("slam_body_id").as_string();

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    max_path_size_ = get_parameter("max_path_size").as_int();
    landmark_observations_topic_ = get_parameter("landmark_observations_topic").as_string();

    landmark_observations_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      landmark_observations_topic_, qos,
      std::bind(&SLAMNode::landmark_observations_cb, this, std::placeholders::_1));

    if (wheel_left_.empty() || wheel_right_.empty()) {
      RCLCPP_ERROR(get_logger(), "wheel_left and wheel_right parameters must be specified");
      throw std::runtime_error("Missing required wheel parameters");
    }

    // construct DiffDrive object with parameters
    diff_drive_ = std::make_unique<turtlelib::DiffDrive>(wheel_radius_, track_width_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // SLAM setup
    // TODO could make these params instead of hardcoding them. for now easier this way.
    arma::mat R = arma::eye(2, 2) * 0.05; // measurement noise covariance (range, bearing)
    
    // TODO make this dynamic later.
    auto n_max_landmarks = 5;
    auto state_dim = 3 + 2 * n_max_landmarks;
    
    // Process noise: only robot pose has noise (landmarks are static in the map)
    arma::mat Q = arma::zeros(state_dim, state_dim);
    Q.submat(0, 0, 2, 2) = arma::eye(3, 3) * 0.01; // process noise for robot pose only
    
    arma::vec initial_state = arma::zeros(state_dim); // initial state: robot pose (x, y, theta) + 5 landmarks
    arma::mat initial_covariance = arma::eye(state_dim, state_dim) * 1000; // high initial uncertainty
    dd_slam_ = std::make_unique<turtlelib::DDSLAM>(
      wheel_radius_, track_width_, R, Q, initial_state, initial_covariance
    );

    // publish initial transforms at the origin so that we have valid tf as soon as possible
    auto identity = turtlelib::Transform2D();
    publish_pose_tf(identity, map_id_, odom_id_);
    publish_pose_tf(identity, map_id_, body_id_);
    publish_pose_tf(identity, odom_id_, slam_body_id_);

    RCLCPP_INFO(get_logger(), "odometry node constructed.");
  }

private:
  /// @brief Callback function for joint_states topic. Computes and publishes tbot3 odometry.
  void joint_states_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // grab new wheel states from the msg by their names
    auto left_it = std::ranges::find(msg->name, wheel_left_);
    auto right_it = std::ranges::find(msg->name, wheel_right_);
    if (left_it == msg->name.end() || right_it == msg->name.end()) {
      // put all the names in js message in the error for easier debugging
      auto names_str = std::accumulate(
          msg->name.begin(), msg->name.end(), std::string{},
        [](const std::string & acc, const std::string & name) {
          return acc.empty() ? name : acc + ", " + name;
        });
      auto errmsg =
        std::format("Wheel joint names '{}' and '{}' not found in joint_states message: {}",
        wheel_left_, wheel_right_, names_str);
      RCLCPP_ERROR(get_logger(), errmsg.c_str());
      return;
    }
    auto left_idx = std::distance(msg->name.begin(), left_it);
    auto right_idx = std::distance(msg->name.begin(), right_it);
    auto left_pos = msg->position[left_idx];
    auto right_pos = msg->position[right_idx];

    // calculate odometry with FK using diff_drive.
    // returns transform odom -> body
    auto T_ob = diff_drive_->forward_kinematics(left_pos, right_pos);
    auto V_b = diff_drive_->get_body_twist();

    auto odom_msg = nav_msgs::msg::Odometry();
    // header specifies timestamp + parent frame (odom)
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = odom_id_;

    // child frame is the body frame of the robot (base_footprint)
    odom_msg.child_frame_id = body_id_;

    // fill in the pose.
    odom_msg.pose.pose.position.x = T_ob.translation().x;
    odom_msg.pose.pose.position.y = T_ob.translation().y;
    odom_msg.pose.pose.position.z = 0.0;
    const auto quat = turtlelib::angle_to_2d_planar_quaternion(T_ob.rotation());
    odom_msg.pose.pose.orientation.x = quat[0];
    odom_msg.pose.pose.orientation.y = quat[1];
    odom_msg.pose.pose.orientation.z = quat[2];
    odom_msg.pose.pose.orientation.w = quat[3];

    // fill in the twist in the body frame.
    odom_msg.twist.twist.angular.z = V_b.omega;
    odom_msg.twist.twist.linear.x = V_b.x;
    odom_msg.twist.twist.linear.y = V_b.y;
    odom_msg.twist.twist.linear.z = 0.0;

    // leave covariance as default (all 0s)

    // publish odometry msg and tf
    odom_pub_->publish(odom_msg);

    // Blue path shows pure odometry in map frame
    publish_path(
      msg->header.stamp,
      map_id_,
      odom_msg.pose.pose,
      path_buffer_,
      path_pub_);

    // update SLAM estimates
    // TODO make this concurrency safe by wrapping in a lock.
    // for now ok because we're using a single threaded executor
    dd_slam_->odom_update(left_pos, right_pos);
    auto T_mb = dd_slam_->get_map_to_body();
    // we need T_mo. derive this from T_mb and T_ob
    auto T_mo = T_mb * T_ob.inv();
    
    // Publish TF tree:
    // - map -> odom: SLAM correction
    // - map -> blue/base_footprint: pure odometry (stays fixed even as odom frame is corrected)
    // - odom -> green/base_footprint: SLAM estimate in odom frame
    publish_pose_tf(T_mo, map_id_, odom_id_);
    publish_pose_tf(T_ob, map_id_, body_id_);
    
    // Compute SLAM estimate in odom frame for green robot
    auto T_og = T_mo.inv() * T_mb;
    publish_pose_tf(T_og, odom_id_, slam_body_id_);

    // joint states can come from diff_drive because we don't need slam for encoder vals.
    publish_joint_states(
      msg->header.stamp,
      diff_drive_->get_wheel_angles(),
      diff_drive_->get_wheel_velocities(),
      green_joint_states_pub_);

    // Green path should show SLAM estimate (odom -> green/base_footprint)
    auto slam_pose = geometry_msgs::msg::Pose();
    slam_pose.position.x = T_og.translation().x;
    slam_pose.position.y = T_og.translation().y;
    slam_pose.position.z = 0.0;
    auto slam_quat = turtlelib::angle_to_2d_planar_quaternion(T_og.rotation());
    slam_pose.orientation.x = slam_quat[0];
    slam_pose.orientation.y = slam_quat[1];
    slam_pose.orientation.z = slam_quat[2];
    slam_pose.orientation.w = slam_quat[3];

    publish_path(
      msg->header.stamp,
      odom_id_,
      slam_pose,
      slam_path_buffer_,
      slam_path_pub_);
  }

  void publish_path(
    const builtin_interfaces::msg::Time & stamp,
    const std::string & frame_id,
    const geometry_msgs::msg::Pose & pose,
    std::deque<geometry_msgs::msg::PoseStamped> & path_buffer,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & path_publisher)
  {
    auto pose_stamped = geometry_msgs::msg::PoseStamped();
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.pose = pose;

    path_buffer.push_back(pose_stamped);
    if (path_buffer.size() > max_path_size_) {
      path_buffer.pop_front();
    }

    auto path_msg = nav_msgs::msg::Path();
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = frame_id;
    path_msg.poses = std::vector<geometry_msgs::msg::PoseStamped>(
      path_buffer.begin(), path_buffer.end());
    path_publisher->publish(path_msg);
  }

  void publish_joint_states(
    const builtin_interfaces::msg::Time & stamp,
    const std::vector<double> & wheel_angles,
    const std::vector<double> & wheel_velocities,
    const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr & joint_states_publisher)
  {
    auto joint_states = sensor_msgs::msg::JointState{};
    joint_states.header.stamp = stamp;
    joint_states.name = {"wheel_left_joint", "wheel_right_joint"};
    joint_states.position = wheel_angles;
    joint_states.velocity = wheel_velocities;
    joint_states_publisher->publish(joint_states);
  }

  void publish_pose_tf(const turtlelib::Transform2D & T, const std::string & parent_frame, const std::string & child_frame)
  {
    const auto quat = turtlelib::angle_to_2d_planar_quaternion(T.rotation());
    auto tf = geometry_msgs::msg::TransformStamped();
    tf.header.stamp = get_clock()->now();
    tf.header.frame_id = parent_frame;
    tf.child_frame_id = child_frame;

    tf.transform.translation.x = T.translation().x;
    tf.transform.translation.y = T.translation().y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = quat[0];
    tf.transform.rotation.y = quat[1];
    tf.transform.rotation.z = quat[2];
    tf.transform.rotation.w = quat[3];
    tf_broadcaster_->sendTransform(tf);
  }

  void landmark_observations_cb(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    // go through the landmarks we got and feed them into SLAM
    for (const auto & marker : msg->markers) {
      if (marker.action != visualization_msgs::msg::Marker::ADD) {
        continue;
      }

      auto landmark_id = marker.id;

      // calculate range and bearing to the landmark from the marker position
      auto range = std::hypot(marker.pose.position.x, marker.pose.position.y);
      auto bearing = std::atan2(marker.pose.position.y, marker.pose.position.x);

      dd_slam_->measurement_update(landmark_id, range, bearing);
    }
  }

  /// @brief Callback function for set_initial_pose service. Sets the initial pose of the robot for odometry.
  void set_initial_pose_cb(
    const std::shared_ptr<turtle_control::srv::SetPose::Request> request,
    std::shared_ptr<turtle_control::srv::SetPose::Response> response)
  {
    // set the initial pose of the robot in the diff_drive object
    auto infomsg = std::format("Received request to set initial pose to x: {}, y: {}, theta: {}",
                    request->x, request->y, request->theta);
    RCLCPP_INFO(get_logger(), infomsg.c_str());

    turtlelib::Transform2D new_pose({request->x, request->y}, request->theta);
    diff_drive_->reset_to_configuration(new_pose);
    
    // Publish initial transforms for blue robot (odometry in map frame)
    publish_pose_tf(new_pose, map_id_, body_id_);
    path_buffer_.clear();
    
    // Also reset SLAM: identity map -> odom, and green robot at origin in odom frame
    publish_pose_tf(turtlelib::Transform2D(), map_id_, odom_id_);
    publish_pose_tf(new_pose, odom_id_, slam_body_id_);
    slam_path_buffer_.clear();
    
    response->success = true;
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_observations_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr slam_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr green_joint_states_pub_;
  rclcpp::Service<turtle_control::srv::SetPose>::SharedPtr initial_pose_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<turtlelib::DiffDrive> diff_drive_;
  std::unique_ptr<turtlelib::DDSLAM> dd_slam_;
  std::deque<geometry_msgs::msg::PoseStamped> path_buffer_;
  std::deque<geometry_msgs::msg::PoseStamped> slam_path_buffer_;
  size_t max_path_size_;
  std::string body_id_;
  std::string odom_id_;
  std::string slam_body_id_;
  std::string map_id_;
  std::string landmark_observations_topic_;
  std::string wheel_left_;
  std::string wheel_right_;
  double wheel_radius_;
  double track_width_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SLAMNode>());
  rclcpp::shutdown();
  return 0;
}
