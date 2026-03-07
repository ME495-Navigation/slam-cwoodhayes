/// @file
/// @brief uses EKF to perform SLAM for a diff-drive robot.

#ifndef DD_SLAM_HPP
#define DD_SLAM_HPP

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"
#include "turtlelib/geometry2d.hpp"
#include <unordered_map>

namespace turtlelib
{

/// @brief Process model for diff-drive EKF. Thin wrapper around diff-drive class
class DDSLAMProcessModel : public ProcessModel
{
public:
  DDSLAMProcessModel() {};

  arma::vec g(const arma::vec & state, const arma::vec & control) const override;
  arma::mat A(const arma::vec & state, const arma::vec & control) const override;
};

/// @brief Measurement model for diff-drive EKF. Measures range and bearing to known landmarks.
/// for now, we assume we have perfect data association and know which landmark is which
/// will fix this later.
class DDSLAMMeasurementModel : public MeasurementModel
{
public:
  DDSLAMMeasurementModel() {};

  arma::vec h(const arma::vec & state) const override;
  arma::mat H(const arma::vec & state) const override;
  // sneaky way to make sure we can only calculate the measurement
  // update for one landmark at a time; see DDSLAM below for usage.
  size_t observed_landmark_id = 0; 

};

class DDSLAM
{
public:
  DDSLAM(
    double wheel_radius, double wheel_track, arma::mat R, arma::mat Q,
    arma::vec initial_state, arma::mat initial_covariance)
  : diff_drive_(wheel_radius, wheel_track),
    process_model_(),
    measurement_model_(),
    ekf_(process_model_, measurement_model_, R, Q, initial_state, initial_covariance)
  {
  }

  /// @brief Perform EKF prediction step given control input (odometry)
  /// @param new_phi_left left wheel angular position
  /// @param new_phi_right right wheel angular position
  void odom_update(const double new_phi_left, const double new_phi_right);

  /// @brief Perform EKF update step given measurement (range and bearing to landmarks)
  /// @param landmark_id id of the observed landmark (key in the landmark_positions map)
  /// @param range measured range to the landmark
  /// @param bearing measured bearing to the landmark
  void measurement_update(const int landmark_id, const double range, const double bearing);

  /// @brief Get current state estimate (robot pose and landmark positions)
  arma::vec get_state() const { return ekf_.get_state(); }

  /// @brief Get current covariance estimate
  arma::mat get_covariance() const { return ekf_.get_covariance(); }

  /// @brief Get the current transform from the map frame to the odom frame, based on the current state estimate of the robot pose. This is useful for publishing a tf between map and odom that reflects the SLAM estimate of the robot's pose in the map.
  /// @return T_mo
  Transform2D get_map_to_odom() const;

  /// @brief Get the number of landmarks currently being estimated by the EKF. 
  size_t get_num_landmarks() const;

  /// @brief Get the current estimated positions of the landmarks.
  /// @return 2xN matrix, where N is the number of landmarks. each col is (x, y) position.
  arma::mat get_landmark_positions() const;

private:
  DiffDrive diff_drive_;
  DDSLAMProcessModel process_model_;
  DDSLAMMeasurementModel measurement_model_;
  EKF ekf_;
};

}  // namespace turtlelib

#endif  // DD_SLAM_HPP