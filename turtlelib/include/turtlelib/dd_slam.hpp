/// @file
/// @brief uses EKF to perform SLAM for a diff-drive robot.

#ifndef DD_SLAM_HPP
#define DD_SLAM_HPP

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"
#include "turtlelib/geometry2d.hpp"
#include <limits>
#include <unordered_map>
#include <vector>

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
  /// @brief Construct DDSLAM object
  /// @param wheel_radius Diff drive wheel radius
  /// @param wheel_track Diff drive wheel track
  /// @param R Measurement noise covariance
  /// @param Q_robot_pose Process noise covariance for robot pose
  /// @param initial_state Initial state vector
  /// @param initial_covariance Initial state covariance matrix
  /// @param max_landmarks Max number of landmarks allowed in state (for memory safety)
  DDSLAM(
    double wheel_radius, double wheel_track, arma::mat R, arma::mat Q_robot_pose,
    arma::vec initial_state, arma::mat initial_covariance,
    size_t max_landmarks,
    double new_landmark_variance = 1000.0
  );

  /// @brief Perform EKF prediction step given control input (odometry)
  /// @param new_phi_left left wheel angular position
  /// @param new_phi_right right wheel angular position
  void odom_update(const double new_phi_left, const double new_phi_right);

  /// @brief Perform EKF update step given measurement (range and bearing to landmarks)
  /// @param landmark_id id of the observed landmark (key in the landmark_positions map)
  /// @param range measured range to the landmark
  /// @param bearing measured bearing to the landmark
  /// @return the id of the landmark that was updated. Same as input landmark id.
  ///         (this changes in subclasses with unknown data association)
  virtual size_t measurement_update(size_t landmark_id, const double range, const double bearing);

  /// @brief Get current state estimate (robot pose and landmark positions)
  arma::vec get_state() const { return ekf_.get_state(); }

  /// @brief Get current covariance estimate
  arma::mat get_covariance() const { return ekf_.get_covariance(); }

  arma::mat get_K() const { return ekf_.K_; }
  arma::vec get_innovation() const { return ekf_.y_; }

  /// @brief Get the current transform from the map frame to the body frame.
  /// @return T_mb
  Transform2D get_map_to_body() const;

  /// @brief Get the number of landmarks currently being estimated by the EKF. 
  size_t get_num_landmarks() const;

  /// @brief Get the current estimated positions of the landmarks.
  /// @return 2xN matrix, where N is the number of landmarks. each col is (x, y) position.
  arma::mat get_landmark_positions() const;

  /// @brief Get landmark ids ordered by internal landmark slot.
  /// @return vector of landmark ids where element i corresponds to state entries (2*i+3, 2*i+4)
  std::vector<size_t> get_landmark_ids() const;

  virtual ~DDSLAM() = default;

protected:
  /// @brief Add a new landmark to the EKF state vector and covariance matrix,
  /// initialized based on the given measurement and current robot pose estimate.
  void add_landmark_to_state(size_t landmark_id, double range, double bearing);

  /// @brief Helper function to expand the process noise covariance matrix for new landmarks.
  static arma::mat resize_process_noise(const arma::mat & Q_robot_pose, arma::uword state_dim);

  DiffDrive diff_drive_;
  DDSLAMProcessModel process_model_;
  DDSLAMMeasurementModel measurement_model_;
  EKF ekf_;
  arma::mat Q_robot_pose_;
  size_t max_landmarks_;
  std::unordered_map<size_t, size_t> landmark_id_to_slot_;
  std::vector<size_t> slot_to_landmark_id_;
  double new_landmark_variance_;
};

/// @brief SLAM with unknown data association using Mahalanobis distance
class DDSLAMMahalanobis : public DDSLAM
{
public:
  /// @brief Construct with same params as DDSLAM, plus association threshold.
  /// @param wheel_radius Diff drive wheel radius
  /// @param wheel_track Diff drive wheel track
  /// @param R Measurement noise covariance
  /// @param Q_robot_pose Process noise covariance for robot pose
  /// @param initial_state Initial state vector
  /// @param initial_covariance Initial state covariance matrix
  /// @param max_landmarks Max number of landmarks allowed in state (for memory safety)
  /// @param new_landmark_variance Initial variance to assign to new landmarks when added.
  /// @param association_threshold Chi-square threshold for Mahalanobis distance 
  ///        Data association (default 5.991 for 95% confidence with 2 DOF)
  /// @param provisional_observation_count Number of consecutive observations below the association threshold
  ///        required before confirming a landmark association (to reduce false positives)
  DDSLAMMahalanobis(
    double wheel_radius, double wheel_track, arma::mat R, arma::mat Q_robot_pose,
    arma::vec initial_state, arma::mat initial_covariance,
    size_t max_landmarks,
    double new_landmark_variance = 1000.0,
    double association_threshold = 5.991,
    size_t provisional_observation_count = 3
  ) : 
  DDSLAM(wheel_radius, wheel_track, R, Q_robot_pose, initial_state, initial_covariance, max_landmarks, new_landmark_variance),
  association_threshold_(association_threshold),
  provisional_observation_count_(provisional_observation_count)
  {}

  /// @brief Perform EKF update step given measurement (range and bearing to landmarks) with unknown data association.
  /// @param landmark_id Ignored (just included for compatibility with DDSLAM interface).
  /// @param range landmark range measurement
  /// @param bearing landmark bearing measurement
  /// @return the id of the landmark that was updated, or newly added if no valid association was found.
  size_t measurement_update(size_t landmark_id, const double range, const double bearing) override;

private:
  size_t add_landmark_to_state_mahalanobis(double range, double bearing);
  size_t pop_landmark_from_state();
  std::unordered_map<size_t, size_t> landmark_id_to_count_;

  double association_threshold_;
  size_t provisional_observation_count_;

};

}  // namespace turtlelib

#endif  // DD_SLAM_HPP