/// @file
/// @brief contains Extended Kalman Filter class for state estimation.
/// Uses Armadillo for matrix operations, and references these notes for notation & math:
/// https://nu-msr.github.io/navigation/lectures/kalman_filter.html
/// as well as wikipedia for clarity:
/// https://en.wikipedia.org/wiki/Extended_Kalman_filter

#ifndef EKF_HPP
#define EKF_HPP

#include "armadillo"
#include "random"

namespace turtlelib
{

  /// @brief Abstract class for measurement model of the EKF.
  class MeasurementModel
  {
  public:
    virtual ~MeasurementModel() = default;
    /// @brief Predicts the measurement we should see at the given state
    /// @param state state vector at time t
    /// @return measurement vector at time t
    virtual arma::vec h(const arma::vec & state) const = 0;
    /// @brief Jacobian of measurement model with respect to state.
    /// @param state state vector at time t
    /// @return Jacobian matrix of h with respect to state
    virtual arma::mat H(const arma::vec & state) const = 0;
  };


  /// @brief Abstract class for process model of the EKF.
  class ProcessModel
  {
  public:
    virtual ~ProcessModel() = default;
    /// @brief Predicts next state given current state and control input. (odometry)
    /// @param state state vector at t-1
    /// @param control control vector at t-1
    /// @return state vector at t
    virtual arma::vec g(const arma::vec & state, const arma::vec & control) const = 0;

    /// @brief Jacobian of process model with respect to state.
    /// @param state state vector at t-1
    /// @param control control vector at t-1
    /// @return Jacobian matrix of g with respect to state
    virtual arma::mat A(const arma::vec & state, const arma::vec & control) const = 0;
  };


  /// @brief Extended Kalman Filter for state estimation.
  class EKF
  {
  public:
    /// @brief constructor
    /// @param process_model process model of the system
    /// @param measurement_model measurement model of the system
    /// @param R gaussian measurement noise covariance
    /// @param Q gaussian process noise covariance
    EKF(const ProcessModel & process_model, const MeasurementModel & measurement_model,
        const arma::mat & R, const arma::mat & Q, arma::vec initial_state, arma::mat initial_covariance);

    /// @brief Performs EKF prediction and update steps given control input and measurement.
    /// @param control control vector u_t-1
    /// @param measurement measurement vector z_t
    /// @post updates internal state estimate and covariance
    void step(const arma::vec & control, const arma::vec & measurement);

    /// @brief Get current state estimate x_t
    arma::vec get_state() const { return state_; }

    /// @brief Get current covariance estimate
    arma::mat get_covariance() const { return covariance_; }

    /// @brief Set the filter state directly.
    /// @param state new state vector, must match current state dimension
    void set_state(const arma::vec & state);

    /// @brief Resize EKF state and covariances together.
    /// @param state new state vector
    /// @param covariance new covariance matrix (must be square and match state dimension)
    /// @param Q new process noise covariance (must be square and match state dimension)
    void resize_filter(const arma::vec & state, const arma::mat & covariance, const arma::mat & Q);

    // debugging vars. not used, just written to.
    // kalman gain from most recent update step
    arma::mat K_;
    arma::mat y_;

  private:
    const ProcessModel & process_model_;
    const MeasurementModel & measurement_model_;
    const arma::mat R_;
    arma::mat Q_;
    arma::vec state_;
    arma::mat covariance_;

  };

};

#endif  // EKF_HPP