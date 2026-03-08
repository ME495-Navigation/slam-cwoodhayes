
# include "turtlelib/ekf.hpp"
# include "armadillo"

#include <format>

namespace turtlelib
{

  EKF::EKF(const ProcessModel & process_model, const MeasurementModel & measurement_model,
           const arma::mat & R, const arma::mat & Q, arma::vec initial_state, arma::mat initial_covariance)
  : process_model_(process_model), measurement_model_(measurement_model), R_(R), Q_(Q), 
    state_(initial_state), covariance_(initial_covariance)
  {
    // validate dimensions of R and Q
    if (R.n_rows != R.n_cols) {
      throw std::invalid_argument("Measurement noise covariance R must be square");
    }
    if (Q.n_rows != Q.n_cols) {
      throw std::invalid_argument("Process noise covariance Q must be square");
    }
    return;
  }

  void EKF::step(const arma::vec & control, const arma::vec & measurement)
  {
    // validate dimensions of Q and R match state and measurement
    if (Q_.n_rows != state_.n_rows) {
      throw std::invalid_argument("Process noise covariance Q must match state dimension");
    }
    if (measurement.n_rows != R_.n_rows && measurement.n_rows != 0) {
      throw std::invalid_argument(std::format(
        "Measurement vector dimension must match measurement noise covariance R (or be empty for no measurement): measurement.n_rows={}, R.n_rows={}",
        measurement.n_rows,
        R_.n_rows));
    }

    // predicted state est
    arma::vec x_hat;
    // predicted covariance est
    arma::mat P;

    // state prediction step (only if control is provided)
    if (control.n_rows > 0) {
      x_hat = process_model_.g(state_, control);
      arma::mat A = process_model_.A(state_, control);
      P = A * covariance_ * A.t() + Q_;
    } else {
      // no control: predicted state is current state
      x_hat = state_;
      P = covariance_;
    }

    // measurement update step (only if measurement is provided)
    if (measurement.n_rows > 0) {
      // see wikipedia EKF article for clear steps,
      // though I use matt's naming conventions for some of the variables
      arma::vec z_hat = measurement_model_.h(x_hat);
      arma::mat H = measurement_model_.H(x_hat);

      // innovation
      arma::vec y = measurement - z_hat;
      // innovation covariance
      arma::mat S = H * P * H.t() + R_;
      // kalman gain
      arma::mat K = P * H.t() * S.i();

      // final updates
      state_ = x_hat + K * y;
      covariance_ = (arma::eye(P.n_rows, P.n_cols) - K * H) * P;
    } else {
      // no measurement: propagate prediction as final estimate
      state_ = x_hat;
      covariance_ = P;
    }
  }

  void EKF::set_state(const arma::vec & state)
  {
    if (state.n_rows != state_.n_rows) {
      throw std::invalid_argument("New state dimension must match existing EKF state dimension");
    }
    state_ = state;
  }

};