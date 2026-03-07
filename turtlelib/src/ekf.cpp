
# include "turtlelib/ekf.hpp"
# include "armadillo"

namespace turtlelib
{

  EKF::EKF(const ProcessModel & process_model, const MeasurementModel & measurement_model,
           const arma::mat & R, const arma::mat & Q, arma::vec initial_state, arma::mat initial_covariance)
  : process_model_(process_model), measurement_model_(measurement_model), R_(R), Q_(Q), 
    state_(initial_state), covariance_(initial_covariance)
  {
    // validate dimensions of R and Q
    if (R.n_rows != R.n_cols) {
      throw std::invalid_argument("Process noise covariance R must be square");
    }
    if (Q.n_rows != Q.n_cols) {
      throw std::invalid_argument("Measurement noise covariance Q must be square");
    }
    return;
  }

  void EKF::step(const arma::vec & control, const arma::vec & measurement)
  {
    // validate dimensions of control and measurement
    if (control.n_rows != Q_.n_rows && control.n_rows != 0) {
      throw std::invalid_argument("Control vector dimension must match process noise covariance Q (or be empty for no control)");
    }
    if (measurement.n_rows != R_.n_rows && measurement.n_rows != 0) {
      throw std::invalid_argument("Measurement vector dimension must match measurement noise covariance R (or be empty for no measurement)");
    }

    // generate noise samples for process and measurement noise
    // w=process noise (w_k in notes)
    arma::vec w = arma::vec(state_.n_rows, arma::fill::zeros);
    // v=measurement noise (v_k in notes)
    arma::vec v = arma::vec(R_.n_rows, arma::fill::zeros);
    
    // only generate noise if we're actually using those updates
    if (control.n_rows > 0) {
      for (size_t i = 0; i < state_.n_rows; ++i) {
        w(i) = noise_dist_(rng_) * std::sqrt(Q_(i, i));
      }
    }
    if (measurement.n_rows > 0) {
      for (size_t i = 0; i < R_.n_rows; ++i) {
        v(i) = noise_dist_(rng_) * std::sqrt(R_(i, i));
      }
    }

    arma::vec x_hat;
    arma::mat cov_hat;

    // state prediction step (only if control is provided)
    if (control.n_rows > 0) {
      x_hat = process_model_.g(state_, control) + w;
      arma::mat A = process_model_.A(state_, control);
      cov_hat = A * covariance_ * A.t() + Q_;
    } else {
      // no control: predicted state is current state
      x_hat = state_;
      cov_hat = covariance_;
    }

    // measurement update step (only if measurement is provided)
    if (measurement.n_rows > 0) {
      // see wikipedia EKF article for clear steps,
      // though I use matt's naming conventions for some of the variables
      arma::vec z_hat = measurement_model_.h(x_hat);
      arma::mat H = measurement_model_.H(x_hat);

      // innovation
      arma::vec y = (measurement + v) - z_hat;
      // innovation covariance
      arma::mat S = H * cov_hat * H.t() + R_;
      // kalman gain
      arma::mat K = cov_hat * H.t() * S.i();

      // final updates
      state_ = x_hat + K * y;
      covariance_ = (arma::eye(cov_hat.n_rows, cov_hat.n_cols) - K * H) * cov_hat;
    } else {
      // no measurement: propagate prediction as final estimate
      state_ = x_hat;
      covariance_ = cov_hat;
    }
  }

};