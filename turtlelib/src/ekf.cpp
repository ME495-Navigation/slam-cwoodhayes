
# include "turtlelib/ekf.hpp"
# include "armadillo"

namespace turtlelib
{

  EKF::EKF(const ProcessModel & process_model, const MeasurementModel & measurement_model,
           const arma::mat & R, const arma::mat & Q)
  : process_model_(process_model), measurement_model_(measurement_model), R_(R), Q_(Q)
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
    if (control.n_rows != Q_.n_rows) {
      throw std::invalid_argument("Control vector dimension must match process noise covariance Q");
    }
    if (measurement.n_rows != R_.n_rows) {
      throw std::invalid_argument("Measurement vector dimension must match measurement noise covariance R");
    }

    // generate noise samples for process and measurement noise
    // w=process noise (w_k in notes)
    arma::vec w = arma::vec(R_.n_rows);
    // v=measurement noise (v_k in notes)
    arma::vec v = arma::vec(Q_.n_rows);
    for (size_t i = 0; i < R_.n_rows; ++i) {
      w(i) = noise_dist_(rng_) * std::sqrt(R_(i, i));
    }
    for (size_t i = 0; i < Q_.n_rows; ++i) {
      v(i) = noise_dist_(rng_) * std::sqrt(Q_(i, i));
    }

    // state prediction step
    arma::vec x_hat = process_model_.g(state_, control);
    arma::mat A = process_model_.A(state_, control);
    arma::mat cov_hat = A * covariance_ * A.t() + Q_;

    // measurement update step (see wikipedia EKF article for clear steps,
    // though I use matt's naming conventions for some of the variables)
    arma::vec z_hat = measurement_model_.h(x_hat);
    arma::mat H = measurement_model_.H(x_hat);

    // innovation
    arma::vec y = measurement - z_hat;
    // innovation covariance
    arma::mat S = H * cov_hat * H.t() + R_;
    // kalman gain
    arma::mat K = cov_hat * H.t() * S.i();

    // final updates
    state_ = x_hat + K * y;
    covariance_ = (arma::eye(cov_hat.n_rows, cov_hat.n_cols) - K * H) * cov_hat;
  }

};