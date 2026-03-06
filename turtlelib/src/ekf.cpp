
# include "turtlelib/ekf.hpp"
# include "armadillo"

namespace turtlelib
{

  EKF::EKF(const ProcessModel & process_model, const MeasurementModel & measurement_model,
           const arma::mat & R, const arma::mat & Q)
  : process_model_(process_model), measurement_model_(measurement_model), R_(R), Q_(Q)
  {
    // TODO
    return;
  }

  void EKF::step(const arma::vec & control, const arma::vec & measurement)
  {
    // TODO
    return;
  }

};