/// @file
/// @brief uses EKF to perform SLAM for a diff-drive robot.
/// see https://nu-msr.github.io/navigation/lectures/slam/slam.pdf for math notes.

#include "turtlelib/dd_slam.hpp"


namespace turtlelib {

  arma::vec DDSLAMProcessModel::g(const arma::vec & state, const arma::vec & control) const
  {
    // state is [x, y, theta, landmark1_x, landmark1_y, landmark2_x, landmark2_y,...]
    // control is [dtheta, dx, dy=0] (dy is zero due to diff drive constraint)
    // we only update the robot pose based on the control input and leave the landmarks unchanged
    arma::vec next_state = state;
    turtlelib::Transform2D T_wb({state(0), state(1)}, state(2));
    turtlelib::Twist2D V_b = {
      .omega = control(0),
      .x = control(1),
      .y = 0.0
    };
    // theta_{t-1} for convenience.
    auto th_prev = T_wb.rotation();

    // 2 cases. 
    if (std::abs(V_b.omega) < 1e-6) {
      // zero rotational velocity
      arma::vec dq = {0, V_b.x * cos(th_prev), V_b.x * sin(th_prev};
      next_state.subvec(0, 2) += dq;
    }
    else {
      // nonzero rotational velocity
      auto dxdth = V_b.x / V_b.omega;
      arma::vec dq = {
        V_b.omega,
        -dxdth * sin(th_prev) + dxdth * sin(th_prev + V_b.omega),
        dxdth * cos(th_prev) - dxdth * cos(th_prev + V_b.omega)
      };
      next_state.subvec(0, 2) += dq;
    }

    // note - the EKF class adds the noise in my implementation;
    // BUT we must trust that it shouldn't add any process noise to the map locations 
    // in the state. we assume that those remain constantly located in our model (since we are
    // operating in map frame).

    return next_state;
  }

  arma::mat DDSLAMProcessModel::A(const arma::vec & state, const arma::vec & control) const
  {
    // Jacobian of process model with respect to state. See derivation in notes linked above.
    turtlelib::Transform2D T_wb({state(0), state(1)}, state(2));
    turtlelib::Twist2D V_b = {
      .omega = control(0),
      .x = control(1),
      .y = 0.0
    };
    // theta_{t-1} for convenience.
    auto th_prev = T_wb.rotation();
    // note - inefficient to allocate a new A (jacobian matrix) every time, (esp since mostly 0's)
    // but let's start here for clarity and optimize if we need to
    arma::mat A = arma::eye(state.size(), state.size());

    // 2 cases. 
    if (std::abs(V_b.omega) < 1e-6) {
      // zero rotational velocity
      arma::vec dcol = {
        0,
        -V_b.x * sin(th_prev),
        V_b.x * cos(th_prev)
      };

      A.submat(0, 2, arma::size(3, 1)) += dcol;
    }
    else {
      // nonzero rotational velocity
      auto dxdth = V_b.x / V_b.omega;
      arma::vec dcol = {
        0,
        -dxdth * cos(th_prev) + dxdth * cos(th_prev + V_b.omega),
        -dxdth * sin(th_prev) + dxdth * sin(th_prev + V_b.omega)
      };

      A.submat(0, 2, arma::size(3, 1)) += dcol;
    }

    return A;
  }

}