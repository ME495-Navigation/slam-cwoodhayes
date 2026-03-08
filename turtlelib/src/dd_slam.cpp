/// @file
/// @brief uses EKF to perform SLAM for a diff-drive robot.
/// see https://nu-msr.github.io/navigation/lectures/slam/slam.pdf for math notes.

#include "turtlelib/dd_slam.hpp"


namespace turtlelib {

  DDSLAM::DDSLAM(
    double wheel_radius, double wheel_track, arma::mat R, arma::mat Q_robot_pose,
    arma::vec initial_state, arma::mat initial_covariance)
  : diff_drive_(wheel_radius, wheel_track),
    process_model_(),
    measurement_model_(),
    ekf_(
      process_model_,
      measurement_model_,
      R,
      expand_process_noise(Q_robot_pose, initial_state.n_rows),
      initial_state,
      initial_covariance)
  {
  }

  arma::mat DDSLAM::expand_process_noise(const arma::mat & Q_robot_pose, size_t state_dim)
  {
    if (Q_robot_pose.n_rows != 3 || Q_robot_pose.n_cols != 3) {
      throw std::runtime_error("Q_robot_pose must be a 3x3 matrix");
    }
    if (state_dim < 3) {
      throw std::runtime_error("state_dim must be at least 3 for [theta, x, y]");
    }

    auto Q_full = arma::mat(state_dim, state_dim, arma::fill::zeros);
    Q_full.submat(0, 0, 2, 2) = Q_robot_pose;
    return Q_full;
  }

  arma::vec DDSLAMProcessModel::g(const arma::vec & state, const arma::vec & control) const
  {
    // state is [theta, x, y, landmark1_x, landmark1_y, landmark2_x, landmark2_y,...]
    // control is [dtheta, dx, dy=0] (dy is zero due to diff drive constraint)
    // we only update the robot pose based on the control input and leave the landmarks unchanged
    arma::vec next_state = state;
    turtlelib::Transform2D T_wb({state(1), state(2)}, state(0));
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
      arma::vec dq = {0, V_b.x * cos(th_prev), V_b.x * sin(th_prev)};
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
    turtlelib::Transform2D T_wb({state(1), state(2)}, state(0));
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

  arma::vec DDSLAMMeasurementModel::h(const arma::vec & state) const
  {
    // measurement model. We can actually only calculate the EKF for one landmark at a time,
    // so we will select only that landmark to operate on here:
    // (and we get to return only a 2x1 vector for h)
    Point2D m = {
      .x = state(observed_landmark_id * 2 + 3), 
      .y = state(observed_landmark_id * 2 + 4)
    };
    auto x = state(1);
    auto y = state(2);
    auto th = state(0);

    arma::vec h = {
      sqrt(pow(m.x - x, 2) + pow(m.y - y, 2)), 
      atan2(m.y - y, m.x - x) - th
    };

    return h;
  }

  arma::mat DDSLAMMeasurementModel::H(const arma::vec & state) const
  {
    // Jacobian of measurement model with respect to state. See derivation in notes linked above.
    Point2D m = {
      .x = state(observed_landmark_id * 2 + 3), 
      .y = state(observed_landmark_id * 2 + 4)
    };
    auto x = state(1);
    auto y = state(2);

    double d = pow(m.x - x, 2) + pow(m.y - y, 2);
    arma::mat H = arma::zeros(2, state.size());
    H(0, 0) = 0;
    H(0, 1) = -(m.x - x) / sqrt(d);
    H(0, 2) = -(m.y - y) / sqrt(d);
    H(0, observed_landmark_id * 2 + 3) = (m.x - x) / sqrt(d);
    H(0, observed_landmark_id * 2 + 4) = (m.y - y) / sqrt(d);

    H(1, 0) = -1;
    H(1, 1) = (m.y - y) / d;
    H(1, 2) = -(m.x - x) / d;
    H(1, observed_landmark_id * 2 + 3) = -(m.y - y) / d;
    H(1, observed_landmark_id * 2 + 4) = (m.x - x) / d;

    return H;
  }


  void DDSLAM::odom_update(const double new_phi_left, const double new_phi_right)
  {
    // calculate control input (odometry) from wheel angles and perform EKF prediction step
    diff_drive_.forward_kinematics(new_phi_left, new_phi_right);
    auto V_b = diff_drive_.get_body_twist();
    auto control = arma::vec({V_b.omega, V_b.x, V_b.y});
    ekf_.step(control, arma::vec()); // empty measurement since we only want to do the prediction step
  }

  void DDSLAM::measurement_update(size_t landmark_id, const double range, const double bearing)
  {
    if (landmark_id >= get_num_landmarks()) {
      // TODO dynamically increase size of landmarks list instead
      throw std::runtime_error("Error: landmark_id exceeds number of landmarks currently in state");
    }

    // set the observed landmark id in the measurement model so that we calculate the measurement update for the correct landmark
    measurement_model_.observed_landmark_id = landmark_id;
    auto measurement = arma::vec({range, bearing});
    ekf_.step(arma::vec(), measurement); // empty control since we only want to do the measurement update step
  }

  Transform2D DDSLAM::get_map_to_body() const
  {
    auto state = ekf_.get_state();
    auto x = state(1);
    auto y = state(2);
    auto th = state(0);
    return Transform2D({x, y}, th);
  }

  size_t DDSLAM::get_num_landmarks() const
  {
    auto state = ekf_.get_state();
    return (state.n_rows - 3) / 2; // subtract 3 for robot pose, divide by 2 for x and y of each landmark
  }

  arma::mat DDSLAM::get_landmark_positions() const
  {
    auto state = ekf_.get_state();
    arma::mat landmarks(2, get_num_landmarks());
    for (size_t i = 0; i < landmarks.n_cols; ++i) {
      landmarks(0, i) = state(i * 2 + 3); // x position of landmark i
      landmarks(1, i) = state(i * 2 + 4); // y position of landmark i
    }
    return landmarks;
  }

}