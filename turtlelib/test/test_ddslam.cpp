#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <numbers>

#include "turtlelib/dd_slam.hpp"

using namespace turtlelib;
using namespace Catch::Matchers;
using std::numbers::pi;

TEST_CASE("DDSLAM basic construction and initialization", "[DDSLAM]")
{
    const auto wheel_radius = 0.1;
    const auto wheel_track = 0.5;
    const auto state_dim = 3;  // [theta, x, y]
    
    // Create simple covariance matrices
    auto R = arma::eye(2, 2);  // measurement noise (2D: range, bearing)
    auto Q = arma::eye(state_dim, state_dim);  // process noise
    auto initial_state = arma::vec(state_dim, arma::fill::zeros);
    auto initial_cov = arma::eye(state_dim, state_dim);
    
    // Construct DDSLAM
    auto slam = DDSLAM(wheel_radius, wheel_track, R, Q, initial_state, initial_cov);
    
    // Check initial state
    auto state = slam.get_state();
    REQUIRE(state.n_rows == state_dim);
    REQUIRE_THAT(state(0), WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(state(1), WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(state(2), WithinAbs(0.0, 1e-6));
    
    // Check initial covariance
    auto cov = slam.get_covariance();
    REQUIRE(cov.n_rows == state_dim);
    REQUIRE(cov.n_cols == state_dim);
}

TEST_CASE("DDSLAM prediction step (odom_update)", "[DDSLAM]")
{
    const auto wheel_radius = 0.1;
    const auto wheel_track = 0.5;
    const auto state_dim = 3;
    
    auto R = arma::eye(2, 2);
    auto Q = arma::eye(state_dim, state_dim) * 0.01;
    auto initial_state = arma::vec(state_dim, arma::fill::zeros);
    auto initial_cov = arma::eye(state_dim, state_dim) * 0.1;
    
    auto slam = DDSLAM(wheel_radius, wheel_track, R, Q, initial_state, initial_cov);
    
    // Initialize the differential drive with first call (no motion yet)
    slam.odom_update(0.0, 0.0);
    
    // Perform a prediction step: move wheels forward
    // If both wheels rotate by 2*pi, robot should move forward
    slam.odom_update(2.0 * pi, 2.0 * pi);
    
    auto state_after = slam.get_state();
    auto cov_after = slam.get_covariance();
    
    // Check that state and covariance have correct dimensions
    REQUIRE(state_after.n_rows == state_dim);
    REQUIRE(cov_after.n_rows == state_dim);
    REQUIRE(cov_after.n_cols == state_dim);
    
    // Check that robot moved (x should increase)
    REQUIRE(state_after(1) > 0.0);
}

TEST_CASE("DDSLAM measurement update", "[DDSLAM]")
{
    const auto wheel_radius = 0.1;
    const auto wheel_track = 0.5;
    const auto state_dim = 5;  // [theta, x, y, landmark1_x, landmark1_y]
    
    auto R = arma::eye(2, 2);
    auto Q = arma::eye(state_dim, state_dim) * 0.01;
    auto initial_state = arma::vec(state_dim, arma::fill::zeros);
    initial_state(3) = 1.0;  // landmark at (1, 0)
    initial_state(4) = 0.0;
    auto initial_cov = arma::eye(state_dim, state_dim) * 0.1;
    
    auto slam = DDSLAM(wheel_radius, wheel_track, R, Q, initial_state, initial_cov);
    
    // Perform a measurement update: observe landmark 0 at range 1.0, bearing 0.0
    slam.measurement_update(0, 1.0, 0.0);
    
    auto state_after = slam.get_state();
    auto cov_after = slam.get_covariance();
    
    // Check dimensions
    REQUIRE(state_after.n_rows == state_dim);
    REQUIRE(cov_after.n_rows == state_dim);
    REQUIRE(cov_after.n_cols == state_dim);
}

TEST_CASE("DDSLAM mixed prediction and measurement updates", "[DDSLAM]")
{
    const auto wheel_radius = 0.1;
    const auto wheel_track = 0.5;
    const auto state_dim = 3;
    
    auto R = arma::eye(2, 2) * 0.01;
    auto Q = arma::eye(state_dim, state_dim) * 0.01;
    auto initial_state = arma::vec(state_dim, arma::fill::zeros);
    auto initial_cov = arma::eye(state_dim, state_dim) * 0.1;
    
    auto slam = DDSLAM(wheel_radius, wheel_track, R, Q, initial_state, initial_cov);
    
    // Initialize differential drive
    slam.odom_update(0.0, 0.0);
    
    // Perform multiple forward motion steps (monotonically increasing phi values)
    slam.odom_update(pi, pi);  // move forward
    slam.odom_update(2.0 * pi, 2.0 * pi);  // move forward more
    slam.odom_update(3.5 * pi, 3.5 * pi);  // move forward again
    
    auto final_state = slam.get_state();
    auto final_cov = slam.get_covariance();
    
    // Check dimensions are preserved
    REQUIRE(final_state.n_rows == state_dim);
    REQUIRE(final_cov.n_rows == state_dim);
    REQUIRE(final_cov.n_cols == state_dim);
    
    // Check that robot moved forward significantly
    REQUIRE(final_state(1) > 1.0);
}

TEST_CASE("DDSLAM get_map_to_odom returns correct transform", "[DDSLAM]")
{
    const auto wheel_radius = 0.1;
    const auto wheel_track = 0.5;
    const auto state_dim = 3;  // [theta, x, y]
    
    auto R = arma::eye(2, 2);
    auto Q = arma::eye(state_dim, state_dim) * 0.01;
    auto initial_state = arma::vec(state_dim);
    initial_state(0) = pi / 4.0;  // theta = 45 degrees
    initial_state(1) = 2.0;       // x = 2.0
    initial_state(2) = 3.0;       // y = 3.0
    auto initial_cov = arma::eye(state_dim, state_dim) * 0.1;
    
    auto slam = DDSLAM(wheel_radius, wheel_track, R, Q, initial_state, initial_cov);
    
    auto T_mo = slam.get_map_to_odom();
    
    // Check that the transform matches the state
    REQUIRE_THAT(T_mo.rotation(), WithinAbs(pi / 4.0, 1e-6));
    REQUIRE_THAT(T_mo.translation().x, WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(T_mo.translation().y, WithinAbs(3.0, 1e-6));
}

TEST_CASE("DDSLAM get_num_landmarks returns correct count", "[DDSLAM]")
{
    const auto wheel_radius = 0.1;
    const auto wheel_track = 0.5;
    
    auto R = arma::eye(2, 2);
    auto Q_no_landmarks = arma::eye(3, 3) * 0.01;
    auto Q_two_landmarks = arma::eye(7, 7) * 0.01;
    auto Q_three_landmarks = arma::eye(9, 9) * 0.01;
    
    SECTION("No landmarks") {
        auto initial_state = arma::vec(3, arma::fill::zeros);
        auto initial_cov = arma::eye(3, 3) * 0.1;
        auto slam = DDSLAM(wheel_radius, wheel_track, R, Q_no_landmarks, initial_state, initial_cov);
        
        REQUIRE(slam.get_num_landmarks() == 0);
    }
    
    SECTION("Two landmarks") {
        auto initial_state = arma::vec(7, arma::fill::zeros);
        initial_state(3) = 1.0;  // landmark 0 at (1, 2)
        initial_state(4) = 2.0;
        initial_state(5) = 3.0;  // landmark 1 at (3, 4)
        initial_state(6) = 4.0;
        auto initial_cov = arma::eye(7, 7) * 0.1;
        auto slam = DDSLAM(wheel_radius, wheel_track, R, Q_two_landmarks, initial_state, initial_cov);
        
        REQUIRE(slam.get_num_landmarks() == 2);
    }
    
    SECTION("Three landmarks") {
        auto initial_state = arma::vec(9, arma::fill::zeros);
        auto initial_cov = arma::eye(9, 9) * 0.1;
        auto slam = DDSLAM(wheel_radius, wheel_track, R, Q_three_landmarks, initial_state, initial_cov);
        
        REQUIRE(slam.get_num_landmarks() == 3);
    }
}

TEST_CASE("DDSLAM get_landmark_positions returns correct positions", "[DDSLAM]")
{
    const auto wheel_radius = 0.1;
    const auto wheel_track = 0.5;
    const auto state_dim = 7;  // [theta, x, y, lm0_x, lm0_y, lm1_x, lm1_y]
    
    auto R = arma::eye(2, 2);
    auto Q = arma::eye(state_dim, state_dim) * 0.01;
    auto initial_state = arma::vec(state_dim);
    initial_state(0) = 0.0;  // robot at origin
    initial_state(1) = 0.0;
    initial_state(2) = 0.0;
    initial_state(3) = 1.5;  // landmark 0 at (1.5, 2.5)
    initial_state(4) = 2.5;
    initial_state(5) = -3.0; // landmark 1 at (-3.0, 4.0)
    initial_state(6) = 4.0;
    auto initial_cov = arma::eye(state_dim, state_dim) * 0.1;
    
    auto slam = DDSLAM(wheel_radius, wheel_track, R, Q, initial_state, initial_cov);
    
    auto landmarks = slam.get_landmark_positions();
    
    // Check dimensions: 2 rows (x, y), 2 columns (2 landmarks)
    REQUIRE(landmarks.n_rows == 2);
    REQUIRE(landmarks.n_cols == 2);
    
    // Check landmark 0 position
    REQUIRE_THAT(landmarks(0, 0), WithinAbs(1.5, 1e-6));
    REQUIRE_THAT(landmarks(1, 0), WithinAbs(2.5, 1e-6));
    
    // Check landmark 1 position
    REQUIRE_THAT(landmarks(0, 1), WithinAbs(-3.0, 1e-6));
    REQUIRE_THAT(landmarks(1, 1), WithinAbs(4.0, 1e-6));
}
