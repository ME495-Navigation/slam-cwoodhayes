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
    arma::mat R = arma::eye(2, 2);  // measurement noise (2D: range, bearing)
    arma::mat Q = arma::eye(3, 3);  // robot pose process noise block
    auto initial_state = arma::vec(state_dim, arma::fill::zeros);
    arma::mat initial_cov = arma::eye(state_dim, state_dim);
    
    // Construct DDSLAM
    auto slam = DDSLAM(wheel_radius, wheel_track, R, Q, initial_state, initial_cov, 5);
    
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

TEST_CASE("DDSLAM rejects non-robot-only initial state", "[DDSLAM]")
{
        const auto wheel_radius = 0.1;
        const auto wheel_track = 0.5;

        arma::mat R = arma::eye(2, 2);
        arma::mat Q = arma::eye(3, 3);
        auto bad_initial_state = arma::vec(5, arma::fill::zeros);
        auto bad_initial_cov = arma::eye(5, 5);

        REQUIRE_THROWS_AS(
            DDSLAM(wheel_radius, wheel_track, R, Q, bad_initial_state, bad_initial_cov, 5),
            std::runtime_error);
}

TEST_CASE("DDSLAM prediction step (odom_update)", "[DDSLAM]")
{
    const auto wheel_radius = 0.1;
    const auto wheel_track = 0.5;
    const auto state_dim = 3;
    
    arma::mat R = arma::eye(2, 2);
    arma::mat Q = arma::eye(3, 3) * 0.01;
    auto initial_state = arma::vec(state_dim, arma::fill::zeros);
    arma::mat initial_cov = arma::eye(state_dim, state_dim) * 0.1;
    
    auto slam = DDSLAM(wheel_radius, wheel_track, R, Q, initial_state, initial_cov, 5);
    
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
    const auto state_dim = 3;  // [theta, x, y]
    
    arma::mat R = arma::eye(2, 2);
    arma::mat Q = arma::eye(3, 3) * 0.01;
    auto initial_state = arma::vec(state_dim, arma::fill::zeros);
    arma::mat initial_cov = arma::eye(state_dim, state_dim) * 0.1;
    
    auto slam = DDSLAM(wheel_radius, wheel_track, R, Q, initial_state, initial_cov, 5);
    
    // Perform a measurement update: observe landmark 0 at range 1.0, bearing 0.0
    slam.measurement_update(0, 1.0, 0.0);
    
    auto state_after = slam.get_state();
    auto cov_after = slam.get_covariance();
    
    // Check dimensions grew by one landmark
    REQUIRE(state_after.n_rows == state_dim + 2);
    REQUIRE(cov_after.n_rows == state_dim + 2);
    REQUIRE(cov_after.n_cols == state_dim + 2);

    auto landmark_ids = slam.get_landmark_ids();
    REQUIRE(landmark_ids.size() == 1);
    REQUIRE(landmark_ids[0] == 0);
}

TEST_CASE("DDSLAM repeated measurement does not grow state twice", "[DDSLAM]")
{
    const auto wheel_radius = 0.1;
    const auto wheel_track = 0.5;
    const auto state_dim = 3;

    arma::mat R = arma::eye(2, 2) * 0.01;
    arma::mat Q = arma::eye(3, 3) * 0.01;
    auto initial_state = arma::vec(state_dim, arma::fill::zeros);
    arma::mat initial_cov = arma::eye(state_dim, state_dim) * 0.1;

    auto slam = DDSLAM(wheel_radius, wheel_track, R, Q, initial_state, initial_cov, 5);

    slam.measurement_update(0, 1.0, 0.0);
    auto rows_after_first = slam.get_state().n_rows;
    slam.measurement_update(0, 1.0, 0.0);

    auto state_after = slam.get_state();
    auto cov_after = slam.get_covariance();

    REQUIRE(rows_after_first == state_dim + 2);
    REQUIRE(state_after.n_rows == rows_after_first);
    REQUIRE(cov_after.n_rows == rows_after_first);
    REQUIRE(cov_after.n_cols == rows_after_first);
    REQUIRE(state_after.is_finite());
    REQUIRE(cov_after.is_finite());
}

TEST_CASE("DDSLAM enforces max landmarks cap", "[DDSLAM]")
{
    const auto wheel_radius = 0.1;
    const auto wheel_track = 0.5;

    arma::mat R = arma::eye(2, 2) * 0.01;
    arma::mat Q = arma::eye(3, 3) * 0.01;
    auto initial_state = arma::vec(3, arma::fill::zeros);
    arma::mat initial_cov = arma::eye(3, 3) * 0.1;
    auto slam = DDSLAM(wheel_radius, wheel_track, R, Q, initial_state, initial_cov, 1);

    slam.measurement_update(10, 1.0, 0.0);
    REQUIRE(slam.get_num_landmarks() == 1);
    REQUIRE_THROWS_AS(slam.measurement_update(11, 2.0, 0.1), std::runtime_error);
}

TEST_CASE("DDSLAM mixed prediction and measurement updates", "[DDSLAM]")
{
    const auto wheel_radius = 0.1;
    const auto wheel_track = 0.5;
    const auto state_dim = 3;
    
    arma::mat R = arma::eye(2, 2) * 0.01;
    arma::mat Q = arma::eye(3, 3) * 0.01;
    auto initial_state = arma::vec(state_dim, arma::fill::zeros);
    arma::mat initial_cov = arma::eye(state_dim, state_dim) * 0.1;
    
    auto slam = DDSLAM(wheel_radius, wheel_track, R, Q, initial_state, initial_cov, 5);
    
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
    
    arma::mat R = arma::eye(2, 2);
    arma::mat Q = arma::eye(3, 3) * 0.01;
    auto initial_state = arma::vec(state_dim);
    initial_state(0) = pi / 4.0;  // theta = 45 degrees
    initial_state(1) = 2.0;       // x = 2.0
    initial_state(2) = 3.0;       // y = 3.0
    arma::mat initial_cov = arma::eye(state_dim, state_dim) * 0.1;
    
    auto slam = DDSLAM(wheel_radius, wheel_track, R, Q, initial_state, initial_cov, 5);
    
    auto T_mo = slam.get_map_to_body();
    
    // Check that the transform matches the state
    REQUIRE_THAT(T_mo.rotation(), WithinAbs(pi / 4.0, 1e-6));
    REQUIRE_THAT(T_mo.translation().x, WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(T_mo.translation().y, WithinAbs(3.0, 1e-6));
}

TEST_CASE("DDSLAM get_num_landmarks returns correct count", "[DDSLAM]")
{
    const auto wheel_radius = 0.1;
    const auto wheel_track = 0.5;
    
    arma::mat R = arma::eye(2, 2);
    arma::mat Q_robot_pose = arma::eye(3, 3) * 0.01;
    
    SECTION("No landmarks") {
        auto initial_state = arma::vec(3, arma::fill::zeros);
        arma::mat initial_cov = arma::eye(3, 3) * 0.1;
        auto slam = DDSLAM(wheel_radius, wheel_track, R, Q_robot_pose, initial_state, initial_cov, 5);
        
        REQUIRE(slam.get_num_landmarks() == 0);
    }
    
    SECTION("Two landmarks after observations") {
        auto initial_state = arma::vec(3, arma::fill::zeros);
        arma::mat initial_cov = arma::eye(3, 3) * 0.1;
        auto slam = DDSLAM(wheel_radius, wheel_track, R, Q_robot_pose, initial_state, initial_cov, 5);

        slam.measurement_update(7, 1.0, 0.0);
        slam.measurement_update(9, 2.0, 0.0);
        
        REQUIRE(slam.get_num_landmarks() == 2);
    }
}

TEST_CASE("DDSLAM get_landmark_positions returns correct positions", "[DDSLAM]")
{
    const auto wheel_radius = 0.1;
    const auto wheel_track = 0.5;
    const auto state_dim = 3;  // [theta, x, y]
    
    arma::mat R = arma::eye(2, 2);
    arma::mat Q = arma::eye(3, 3) * 0.01;
    auto initial_state = arma::vec(state_dim);
    initial_state(0) = 0.0;  // robot at origin
    initial_state(1) = 0.0;
    initial_state(2) = 0.0;
    arma::mat initial_cov = arma::eye(state_dim, state_dim) * 0.1;
    
    auto slam = DDSLAM(wheel_radius, wheel_track, R, Q, initial_state, initial_cov, 5);
    slam.measurement_update(21, std::hypot(1.5, 2.5), std::atan2(2.5, 1.5));
    slam.measurement_update(22, 5.0, std::atan2(4.0, -3.0));
    
    auto landmarks = slam.get_landmark_positions();
    auto landmark_ids = slam.get_landmark_ids();
    
    // Check dimensions: 2 rows (x, y), 2 columns (2 landmarks)
    REQUIRE(landmarks.n_rows == 2);
    REQUIRE(landmarks.n_cols == 2);
    
    // Check landmark 0 position
    REQUIRE_THAT(landmarks(0, 0), WithinAbs(1.5, 1e-6));
    REQUIRE_THAT(landmarks(1, 0), WithinAbs(2.5, 1e-6));
    
    // Check landmark 1 position
    REQUIRE_THAT(landmarks(0, 1), WithinAbs(-3.0, 1e-6));
    REQUIRE_THAT(landmarks(1, 1), WithinAbs(4.0, 1e-6));

    REQUIRE(landmark_ids.size() == 2);
    REQUIRE(landmark_ids[0] == 21);
    REQUIRE(landmark_ids[1] == 22);
}
