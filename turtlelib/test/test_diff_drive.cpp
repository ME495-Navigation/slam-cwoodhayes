#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <stdexcept>

#include "turtlelib/diff_drive.hpp"

using namespace turtlelib;
using namespace Catch::Matchers;

TEST_CASE("DiffDrive forward motion (forward/inverse)", "[Conor]")
{
    const double wheel_radius = 0.1;
    const double wheel_track = 0.5;

    DiffDrive dd{wheel_radius, wheel_track};

    auto tf = dd.forward_kinematics(1.0, 1.0);

    REQUIRE_THAT(tf.rotation(), WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(tf.translation().x, WithinAbs(0.1, 1e-6));
    REQUIRE_THAT(tf.translation().y, WithinAbs(0.0, 1e-6));

    auto wheels = dd.inverse_kinematics(Twist2D{0.0, 0.1, 0.0});
    REQUIRE_THAT(wheels.first, WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(wheels.second, WithinAbs(1.0, 1e-6));
}

TEST_CASE("DiffDrive pure rotation (forward/inverse)", "[Conor]")
{
    const double wheel_radius = 0.1;
    const double wheel_track = 0.5;

    DiffDrive dd{wheel_radius, wheel_track};

    auto tf = dd.forward_kinematics(-1.0, 1.0);

    REQUIRE_THAT(tf.rotation(), WithinAbs(0.4, 1e-6));
    REQUIRE_THAT(tf.translation().x, WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(tf.translation().y, WithinAbs(0.0, 1e-6));

    auto wheels = dd.inverse_kinematics(Twist2D{0.4, 0.0, 0.0});
    REQUIRE_THAT(wheels.first, WithinAbs(-1.0, 1e-6));
    REQUIRE_THAT(wheels.second, WithinAbs(1.0, 1e-6));
}

TEST_CASE("DiffDrive circular arc (forward/inverse)", "[Conor]")
{
    const double wheel_radius = 0.1;
    const double wheel_track = 0.5;

    DiffDrive dd{wheel_radius, wheel_track};

    auto tf = dd.forward_kinematics(1.0, 2.0);

    const double omega = (wheel_radius / wheel_track) * (2.0 - 1.0);
    const double v_x = (wheel_radius / 2.0) * (1.0 + 2.0);
    const double expected_x = (v_x / omega) * std::sin(omega);
    const double expected_y = (v_x / omega) * (1.0 - std::cos(omega));

    REQUIRE_THAT(tf.rotation(), WithinAbs(omega, 1e-6));
    REQUIRE_THAT(tf.translation().x, WithinAbs(expected_x, 1e-6));
    REQUIRE_THAT(tf.translation().y, WithinAbs(expected_y, 1e-6));

    auto wheels = dd.inverse_kinematics(Twist2D{omega, v_x, 0.0});
    REQUIRE_THAT(wheels.first, WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(wheels.second, WithinAbs(2.0, 1e-6));
}

TEST_CASE("DiffDrive inverse kinematics rejects lateral motion", "[Conor]")
{
    DiffDrive dd{0.1, 0.5};

    REQUIRE_THROWS_AS(dd.inverse_kinematics(Twist2D{0.0, 0.1, 0.01}), std::invalid_argument);
}