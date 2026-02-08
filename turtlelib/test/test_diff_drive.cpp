#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include "turtlelib/diff_drive.hpp"

using namespace turtlelib;
using namespace Catch::Matchers;
using std::numbers::pi;

TEST_CASE("DiffDrive forward motion (forward/inverse)", "[Conor]")
{
    const auto wheel_radius = 0.1;
    const auto wheel_track = 0.5;

    auto dd = DiffDrive{wheel_radius, wheel_track};

    auto tf = dd.forward_kinematics(2.0 * pi, 2.0 * pi);

    REQUIRE_THAT(tf.rotation(), WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(tf.translation().x, WithinAbs(2.0 * pi * wheel_radius, 1e-6));
    REQUIRE_THAT(tf.translation().y, WithinAbs(0.0, 1e-6));

    auto wheels = dd.inverse_kinematics(Twist2D{0.0, 0.1, 0.0});
    REQUIRE_THAT(wheels.first, WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(wheels.second, WithinAbs(1.0, 1e-6));
}

TEST_CASE("DiffDrive pure rotation (forward/inverse)", "[Conor]")
{
    const auto wheel_radius = 0.1;
    const auto wheel_track = 0.5;

    auto dd = DiffDrive{wheel_radius, wheel_track};

    // spin around 180 degrees
    // this means 0.5/2 * pi = 0.25pi arc length for both wheels
    // so the wheels have to spin 0.25pi / 0.1 = 2.5pi radians in opposite directions
    auto tf = dd.forward_kinematics(-pi * 2.5, pi * 2.5);

    REQUIRE_THAT(tf.translation().x, WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(tf.translation().y, WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(tf.rotation(), WithinAbs(pi, 1e-6));

    // 180 degree rotation in place should use the same math in reverse
    auto wheels = dd.inverse_kinematics(Twist2D{pi, 0.0, 0.0});
    REQUIRE_THAT(wheels.first, WithinAbs(-2.5 * pi, 1e-6));
    REQUIRE_THAT(wheels.second, WithinAbs(2.5 * pi, 1e-6));
}

TEST_CASE("DiffDrive circular arc (forward/inverse)", "[Conor]")
{
    const auto wheel_radius = 0.1;
    const auto wheel_track = 0.5;

    auto dd = DiffDrive{wheel_radius, wheel_track};

    auto tf = dd.forward_kinematics(1.0, 2.0);

    const auto omega = (wheel_radius / wheel_track) * (2.0 - 1.0);
    const auto v_x = (wheel_radius / 2.0) * (1.0 + 2.0);
    const auto expected_x = (v_x / omega) * std::sin(omega);
    const auto expected_y = (v_x / omega) * (1.0 - std::cos(omega));

    REQUIRE_THAT(tf.rotation(), WithinAbs(omega, 1e-6));
    REQUIRE_THAT(tf.translation().x, WithinAbs(expected_x, 1e-6));
    REQUIRE_THAT(tf.translation().y, WithinAbs(expected_y, 1e-6));

    auto wheels = dd.inverse_kinematics(Twist2D{omega, v_x, 0.0});
    REQUIRE_THAT(wheels.first, WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(wheels.second, WithinAbs(2.0, 1e-6));
}

TEST_CASE("DiffDrive inverse kinematics rejects lateral motion", "[Conor]")
{
    auto dd = DiffDrive{0.1, 0.5};

    REQUIRE_THROWS_AS(dd.inverse_kinematics(Twist2D{0.0, 0.1, 0.01}), std::logic_error);
}