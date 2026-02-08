#ifndef TURTLELIB_ANGLE_HPP_INCLUDE_GUARD
#define TURTLELIB_ANGLE_HPP_INCLUDE_GUARD

/// \brief Functions for handling angles

#include <numbers>
#include <cmath>
#include <vector>

namespace turtlelib
{
    /// \brief Approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 A number to compare
    /// \param d2 A second number to compare
    /// \param epsilon Absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        return std::abs(d1 - d2) < epsilon;
    }

    /// \brief Convert degrees to radians
    /// \param deg Angle in degrees
    /// \returns The equivalent angle in radians
    constexpr double deg2rad(double deg)
    {
        using std::numbers::pi;
        return deg / 360.0 * 2.0 * pi;
    }

    /// \brief Convert radians to degrees
    /// \param rad  Angle in radians
    /// \returns The equivalent angle in degrees
    constexpr double rad2deg(double rad)
    {
        using std::numbers::pi;
        return rad / (2.0 * pi) * 360.0;
    }

    /// \brief Wrap an angle to (-PI, PI]
    /// \param rad Angle in radians
    /// \return An equivalent angle the range (-PI, PI]
    constexpr double normalize_angle(double rad)
    {
        using std::numbers::pi;

        auto rem = std::fmod(rad, 2.0 * pi);

        if (rem > pi) {
            rem -= 2.0*pi;
        }
        else if (rem <= -pi) {
            rem += 2.0*pi;
        }
        return rem;
    }

    /// @brief Convert a planar angle to a quaternion (x, y, z, w) restricted to the xy plane.
    /// @param theta Angle in radians representing rotation about the z axis
    /// @return Quaternion as an array of 4 doubles (x, y, z, w)
    constexpr std::vector<double> angle_to_2d_planar_quaternion(double theta)
    {
        using std::numbers::pi;
        const auto half_theta = theta / 2.0;
        return {0.0, 0.0, std::sin(half_theta), std::cos(half_theta)};
    }


    // ############################ Begin_Citation [2] ############################

    /// static_assertions test compile time assumptions.
    /// These tests can provide assurance that your code is correct at compile time!
    static_assert(almost_equal(0, 0), "is_zero failed");
    /// TASK: Write (at least) the following tests:
    /// 1. Compare two numbers where almost_equal is true or false depending on the epsilon argument
    ///    almost_equal(x, y, e1) == true
    ///    almost_equal(x, y, e2) == false
    static_assert(almost_equal(1.0, 1.0 + 1e-13, 1e-12), "almost_equal with larger epsilon failed");
    static_assert(!almost_equal(1.0, 1.0 + 1e-11, 1e-12), "almost_equal with smaller epsilon failed");

    /// 2. Compare negative and positive numbers that should not be equal
    static_assert(!almost_equal(-5.0, 5.0), "almost_equal negative vs positive failed");

    /// 3. Compare negative and positive numbers that should be equal
    static_assert(almost_equal(-1e-13, 1e-13), "almost_equal negative and positive near zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    /// TASK: Write at least 4 additional tests for deg2rad. Include at least one negative angle
    /// HINT: It helps to use angles where there is a simple known formula (e.g., 30 degrees)
    static_assert(almost_equal(deg2rad(180.0), std::numbers::pi), "deg2rad 180 degrees failed");
    static_assert(almost_equal(deg2rad(90.0), std::numbers::pi / 2.0), "deg2rad 90 degrees failed");
    static_assert(almost_equal(deg2rad(360.0), 2.0 * std::numbers::pi), "deg2rad 360 degrees failed");
    static_assert(almost_equal(deg2rad(-90.0), -std::numbers::pi / 2.0), "deg2rad -90 degrees failed");
    static_assert(almost_equal(deg2rad(45.0), std::numbers::pi / 4.0), "deg2rad 45 degrees failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");
    /// TASK: Write at least 4 additional tests for rad2deg. Include at least one negative angle
    /// HINT: It helps to use angles where there is a simple known formula (e.g., 30 degrees)
    static_assert(almost_equal(rad2deg(std::numbers::pi), 180.0), "rad2deg pi radians failed");
    static_assert(almost_equal(rad2deg(std::numbers::pi / 2.0), 90.0), "rad2deg pi/2 radians failed");
    static_assert(almost_equal(rad2deg(2.0 * std::numbers::pi), 360.0), "rad2deg 2pi radians failed");
    static_assert(almost_equal(rad2deg(-std::numbers::pi / 2.0), -90.0), "rad2deg -pi/2 radians failed");
    static_assert(almost_equal(rad2deg(std::numbers::pi / 4.0), 45.0), "rad2deg pi/4 radians failed");

    static_assert(almost_equal(normalize_angle(0.0), 0.0), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(std::numbers::pi), std::numbers::pi), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(-std::numbers::pi), std::numbers::pi), "norm_angle failed");
    /// Task: Write at least 8 additional tests for normalize_angle. This function is absolutely critical, so you want to get it right!
    /// Include at least once case  where the angle > 5.0*pi and one where the angle < -5.0*pi
    /// Also include: pi, -pi, -pi/4.0, 3*pi/2, and -5*pi/2.
    using std::numbers::pi;
    static_assert(almost_equal(normalize_angle(-pi / 4.0), -pi / 4.0), "normalize_angle -pi/4 failed");
    static_assert(almost_equal(normalize_angle(3.0 * pi / 2.0), -pi / 2.0), "normalize_angle 3pi/2 failed");
    static_assert(almost_equal(normalize_angle(-5.0 * pi / 2.0), -pi / 2.0), "normalize_angle -5pi/2 failed");
    static_assert(almost_equal(normalize_angle(5.5 * pi), -0.5 * pi), "normalize_angle >5pi failed");
    static_assert(almost_equal(normalize_angle(-5.5 * pi), 0.5 * pi), "normalize_angle <-5pi failed");
    static_assert(almost_equal(normalize_angle(2.0 * pi), 0.0), "normalize_angle 2pi failed");
    static_assert(almost_equal(normalize_angle(-2.0 * pi), 0.0), "normalize_angle -2pi failed");
    static_assert(almost_equal(normalize_angle(pi / 2.0), pi / 2.0), "normalize_angle pi/2 failed");
    static_assert(almost_equal(normalize_angle(7.0 * pi / 4.0), -pi / 4.0), "normalize_angle 7pi/4 failed");
    static_assert(almost_equal(normalize_angle(1), 1), "normalize angle 1 failed");
    // ############################ End_Citation [2] ############################
}
#endif
