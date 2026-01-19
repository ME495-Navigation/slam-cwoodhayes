#ifndef TURTLELIB_ANGLE_HPP_INCLUDE_GUARD
#define TURTLELIB_ANGLE_HPP_INCLUDE_GUARD

/// \brief Functions for handling angles

#include <numbers>
#include <cmath>

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
        return deg / 360.0 * 2 * pi;
    }

    /// \brief Convert radians to degrees
    /// \param rad  Angle in radians
    /// \returns The equivalent angle in degrees
    constexpr double rad2deg(double rad)
    {
        using std::numbers::pi;
        return rad / (2 * pi) * 360.0;
    }

    /// \brief Wrap an angle to (-PI, PI]
    /// \param rad Angle in radians
    /// \return An equivalent angle the range (-PI, PI]
    constexpr double normalize_angle(double rad)
    {
        using std::numbers::pi;

        // need to do modulo manually to comply with constexpr
        auto quo = rad / (2.0 * pi);
        auto floor_quo = static_cast<long long>(quo);
        auto rem = rad - floor_quo * (2.0 * pi);

        if (rem > pi) {
            rem -= 2.0*pi;
        }
        return rem;
    }

    /// static_assertions test compile time assumptions.
    /// These tests can provide assurance that your code is correct at compile time!
    static_assert(almost_equal(0, 0), "is_zero failed");
    /// TASK: Write (at least) the following tests:
    /// 1. Compare two numbers where almost_equal is true or false depending on the epsilon argument
    ///    almost_equal(x, y, e1) == true
    ///    almost_equal(x, y, e2) == false
    /// 2. Compare negative and positive numbers that should not be equal
    /// 3. Compare negative and positive numbers that should be equal

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    /// TASK: Write at least 4 additional tests for deg2rad. Include at least one negative angle
    /// HINT: It helps to use angles where there is a simple known formula (e.g., 30 degrees)

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");
    /// TASK: Write at least 4 additional tests for rad2deg. Include at least one negative angle
    /// HINT: It helps to use angles where there is a simple known formula (e.g., 30 degrees)


    static_assert(almost_equal(normalize_angle(0.0), 0.0), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(std::numbers::pi), std::numbers::pi), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(-std::numbers::pi), std::numbers::pi), "norm_angle failed");
    /// Task: Write at least 8 additional tests for normalize_angle. This function is absolutely critical, so you want to get it right!
    /// Include at least once case  where the angle > 5.0*pi and one where the angle < -5.0*pi
    /// Also include: pi, -pi, -pi/4.0, 3*pi/2, and -5*pi/2.


}
#endif
