#ifndef TURTLELIB_SE2_INCLUDE_GUARD_HPP
#define TURTLELIB_SE2_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include<iosfwd>
#include<format>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/angle.hpp"


namespace turtlelib
{

    /// \brief represent a 2-Dimensional twist
    struct Twist2D
    {
        /// \brief the angular velocity
        double omega = 0.0;

        /// \brief the linear x velocity
        double x = 0.0;

        /// \brief the linear y velocity
        double y = 0.0;

        /// \brief scale a twist by a scalar
        /// \param scalar The scaling factor
        /// \return A reference to this twist after scaling
        Twist2D & operator*=(const double scalar);

        Vector2D v() { return {x, y}; }
    };

    /// \brief scale a twist by a scalar
    /// \param tw The twist to scale
    /// \param scalar The scaling factor
    Twist2D operator*(const Twist2D& tw, const double scalar);

    /// \brief scale a twist by a scalar
    /// \param scalar The scaling factor
    /// \param tw The twist to scale
    Twist2D operator*(const double scalar, const Twist2D& tw);

    /// \brief read the Twist2D in the format "<w [<unit>], x, y>" or as "w [<unit>] x y"
    /// The "" are not part of the input.
    /// The [<unit>] is optional and can be any string without spaces that starts with an r
    /// (for rad/s) and any string without spaces that starts with a d for deg/s)
    /// If the unit is omitted, assume rad/s.
    /// \param is [in/out] the istream to read from
    /// \param tw [out] the twist read from the stream
    /// \returns the istream is with the twist characters removed
    std::istream & operator>>(std::istream & is, Twist2D & tw);


    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief copy constructor
        Transform2D(const Transform2D &) = default;

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(Vector2D trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param radians - the rotation, in radians
        Transform2D(Vector2D trans, double radians);

        /// \brief apply a transformation to a 2D Point
        /// \param p the point to transform
        /// \return a point in the new coordinate system
        Point2D operator()(Point2D p) const;

        /// \brief apply a transformation to a 2D Vector
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief apply a transformation to a Twist2D (e.g. using the adjoint)
        /// \param v - the twist to transform
        /// \return a twist in the new coordinate system
        Twist2D operator()(Twist2D v) const;

        /// \brief invert the transformation
        /// \return the inverse transformation.
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
        Vector2D translation() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
        double rotation() const;

        /// \brief see std::formatter for the Transform2D
        template<class CharT>
        friend struct std::formatter;
    
    private:
        Twist2D tw_;



    };

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as:
    ///  "theta [<unit>] dx dy" (i.e., three numbers separated by whitespace, angle assumed to be radians)
    //   "{<angle> [<unit>], <x>, <y>}" (as output by std::format)
    ///  [<unit>] is optional and can be any string without spaces that starts with a d for deg or r for rad
    ///  If [<unit>] is omitted, assume the unit is radians
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// \brief integrate a twist to get a transform
    /// \param tw The twist to integrate
    /// \return the transform corresponding to integrating the twist for 1 second
    Transform2D integrate_twist(const Twist2D& tw);
}


/// \brief A formatter for Transform2D
/// Creates a string representation of a Transform2D
/// as "{<angle> [<unit>], <x>, <y>}"
/// An R at the beginning of the format-spec makes [<unit>] rad
/// A D  at the beginning of the format-spec makes [<unit>] deg
/// No R or D means no unit is printed but the angle is in radians.
///
/// After the optional "unit specifier" all double
/// format-spec values are accepted and apply to all numbers that
/// are put into the string
template<class CharT>
class std::formatter<turtlelib::Transform2D, CharT>
{
private:
    char unit_spec = '\0'; // 'R', 'D', or '\0' for none
    std::string num_fmt_{};

public:
    constexpr auto parse(std::format_parse_context& ctx) {
        auto it = ctx.begin();
        
        // Check for optional unit specifier (R or D)
        if (it != ctx.end() && (*it == 'R' || *it == 'D' || *it == 'r' || *it == 'd')) {
            unit_spec = *it;
            ++it;
        }

        auto spec = std::string{};
        auto end = ctx.end();
        while (it != end && *it != '}') {
            spec.push_back(*it);
            ++it;
        }

        if (spec.empty()) {
            num_fmt_ = "{}";
        } else {
            num_fmt_.reserve(spec.size() + 3);
            num_fmt_ = "{:";
            num_fmt_ += spec;
            num_fmt_ += "}";
        }
        return it;
    }

    auto format(const turtlelib::Transform2D& tf, std::format_context& ctx) const {
        auto angle = tf.rotation();
        auto trans = tf.translation();

        std::string unit_str = "";
        if (unit_spec == 'D' || unit_spec == 'd') {
            angle = turtlelib::rad2deg(angle);
            unit_str = " deg";
        } else if (unit_spec == 'R' || unit_spec == 'r') {
            unit_str = " rad";
        }

        auto angle_str = std::vformat(num_fmt_, std::make_format_args(angle));
        auto x_str = std::vformat(num_fmt_, std::make_format_args(trans.x));
        auto y_str = std::vformat(num_fmt_, std::make_format_args(trans.y));

        return std::format_to(ctx.out(), "{{{}{}, {}, {}}}", angle_str, unit_str, x_str, y_str);
    }
};

/// \brief print the Twist2D as "<w [<unit>], x, y>"
/// An R at the beginning of the format-spec makes [<unit>] rad/s
/// A  D at the beginning of the format-spec makes [<unit>] deg/s
/// No R or D means no unit is printed but the w is taken to be in rad/s
///
/// After the optional "unit specifier" all double
/// format-spec values are accepted and apply to all numbers inserted
/// into the string.
template<class CharT>
class std::formatter<turtlelib::Twist2D, CharT>
{
private:
    char unit_spec = '\0'; // 'R', 'D', or '\0' for none
    std::string num_fmt_{};

public:
    constexpr auto parse(std::format_parse_context& ctx) {
        auto it = ctx.begin();
        
        // Check for optional unit specifier (R or D)
        if (it != ctx.end() && (*it == 'R' || *it == 'D' || *it == 'r' || *it == 'd')) {
            unit_spec = *it;
            ++it;
        }

        auto spec = std::string{};
        auto end = ctx.end();
        while (it != end && *it != '}') {
            spec.push_back(*it);
            ++it;
        }

        if (spec.empty()) {
            num_fmt_ = "{}";
        } else {
            num_fmt_.reserve(spec.size() + 3);
            num_fmt_ = "{:";
            num_fmt_ += spec;
            num_fmt_ += "}";
        }
        return it;
    }

    auto format(const turtlelib::Twist2D& tw, std::format_context& ctx) const {
        auto omega = tw.omega;
        
        // Convert to degrees if D was specified
        std::string unit_str = "";
        if (unit_spec == 'D' || unit_spec == 'd') {
            omega = turtlelib::rad2deg(omega);
            unit_str = " deg/s";
        } else if (unit_spec == 'R' || unit_spec == 'r') {
            unit_str = " rad/s";
        }

        auto omega_str = std::vformat(num_fmt_, std::make_format_args(omega));
        auto x_str = std::vformat(num_fmt_, std::make_format_args(tw.x));
        auto y_str = std::vformat(num_fmt_, std::make_format_args(tw.y));

        return std::format_to(ctx.out(), "<{}{}, {}, {}>", omega_str, unit_str, x_str, y_str);
    }
};
#endif
