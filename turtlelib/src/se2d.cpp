/// \file
/// \brief Implementation for se2d.hpp

#include "turtlelib/se2d.hpp"
#include <iostream>
#include <cmath>
#include "turtlelib/angle.hpp"

namespace turtlelib
{
    std::istream &operator>>(std::istream &is, Twist2D &tw)
    {
        char c = is.peek();
        bool brackets = (c == '<');

        if (brackets)
        {
            // consume '<'
            is.get();
        }

        is >> tw.omega;

        // Check for optional unit
        // skip whitespace
        is >> std::ws;
        c = is.peek();
        if (std::isalpha(c))
        {
            std::string unit;
            is >> unit;
            if (unit[0] == 'd' || unit[0] == 'D')
            {
                tw.omega = normalize_angle(deg2rad(tw.omega));
            }
            else if (unit[0] == 'r' || unit[0] == 'R')
            {
                // If 'r' or 'R', already in radians
                tw.omega = normalize_angle(tw.omega);
            }
            else
            {
                is.setstate(std::ios::failbit);
                return is;
            }
        }

        if (brackets)
        {
            // consume ','
            is.get();
        }
        is >> tw.x;
        if (brackets)
        {
            // consume ','
            is.get();
        }
        is >> tw.y;

        if (brackets)
        {
            // consume '>'
            is.get();
        }

        return is;
    }

    Transform2D::Transform2D()
        : tw_{}
    {
    }

    Transform2D::Transform2D(Vector2D trans)
        : tw_{0.0, trans.x, trans.y}
    {
    }

    Transform2D::Transform2D(double radians)
        : tw_{radians, 0.0, 0.0}
    {
    }

    Transform2D::Transform2D(Vector2D trans, double radians)
        : tw_{radians, trans.x, trans.y}
    {
    }

    Point2D Transform2D::operator()(Point2D p) const
    {
        // Rotate the point, then translate
        double cos_theta = std::cos(tw_.omega);
        double sin_theta = std::sin(tw_.omega);

        double rotated_x = p.x * cos_theta - p.y * sin_theta;
        double rotated_y = p.x * sin_theta + p.y * cos_theta;

        return Point2D{rotated_x + tw_.x, rotated_y + tw_.y};
    }

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        // Only rotate the vector (no translation, by definition of vector transform)
        double cos_theta = std::cos(tw_.omega);
        double sin_theta = std::sin(tw_.omega);

        return Vector2D{v.x * cos_theta - v.y * sin_theta,
                        v.x * sin_theta + v.y * cos_theta};
    }

    Twist2D Transform2D::operator()(Twist2D v) const
    {
        double omega_new = v.omega;
        double vx_new = tw_.y * v.omega + cos(tw_.omega) * v.x - sin(tw_.omega) * v.y;
        double vy_new = -tw_.x * v.omega + sin(tw_.omega) * v.x + cos(tw_.omega) * v.y;

        auto out = Twist2D{normalize_angle(omega_new), vx_new, vy_new};

        return out;
    }

    Transform2D Transform2D::inv() const
    {
        auto v = Vector2D{-tw_.x, -tw_.y};
        return Transform2D(v, -tw_.omega);
    }

    Transform2D &Transform2D::operator*=(const Transform2D &rhs)
    {
        // Compose transforms: T1 *= T2 means T1 = T1 * T2
        // New rotation is sum of rotations
        double new_omega = normalize_angle(tw_.omega + rhs.rotation());

        // New translation is: current translation + rotated rhs translation
        Vector2D rhs_trans = rhs.translation();
        double cos_theta = std::cos(tw_.omega);
        double sin_theta = std::sin(tw_.omega);

        double new_x = tw_.x + (rhs_trans.x * cos_theta - rhs_trans.y * sin_theta);
        double new_y = tw_.y + (rhs_trans.x * sin_theta + rhs_trans.y * cos_theta);

        tw_ = Twist2D{new_omega, new_x, new_y};
        return *this;
    }

    Vector2D Transform2D::translation() const
    {
        return Vector2D{tw_.x, tw_.y};
    }

    double Transform2D::rotation() const
    {
        return tw_.omega;
    }

    std::istream &operator>>(std::istream &is, Transform2D &tf)
    {
        double angle, x, y;
        char c = is.peek();
        bool brackets = (c == '{');

        if (brackets)
        {
            // consume '{'
            is.get();
        }

        is >> angle;

        // Check for optional unit
        // skip whitespace
        is >> std::ws;
        c = is.peek();
        if (std::isalpha(c))
        {
            std::string unit;
            is >> unit;
            if (unit[0] == 'd' || unit[0] == 'D')
            {
                angle = normalize_angle(deg2rad(angle));
            }
            else if (unit[0] == 'r' || unit[0] == 'R')
            {
                // If 'r' or 'R', already in radians
                angle = normalize_angle(angle);
            }
            else
            {
                is.setstate(std::ios::failbit);
                return is;
            }
        }

        if (brackets)
        {
            // consume ','
            is.get();
        }
        is >> x;
        if (brackets)
        {
            // consume ','
            is.get();
        }
        is >> y;

        if (brackets)
        {
            // consume '}'
            is.get();
        }

        tf = Transform2D({x, y}, angle);

        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D &rhs)
    {
        lhs *= rhs;
        return lhs;
    }
}
