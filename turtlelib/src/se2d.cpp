/// \file
/// \brief Implementation for se2d.hpp

#include "turtlelib/se2d.hpp"
#include <iostream>
#include <cmath>
#include "turtlelib/angle.hpp"

namespace turtlelib
{
    std::istream & operator>>(std::istream & is, Twist2D & tw)
    {
        char c = is.peek();
        bool brackets = (c == '<');
        
        if (brackets) {
            // consume '<'
            is.get(); 
        }
        
        is >> tw.omega;
        
        // Check for optional unit
        // skip whitespace
        is >> std::ws; 
        c = is.peek();
        if (std::isalpha(c)) {
            std::string unit;
            is >> unit;
            if (unit[0] == 'd' || unit[0] == 'D') {
                tw.omega = normalize_angle(deg2rad(tw.omega));
            }
            else if (unit[0] == 'r' || unit[0] == 'R') {
                // If 'r' or 'R', already in radians
                tw.omega = normalize_angle(tw.omega);
            }
            else {
                is.setstate(std::ios::failbit);
                return is;
            }
        }
        
        if (brackets) {
            // consume ','
            is.get(); 
        }
        is >> tw.x;
        if (brackets) {
            // consume ','
            is.get(); 
        }
        is >> tw.y;
        
        if (brackets) {
            // consume '>'
            is.get(); 
        }
        
        return is;
    }


    Transform2D::Transform2D() : tw_ {}
    {}

    Transform2D::Transform2D(Vector2D trans) : 
    tw_ {0.0, trans.x, trans.y}
    {}

    Transform2D::Transform2D(double radians) : 
    tw_ {radians, 0.0, 0.0}
    {}

    Transform2D::Transform2D(Vector2D trans, double radians) :
    tw_ {radians, trans.x, trans.y}
    {}

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
        
        return {omega_new, vx_new, vy_new};
    }

    Transform2D Transform2D::inv() const
    {
        auto v = Vector2D{-tw_.x, -tw_.y};
        return Transform2D(v, -tw_.omega);
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        auto twist = Twist2D{rhs.rotation(), rhs.translation().x, rhs.translation().y};
        tw_ = (*this)(twist);
        return *this;
    }

    Vector2D Transform2D::translation() const
    {
        // TODO: implement
        return Vector2D{tw_.x, tw_.y};
    }

    double Transform2D::rotation() const
    {
        return tw_.omega;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf)
    {
        // TODO: implement
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        // TODO: implement
        return lhs *= rhs;
    }
}
