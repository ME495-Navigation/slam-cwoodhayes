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
        // TODO: implement
        return p;
    }

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        // TODO: implement
        return v;
    }

    Twist2D Transform2D::operator()(Twist2D v) const
    {
        // TODO: implement
        return v;
    }

    Transform2D Transform2D::inv() const
    {
        // TODO: implement
        return Transform2D();
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        // TODO: implement
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
