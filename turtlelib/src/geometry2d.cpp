/// \file
/// \brief Implementation for geometry2d.hpp

#include "turtlelib/geometry2d.hpp"
#include <iostream>
#include <cmath>
#include <stdexcept>

namespace turtlelib
{
    Vector2D & Vector2D::operator+=( const Vector2D& rhs )
    {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator-=( const Vector2D& rhs )
    {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator*=( const double scalar )
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    std::istream & operator>>(std::istream & is, Point2D & p)
    {
        char c = is.peek();
        
        if (c == '(') {
            // Format: (x, y)
            is.get(); // consume '('
            is >> p.x;
            is.get(); // consume ','
            is >> p.y;
            is.get(); // consume ')'
        } else {
            // Format: x y
            is >> p.x >> p.y;
        }
        
        return is;
    }

    

    Vector2D operator-(const Point2D & head, const Point2D & tail)
    {
        return {head.x - tail.x, head.y - tail.y};
    }

    Vector2D operator+(const Vector2D& v1, const Vector2D& v2)
    {
        auto res = v1;
        res += v2;
        return res;
    }

    Vector2D operator-(const Vector2D& v1, const Vector2D& v2)
    {
        auto res = v1;
        res -= v2;
        return res;
    }

    Vector2D operator*(const Vector2D& v1, const double scalar)
    {
        auto res = v1;
        res *= scalar;
        return res;
    }

    Vector2D operator*(const double scalar, const Vector2D& v1)
    {
        return v1 * scalar;
    }

    double dot(const Vector2D& v1, const Vector2D& v2)
    {
        return v1.x * v2.x + v1.y * v2.y;
    }

    double magnitude(const Vector2D& v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y);
    }

    double angle(const Vector2D& v1, const Vector2D& v2)
    {
        return std::acos(dot(v1, v2) / (magnitude(v1) * magnitude(v2)));
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp)
    {
        return {tail.x + disp.x, tail.y + disp.y};
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        os << "[" << v.x << ", " << v.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        char c = is.peek();
        
        if (c == '[') {
            // Format: [x, y]
            is.get(); // consume '['
            is >> v.x;
            is.get(); // consume ','
            is >> v.y;
            is.get(); // consume ']'
        } else {
            // Format: x y
            is >> v.x >> v.y;
        }
        
        return is;
    }

    Vector2D normalize(Vector2D in)
    {
        const auto magnitude = std::sqrt(in.x * in.x + in.y * in.y); // const auto
        
        if (magnitude == 0.0) {
            throw std::invalid_argument("Cannot normalize a zero vector");
        }
        
        return Vector2D{in.x / magnitude, in.y / magnitude};
    }
}
