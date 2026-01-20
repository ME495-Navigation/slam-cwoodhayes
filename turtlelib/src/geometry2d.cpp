/// \file
/// \brief Implementation for geometry2d.hpp

#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
    std::istream & operator>>(std::istream & is, Point2D & p)
    {
        return is;
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail)
    {
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp)
    {
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
    }

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
    }

    Vector2D normalize(Vector2D in)
    {
    }
}
