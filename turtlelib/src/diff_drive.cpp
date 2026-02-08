#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
    Transform2D DiffDrive::forward_kinematics(double new_phi_left, double new_phi_right) const
    {
        (void)new_phi_left;
        (void)new_phi_right;
        return Transform2D{};
    }

    std::pair<double, double> DiffDrive::inverse_kinematics(const Twist2D& twist) const
    {
        (void)twist;
        return {0.0, 0.0};
    }
}