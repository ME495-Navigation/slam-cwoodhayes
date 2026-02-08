#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
    DiffDrive::DiffDrive(double wheel_radius, double wheel_track)
        : wheel_radius_(wheel_radius), wheel_track_(wheel_track)
    {
        // construct transforms to wheels
        T_bl_ = Transform2D(Vector2D{-wheel_track_ / 2.0, 0.0}, 0.0);
        T_br_ = Transform2D(Vector2D{wheel_track_ / 2.0, 0.0}, 0.0);
    }

    const Transform2D& DiffDrive::forward_kinematics(double new_phi_left, double new_phi_right)
    {
        // get the change in wheel angles (wheel velocity)
        auto dphi_l = new_phi_left - phi_left_;
        auto dphi_r = new_phi_right - phi_right_;

        // get the arc length for each wheel's trajectory
        auto s_l = dphi_l * wheel_radius_;
        auto s_r = dphi_r * wheel_radius_;

        // get omega for all twists (equation 1)
        auto omega = (s_r - s_l) / wheel_track_;

        // get arc length for the body frame twist (equation 2)
        auto s_b = (s_r + s_l) / 2.0;

        // because this is diff drive, the twist in the body frame has no y component
        auto Vb = Twist2D{omega, s_b, 0.0};

        // integrate the body twist to get the change in pose of the robot in the space frame
        auto T_bbprime = integrate_twist(Vb);

        // update the wheel angles and robot pose
        T_sb_ = T_sb_ * T_bbprime;
        phi_left_ = new_phi_left;
        phi_right_ = new_phi_right;

        return T_sb_;
    }

    std::pair<double, double> DiffDrive::inverse_kinematics(const Twist2D& Vb) const
    {
        // get the twists at the wheels
        auto V_l = T_bl_.inv()(Vb);
        auto V_r = T_br_.inv()(Vb);

        // get the arc length for each wheel's trajectory
        auto s_l = magnitude(V_l.v());
        auto s_r = magnitude(V_r.v());

        // divide by radius to get radians
        return {s_l / wheel_radius_, s_r / wheel_radius_};
    }
}