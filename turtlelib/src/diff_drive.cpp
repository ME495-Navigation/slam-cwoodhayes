#include "turtlelib/diff_drive.hpp"

#include <stdexcept>

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
        // but assume we didn't move for the first tick received
        // during object lifetime, because we will initialize to random encoder values
        auto dphi_l = 0.0;
        auto dphi_r = 0.0;
        if (has_received_initial_wheel_angle_) {
            dphi_l = new_phi_left - phi_left_;
            dphi_r = new_phi_right - phi_right_;
        }
        else {
            has_received_initial_wheel_angle_ = true;
        }

        // get the arc length for each wheel's trajectory
        auto s_l = dphi_l * wheel_radius_;
        auto s_r = dphi_r * wheel_radius_;

        // get omega for all twists (equation 1)
        auto omega = (s_r - s_l) / wheel_track_;

        // get arc length for the body frame twist (equation 2)
        auto s_b = (s_r + s_l) / 2.0;

        // because this is diff drive, the twist in the body frame has no y component
        Vb_ = Twist2D{omega, s_b, 0.0};

        // integrate the body twist to get the change in pose of the robot in the space frame
        auto T_bbprime = integrate_twist(Vb_);

        // update the wheel angles and robot pose
        T_sb_ = T_sb_ * T_bbprime;
        phi_dot_left_ = dphi_l;
        phi_dot_right_ = dphi_r;
        phi_left_ = new_phi_left;
        phi_right_ = new_phi_right;

        return T_sb_;
    }

    std::pair<double, double> DiffDrive::inverse_kinematics(const Twist2D& Vb) const
    {
        // input checking
        if (Vb.y != 0.0)
        {
            auto errmsg = std::format("DiffDrive IK does not support a twist with a y component. Received Vb = {}", Vb);
            throw std::logic_error(errmsg);
        }

        // equation 3
        const auto v_l = Vb.x - (Vb.omega * wheel_track_ / 2.0);
        const auto v_r = Vb.x + (Vb.omega * wheel_track_ / 2.0);

        return {v_l / wheel_radius_, v_r / wheel_radius_};
    }

    void DiffDrive::reset_to_configuration(const Transform2D& new_pose)
    {
        T_sb_ = new_pose;
        phi_left_ = 0.0;
        phi_right_ = 0.0;
        phi_dot_left_ = 0.0;
        phi_dot_right_ = 0.0;
        has_received_initial_wheel_angle_ = false;
    }
}