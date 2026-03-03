#include "turtlelib/se2d.hpp"
#include <utility>
#include <vector>

#ifndef DIFF_DRIVE_HPP
#define DIFF_DRIVE_HPP

namespace turtlelib
{
    class DiffDrive
    {
    public:
        /// @brief Construct a new Diff Drive object
        /// @param wheel_radius Radius of the wheels in meters
        /// @param wheel_track Distance between the two wheels in meters
        DiffDrive(double wheel_radius, double wheel_track);

        /// @brief Forward kinematics for a differential drive robot
        /// @param new_phi_left New wheel angle for the left wheel, given in radians
        /// @param new_phi_right New wheel angle for the right wheel, given in radians
        /// @return new pose of the robot in the space frame, as a Transform2D
        const Transform2D& forward_kinematics(double new_phi_left, double new_phi_right);

        /// @brief Inverse kinematics for a differential drive robot
        /// @param twist Desired twist of the robot in the space frame
        /// @return Left and right wheel velocities (rad/s) required to achieve the desired twist
        std::pair<double, double> inverse_kinematics(const Twist2D& twist) const;

        /// @brief Reset the robot's pose to a new value, and velocities & wheel positions to 0.
        /// @param new_pose The new pose to set for the robot, given as a Transform2D
        void reset_to_configuration(const Transform2D& new_pose);

        auto get_wheel_angles() const
        {
            return std::vector{phi_left_, phi_right_};
        }

        auto get_wheel_velocities() const
        {
            return std::vector{phi_dot_left_, phi_dot_right_};
        }

        const Transform2D& get_pose() const
        {
            return T_sb_;
        }

        void set_pose(const Transform2D& new_pose)
        {
            T_sb_ = new_pose;
        }

        const Twist2D get_body_twist() const
        {
            return Vb_;
        }

    private:
        const double wheel_radius_;
        const double wheel_track_;

        // wheel angles
        double phi_left_ = 0.0;
        double phi_right_ = 0.0;

        // we need to allow the first call to fk to initialize
        // the first wheel angle
        bool has_received_initial_wheel_angle_ = false;

        // wheel velocities
        double phi_dot_left_ = 0.0;
        double phi_dot_right_ = 0.0;

        // robot pose in the space frame
        Transform2D T_sb_{};

        // robot twist from the most recent call of FK
        Twist2D Vb_{};

        // bookkeeping variables
        Transform2D T_bl_;  // transform from body to left wheel
        Transform2D T_br_; // transform from body to right wheel
    };

}
#endif // DIFF_DRIVE_HPP