#include "turtlelib/se2d.hpp"
#include <utility>

namespace turtlelib
{
    class DiffDrive
    {
    public:
        DiffDrive(double wheel_radius, double wheel_track) : wheel_radius_(wheel_radius), wheel_track_(wheel_track) {};

        /// @brief Forward kinematics for a differential drive robot
        /// @param new_phi_left New wheel angle for the left wheel, as given by an encoder
        /// @param new_phi_right New wheel angle for the right wheel, as given by an encoder
        /// @return new pose of the robot in the space frame, as a Transform2D
        Transform2D forward_kinematics(double new_phi_left, double new_phi_right) const;

        /// @brief Inverse kinematics for a differential drive robot
        /// @param twist Desired twist of the robot in the space frame
        /// @return Left and right wheel velocities (phi_dot) required to achieve the desired twist
        std::pair<double, double> inverse_kinematics(const Twist2D& twist) const;

    private:
        const double wheel_radius_;
        const double wheel_track_;

        // wheel angles
        double phi_left_ = 0.0;
        double phi_right_ = 0.0;

        // robot pose in the space frame
        Transform2D T_sb{};
    };

}