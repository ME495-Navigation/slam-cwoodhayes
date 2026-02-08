#include "turtlelib/se2d.hpp"
#include <utility>
#include <vector>

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

    private:
        const double wheel_radius_;
        const double wheel_track_;

        // wheel angles
        double phi_left_ = 0.0;
        double phi_right_ = 0.0;

        // wheel velocities
        double phi_dot_left_ = 0.0;
        double phi_dot_right_ = 0.0;

        // robot pose in the space frame
        Transform2D T_sb_{};

        // bookkeeping variables
        Transform2D T_bl_;  // transform from body to left wheel
        Transform2D T_br_; // transform from body to right wheel
    };

}