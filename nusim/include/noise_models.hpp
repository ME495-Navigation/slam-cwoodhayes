/// @file
/// @brief helper functions & classes for adding noise to simulated objects

#ifndef NOISE_MODELS_HPP
#define NOISE_MODELS_HPP
#include "turtlelib/diff_drive.hpp"
#include <random>

/// @brief A wrapper around DiffDrive that adds noise to the commanded wheel velocities & movements, for use in the NUSimulator node.
class NoisyDiffDrive
{
public:
  NoisyDiffDrive(double wheel_radius, double track_width, double input_noise_stddev, double slip_fraction)
    : robot(wheel_radius, track_width), input_noise_stddev_(input_noise_stddev), slip_fraction_(slip_fraction)
  {
    rand_gen_ = std::mt19937(std::random_device{}());
  }

  std::pair<double, double> noisy_fk(double wheel_vel_left, double wheel_vel_right, double dt)
  {
    // add control noise to wheel velocities (but no noise if wheel isn't moving)
    if (wheel_vel_left != 0.0)
    {
      // add gaussian noise
      std::normal_distribution<double> noise_dist(0.0, input_noise_stddev_);
      double noise = noise_dist(rand_gen_);
      // add slip noise
      std::uniform_real_distribution<double> slip_dist(-slip_fraction_, slip_fraction_);
      double slip = slip_dist(rand_gen_) * std::abs(wheel_vel_left);
      wheel_vel_left += noise + slip;
    }
    if (wheel_vel_right != 0.0)
    {
      // add gaussian noise
      std::normal_distribution<double> noise_dist(0.0, input_noise_stddev_);
      double noise = noise_dist(rand_gen_);
      // add slip noise
      std::uniform_real_distribution<double> slip_dist(-slip_fraction_, slip_fraction_);
      double slip = slip_dist(rand_gen_) * std::abs(wheel_vel_right);
      wheel_vel_right += noise + slip;
    }

    auto prev_wheels = robot.get_wheel_angles();
    auto new_wheel_angle_left = prev_wheels[0] + wheel_vel_left * dt;
    auto new_wheel_angle_right = prev_wheels[1] + wheel_vel_right * dt;
    robot.forward_kinematics(new_wheel_angle_left, new_wheel_angle_right);
    return { new_wheel_angle_left, new_wheel_angle_right };
  }

  turtlelib::DiffDrive robot;
  double input_noise_stddev_;
  double slip_fraction_;
  std::mt19937 rand_gen_;
};

#endif // NOISE_MODELS_HPP