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
    : noisy_robot(wheel_radius, track_width), clean_robot(wheel_radius, track_width), input_noise_stddev_(input_noise_stddev), slip_fraction_(slip_fraction)
  {
    rand_gen_ = std::mt19937(std::random_device{}());
  }

  void noisy_fk(double wheel_vel_left, double wheel_vel_right, double dt)
  {
    // add control noise to wheel velocities (but no noise if wheel isn't moving)
    auto prev_wheels = noisy_robot.get_wheel_angles();
    auto noisy_wheel_velocities = std::array<double*, 2>({&wheel_vel_left, &wheel_vel_right});
    auto noisy_wheel_angles = std::array<double, 2>({prev_wheels[0], prev_wheels[1]});
    for (size_t idx = 0; idx < noisy_wheel_velocities.size(); idx++)
    {
      auto vel_ptr = noisy_wheel_velocities[idx];
      if (*vel_ptr != 0.0)
      {
        std::normal_distribution<double> noise_dist(0.0, input_noise_stddev_);
        auto noise = noise_dist(rand_gen_);
        std::uniform_real_distribution<double> slip_dist(-slip_fraction_, slip_fraction_);
        auto slip = slip_dist(rand_gen_) * std::abs(*vel_ptr);
        *vel_ptr += noise + slip;

        noisy_wheel_angles[idx] = prev_wheels[idx] + *vel_ptr * dt;
      }
    }

    noisy_robot.forward_kinematics(noisy_wheel_angles[0], noisy_wheel_angles[1]);
    clean_robot.forward_kinematics(prev_wheels[0] + wheel_vel_left * dt, prev_wheels[1] + wheel_vel_right * dt);
    return;
  }

  turtlelib::DiffDrive noisy_robot;
  // the same commands without noise would produce this pose
  turtlelib::DiffDrive clean_robot; 
  double input_noise_stddev_;
  double slip_fraction_;
  std::mt19937 rand_gen_;
};

#endif // NOISE_MODELS_HPP