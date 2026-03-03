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
    : clean_robot(wheel_radius, track_width), encoder_robot(wheel_radius, track_width), noisy_robot(wheel_radius, track_width), input_noise_stddev_(input_noise_stddev), slip_fraction_(slip_fraction)
  {
    rand_gen_ = std::mt19937(std::random_device{}());
  }

  void noisy_fk(double wheel_vel_left, double wheel_vel_right, double dt)
  {
    // Save unperturbed velocities for clean robot
    const auto clean_vel_left = wheel_vel_left;
    const auto clean_vel_right = wheel_vel_right;

    // Apply noise to velocities (for encoder readings and actual motion)
    auto noisy_vel_left = wheel_vel_left;
    auto noisy_vel_right = wheel_vel_right;
    
    if (wheel_vel_left != 0.0)
    {
      std::normal_distribution<double> noise_dist(0.0, input_noise_stddev_);
      auto noise = noise_dist(rand_gen_);
      noisy_vel_left += noise;
    }
    if (wheel_vel_right != 0.0)
    {
      std::normal_distribution<double> noise_dist(0.0, input_noise_stddev_);
      auto noise = noise_dist(rand_gen_);
      noisy_vel_right += noise;
    }

    // Update clean robot (no noise, no slip)
    auto clean_prev_wheels = clean_robot.get_wheel_angles();
    clean_robot.forward_kinematics(
      clean_prev_wheels[0] + clean_vel_left * dt,
      clean_prev_wheels[1] + clean_vel_right * dt);

    // Update encoder robot (noise only, no slip)
    auto encoder_prev_wheels = encoder_robot.get_wheel_angles();
    encoder_robot.forward_kinematics(
      encoder_prev_wheels[0] + noisy_vel_left * dt,
      encoder_prev_wheels[1] + noisy_vel_right * dt);

    // Apply slip to noisy velocities for actual motion
    std::uniform_real_distribution<double> slip_dist(-slip_fraction_, slip_fraction_);
    auto slip_left = slip_dist(rand_gen_) * std::abs(noisy_vel_left);
    auto slip_right = slip_dist(rand_gen_) * std::abs(noisy_vel_right);
    auto slip_vel_left = noisy_vel_left + slip_left;
    auto slip_vel_right = noisy_vel_right + slip_right;

    // Update noisy robot (noise + slip for actual ground truth pose)
    auto noisy_prev_wheels = noisy_robot.get_wheel_angles();
    noisy_robot.forward_kinematics(
      noisy_prev_wheels[0] + slip_vel_left * dt,
      noisy_prev_wheels[1] + slip_vel_right * dt);
  }

  // no noise, no slip (just for debugging)
  turtlelib::DiffDrive clean_robot;
  // noise only (what encoders read)
  turtlelib::DiffDrive encoder_robot;    
  // noise + slip (actual motion)
  turtlelib::DiffDrive noisy_robot;

  double input_noise_stddev_;
  double slip_fraction_;
  std::mt19937 rand_gen_;
};

#endif // NOISE_MODELS_HPP