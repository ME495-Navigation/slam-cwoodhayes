/// @file
/// @brief LiDAR simulation and related utilities

#ifndef LIDAR_HPP
#define LIDAR_HPP

#include <vector>
#include <random>
#include <cmath>
#include "turtlelib/se2d.hpp"
#include "turtlelib/obstacles.hpp"

namespace turtlelib
{

  class Lidar
  {
  public:
    /// @brief Construct a new Lidar object
    /// @param min_range Minimum range of the LiDAR in meters
    /// @param max_range Maximum range of the LiDAR in meters
    /// @param angle_increment Angular resolution of the LiDAR in radians
    /// @param resolution Range resolution of the LiDAR in meters
    /// @param noise_variance Variance of zero-mean Gaussian noise to add to range measurements
    /// @note We assume that the LiDAR sweeps 360deg around.
    Lidar(double min_range, double max_range, double angle_increment, double resolution, double noise_variance);

    /// @brief Simulate a LiDAR scan from a given robot pose and set of obstacles
    /// @param pose The robot's pose in the world frame
    /// @param obstacles The obstacles in the environment
    /// @return A vector of range measurements corresponding to each angle increment, with added Gaussian noise
    std::vector<double> simulate_scan(const Transform2D& pose, const Obstacles& obstacles);

    /// @brief Get the minimum range of the LiDAR
    /// @return Minimum range in meters
    double get_min_range() const { return min_range_; }

    /// @brief Get the maximum range of the LiDAR
    /// @return Maximum range in meters
    double get_max_range() const { return max_range_; }

    /// @brief Get the angular resolution of the LiDAR
    /// @return Angular increment in radians
    double get_angle_increment() const { return angle_increment_; }

    /// @brief Get the starting angle of the LiDAR sweep
    /// @return Minimum angle in radians (-π for 360° sweep)
    constexpr double get_angle_min() const { return -M_PI; }

    /// @brief Get the ending angle of the LiDAR sweep
    /// @return Maximum angle in radians (π for 360° sweep)
    constexpr double get_angle_max() const { return M_PI; }

  private:
    /// @brief Find the first point along a ray from the robot's pose that intersects an obstacle
    /// @param pose The robot's pose in the world frame
    /// @param angle The angle of the ray relative to the robot's heading
    /// @param obstacles The obstacles in the environment
    /// @return The distance to the intersection, or max_range if no intersection
    double ray_trace(const Transform2D& pose, double angle, const Obstacles& obstacles) const;

    double min_range_;
    double max_range_;
    double angle_increment_;
    double resolution_;
    double noise_variance_;

    // Random number generator for noise
    std::normal_distribution<double> noise_dist_;
    std::default_random_engine rand_gen_;
  };
}

#endif // LIDAR_HPP
