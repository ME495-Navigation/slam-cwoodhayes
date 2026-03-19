/// @file
/// @brief LiDAR point clustering and circle fitting utilities

#ifndef TURTLELIB_CIRCLE_FIT_HPP_INCLUDE_GUARD
#define TURTLELIB_CIRCLE_FIT_HPP_INCLUDE_GUARD

#include <tuple>
#include <vector>

#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
  /// @brief Cluster LiDAR range measurements into contiguous segments based on a distance threshold
  /// @param ranges The vector of LiDAR range measurements
  /// @param angle_increment The angular resolution of the LiDAR in radians
  /// @param distance_threshold The maximum allowed distance between consecutive points in a cluster
  /// @return A vector of clusters, where each cluster is a vector of (x, y) points in the robot frame
  std::vector<std::vector<Point2D>> cluster(
    const std::vector<double> & ranges,
    double angle_increment,
    double distance_threshold);

  /// @brief Fit a circle to a set of 2D points
  /// @note This follows the algorithm described here:
  ///   https://projecteuclid.org/euclid.ejs/1251119958
  ///   and summarized in the course notes here:
  ///   https://nu-msr.github.io/navigation/lectures/circle_fit.html
  /// @param points The vector of 2D points
  /// @return A tuple containing the fitted circle and the RMSE of the fit.
  std::tuple<turtlelib::Circle, double> fit_circle(const std::vector<Point2D> & points);

  /// @brief Configuration for cylinder detection
  struct DetectorConfig
  {
    /// @brief Maximum allowed distance between consecutive points in a cluster (meters)
    double distance_threshold = 0.01;
    /// @brief RMSE threshold for accepting circle fits
    double rmse_threshold = 0.1;
    /// @brief Range of mean inscribed angles to accept for circle detection (degrees)
    std::pair<double, double> inscribed_angle_mean_range_deg{90.0, 130.0};
    /// @brief Threshold for standard deviation of inscribed angles (degrees)
    double inscribed_angle_stddev_threshold_deg = 10.0;
    /// @brief Minimum number of points required in a cluster to attempt circle fitting
    int min_cluster_size = 3;
    /// @brief Threshold for concavity check based on mean point location relative to endpoints.
    double concavity_threshold = 0.02;  // meters
    /// @brief Expected range of circle radii for valid cylinder detections (meters)
    std::pair<double, double> circle_radius_range{0.001, 2.0}; // meters
  };

  class CylinderDetector
  {
  public:
    /// @brief Construct a new CylinderDetector object
    /// @param config Configuration parameters for cylinder detection
    explicit CylinderDetector(const DetectorConfig & config)
    : cfg_(config)
    {}

    /// @brief Detect cylindrical obstacles from LiDAR range measurements
    /// @param ranges The vector of LiDAR range measurements
    /// @param angle_increment The angular resolution of the LiDAR in radians
    /// @return A vector of detected circles representing cylindrical obstacles
    std::vector<turtlelib::Circle> detect(const std::vector<double> & ranges, double angle_increment);
  private:

    /// @brief Check if the mean inscribed angle of the points with respect to the arc endpoints
    /// is consistent across points
    bool inscribed_angle_check(const std::vector<Point2D> & points);

    DetectorConfig cfg_;

  };
};

#endif