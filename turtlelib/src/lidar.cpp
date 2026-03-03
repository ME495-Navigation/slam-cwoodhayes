

#include "turtlelib/lidar.hpp"


namespace turtlelib
{

  Lidar::Lidar(double min_range, double max_range, double angle_increment, double resolution, double noise_variance):
    min_range_(min_range), 
    max_range_(max_range), 
    angle_increment_(angle_increment), 
    resolution_(resolution), 
    noise_variance_(noise_variance),
    noise_dist_(0.0, std::sqrt(noise_variance)), 
    rand_gen_(std::random_device{}())
    {}

  std::vector<double> Lidar::simulate_scan(const Transform2D& pose, const Obstacles& obstacles)
  {
    std::vector<double> ranges;
    for (double angle = 0.0; angle < 2 * M_PI; angle += angle_increment_) {
      double range = magnitude(obstacles.ray_trace(pose, angle, max_range_));
      // add noise then clamp
      range += noise_dist_(rand_gen_);
      if (range < min_range_) {
        range = min_range_;
      } else if (range > max_range_) {
        range = max_range_;
      }
      ranges.push_back(range);
    }
    return ranges;
  }

}