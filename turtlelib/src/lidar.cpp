

#include "turtlelib/lidar.hpp"
#include <cmath>


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

  double Lidar::ray_trace(const Transform2D& pose, double angle, const Obstacles& obstacles) const
  {
    Vector2D ray_dir{std::cos(pose.rotation() + angle), std::sin(pose.rotation() + angle)};
    
    double closest_dist = max_range_;
    
    for (size_t i = 0; i < obstacles.x.size(); ++i) {
      Vector2D obs_pos{obstacles.x[i], obstacles.y[i]};
      Vector2D to_obs = obs_pos - pose.translation();
      double proj_length = to_obs.x * ray_dir.x + to_obs.y * ray_dir.y;
      
      if (proj_length > 0 && proj_length < max_range_) {
        Vector2D closest_point = pose.translation() + proj_length * ray_dir;
        double dist_to_obs = magnitude(obs_pos - closest_point);
        
        if (dist_to_obs < obstacles.r) {
          // Obstacle intersects the ray
          double intersection_dist = proj_length - std::sqrt(obstacles.r * obstacles.r - dist_to_obs * dist_to_obs);
          closest_dist = std::min(closest_dist, std::max(min_range_, intersection_dist));
        }
      }
    }
    
    return closest_dist;
  }

  std::vector<double> Lidar::simulate_scan(const Transform2D& pose, const Obstacles& obstacles)
  {
    std::vector<double> ranges;
    for (double angle = 0.0; angle < 2 * M_PI; angle += angle_increment_) {
      double range = ray_trace(pose, angle, obstacles);
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