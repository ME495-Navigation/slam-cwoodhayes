

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
    Vector2D ray_origin = pose.translation();
    Vector2D ray_dir{std::cos(pose.rotation() + angle), std::sin(pose.rotation() + angle)};
    
    double closest_dist = max_range_;
    
    for (size_t i = 0; i < obstacles.x.size(); ++i) {
      Vector2D obs_center{obstacles.x[i], obstacles.y[i]};
      
      /////////////////////////////// Begin_Citation[15] ///////////////////////////////
      // Circle-line intersection referenced from:
      // https://mathworld.wolfram.com/Circle-LineIntersection.html
      // circle center to robot
      Vector2D u = ray_origin - obs_center;  
      
      // Quadratic coefficients for: |u + t*d|² = r²
      // Expands to: (d·d)t² + 2(u·d)t + (u·u - r²) = 0
      // Since ray_dir is unit vector (cos² + sin² = 1), d·d = 1
      double a = 1.0;  // ray_dir is unit vector
      double b = 2.0 * (u.x * ray_dir.x + u.y * ray_dir.y);
      double c = (u.x * u.x + u.y * u.y) - (obstacles.r * obstacles.r);
      
      double discriminant = b * b - 4.0 * a * c;
      
      if (discriminant >= 0.0) {
        double sqrt_disc = std::sqrt(discriminant);
        
        // Two solutions: t = (-b ± √Δ) / 2
        double t1 = (-b - sqrt_disc) / 2.0;
        double t2 = (-b + sqrt_disc) / 2.0;
        
        // We want the smallest positive t (first intersection along the ray)
        double t = -1.0;
        if (t1 > 0.0) {
          t = t1;
        } else if (t2 > 0.0) {
          t = t2;
        }
        
        // Update closest intersection if it's within range
        if (t > 0.0 && t < closest_dist && t < max_range_) {
          closest_dist = t;
        }
      }
    }
    
    return std::max(min_range_, closest_dist);
    ///////////////////////////////// End_Citation[15] ///////////////////////////////
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