/// @file
/// @brief logic for handling collisions with obstacles

#ifndef OBSTACLES_HPP
#define OBSTACLES_HPP
#include <vector>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"

class Obstacles {
public:
  std::vector<double> x;
  std::vector<double> y;
  double r;

  /// @brief Construct an Obstacles object
  /// @param x_ x coordinates of obstacles
  /// @param y_ y coordinates of obstacles
  /// @param r_ radius of obstacles
  Obstacles(std::vector<double> x_, std::vector<double> y_, double r_)
    : x(std::move(x_)), y(std::move(y_)), r(r_) {}

  /// @brief Check if a given pose collides with any of the obstacles, given a collision radius for the robot.
  /// @param pose The proposed robot pose
  /// @param collision_radius The collision radius of the robot
  /// @return new robot pose adjusted to be outside the obstacle if there is a collision, otherwise returns the original pose
  turtlelib::Transform2D collide(turtlelib::Transform2D pose, double collision_radius)
  {
    for (size_t i = 0; i < x.size(); ++i) {
      turtlelib::Vector2D obs_pos{x[i], y[i]};
      auto r_vec = obs_pos - pose.translation();
      double dist = turtlelib::magnitude(r_vec);
      if (dist < r + collision_radius) {
        // Compute the line between the robot center and the obstacle center
        // Move the robot's center along this line so that the collision circles are tangent
        did_collide_ = true;
        turtlelib::Vector2D new_pos = obs_pos - (r + collision_radius) * turtlelib::normalize(r_vec);
        return turtlelib::Transform2D{new_pos, pose.rotation()};
      }
    }
    did_collide_ = false;
    return pose;
  }

  /// @brief Check if the most recent call to collide resulted in a collision
  bool did_collide() const { return did_collide_; }

  /// @brief Find the first point along a ray from the robot's pose that intersects at an obstacle.
  /// @param pose The robot's pose
  /// @param angle The angle of the ray relative to the robot's heading
  /// @param max_range The maximum range of the ray
  /// @return The point where the ray intersects an obstacle, or the end point if no intersection occurs
  turtlelib::Vector2D ray_trace(turtlelib::Transform2D pose, double angle, double max_range) const
  {
    turtlelib::Vector2D ray_dir{std::cos(pose.rotation() + angle), std::sin(pose.rotation() + angle)};
    turtlelib::Vector2D ray_end = pose.translation() + max_range * ray_dir;

    for (size_t i = 0; i < x.size(); ++i) {
      turtlelib::Vector2D obs_pos{x[i], y[i]};
      turtlelib::Vector2D to_obs = obs_pos - pose.translation();
      double proj_length = to_obs.x * ray_dir.x + to_obs.y * ray_dir.y;
      if (proj_length > 0 && proj_length < max_range) {
        turtlelib::Vector2D closest_point = pose.translation() + proj_length * ray_dir;
        double dist_to_obs = turtlelib::magnitude(obs_pos - closest_point);
        if (dist_to_obs < r) {
          // Obstacle intersects the ray, return the point of intersection
          return {closest_point.x, closest_point.y};
        }
      }
    }
    // No obstacle intersected the ray, return the end point
    return ray_end;
  }

private:
  bool did_collide_ = false;
};

#endif // OBSTACLES_HPP