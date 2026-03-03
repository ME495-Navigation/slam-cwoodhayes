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

private:
  bool did_collide_ = false;
};

#endif // OBSTACLES_HPP