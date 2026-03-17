#include "turtlelib/circle_fit.hpp"

#include <cmath>
#include <stdexcept>

namespace turtlelib
{
  std::vector<std::vector<Point2D>> cluster(
    const std::vector<double> & ranges,
    double angle_increment,
    double distance_threshold)
  {
    auto clusters = std::vector<std::vector<Point2D>>{};
    auto current_cluster = std::vector<Point2D>{};

    for (size_t i = 0; i < ranges.size(); ++i) {
      const auto range = ranges[i];
      const auto angle = static_cast<double>(i) * angle_increment;
      const auto point = Point2D{range * std::cos(angle), range * std::sin(angle)};

      if (current_cluster.empty()) {
        current_cluster.push_back(point);
      } else {
        const auto & last_point = current_cluster.back();
        const auto dist = std::hypot(point.x - last_point.x, point.y - last_point.y);
        if (dist <= distance_threshold) {
          current_cluster.push_back(point);
        } else {
          clusters.push_back(current_cluster);
          current_cluster.clear();
          current_cluster.push_back(point);
        }
      }
    }

    if (!current_cluster.empty()) {
      // If this cluster wraps around, merge it with the first cluster.
      if (!clusters.empty()) {
        const auto & first_point = clusters.front().front();
        const auto & last_point = current_cluster.back();
        const auto dist = std::hypot(first_point.x - last_point.x, first_point.y - last_point.y);
        if (dist <= distance_threshold) {
          clusters.front().insert(
            clusters.front().begin(), current_cluster.begin(), current_cluster.end());
        } else {
          clusters.push_back(current_cluster);
        }
      } else {
        clusters.push_back(current_cluster);
      }
    }

    return clusters;
  }

  std::tuple<turtlelib::Circle, double> fit_circle(const std::vector<Point2D> & points)
  {
    const auto n = points.size();
    if (n < 3) {
      throw std::runtime_error("At least 3 points are required to fit a circle");
    }

    // TODO: implement Pratt/Taubin style circle fit.
    return {Circle{}, 0.0};
  }
}
