#include "turtlelib/circle_fit.hpp"

#include <armadillo>
#include <cmath>
#include <limits>
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
    // This follows the algorithm described here:
    // https://projecteuclid.org/euclid.ejs/1251119958
    // and summarized in the course notes here:
    // https://nu-msr.github.io/navigation/lectures/circle_fit.html

    const auto n = points.size();
    if (n < 3) {
      throw std::runtime_error("At least 3 points are required to fit a circle");
    }

    auto x_sum = 0.0;
    auto y_sum = 0.0;
    for (const auto & point : points) {
      x_sum += point.x;
      y_sum += point.y;
    }

    const auto x_mean = x_sum / static_cast<double>(n);
    const auto y_mean = y_sum / static_cast<double>(n);

    auto Z = arma::mat(n, 4, arma::fill::zeros);
    auto z_sum = 0.0;

    for (size_t i = 0; i < n; ++i) {
      const auto x = points[i].x - x_mean;
      const auto y = points[i].y - y_mean;
      const auto z = x * x + y * y;

      Z(i, 0) = z;
      Z(i, 1) = x;
      Z(i, 2) = y;
      Z(i, 3) = 1.0;

      z_sum += z;
    }

    const auto z_mean = z_sum / static_cast<double>(n);

    auto H_inv = arma::mat(4, 4, arma::fill::zeros);
    H_inv(0, 3) = 0.5;
    H_inv(1, 1) = 1.0;
    H_inv(2, 2) = 1.0;
    H_inv(3, 0) = 0.5;
    H_inv(3, 3) = -2.0 * z_mean;

    auto U = arma::mat{};
    auto s = arma::vec{};
    auto V = arma::mat{};
    const auto svd_ok = arma::svd(U, s, V, Z);
    if (!svd_ok || s.n_elem < 4) {
      throw std::runtime_error("SVD failed during circle fit");
    }

    auto A = arma::vec(4, arma::fill::zeros);
    if (s(3) < 1e-12) {
      A = V.col(3);
    } else {
      const arma::mat S = arma::diagmat(s);
      const arma::mat Y = V * S * V.t();
      const arma::mat Q = Y * H_inv * Y;

      auto eigenvalues = arma::vec{};
      auto eigenvectors = arma::mat{};
      const auto eig_ok = arma::eig_sym(eigenvalues, eigenvectors, Q);
      if (!eig_ok || eigenvalues.n_elem == 0) {
        throw std::runtime_error("Eigen decomposition failed during circle fit");
      }

      auto smallest_positive = std::numeric_limits<double>::infinity();
      auto smallest_positive_idx = arma::uword{0};
      auto found_positive = false;
      for (arma::uword i = 0; i < eigenvalues.n_elem; ++i) {
        if (eigenvalues(i) > 0.0 && eigenvalues(i) < smallest_positive) {
          smallest_positive = eigenvalues(i);
          smallest_positive_idx = i;
          found_positive = true;
        }
      }

      if (!found_positive) {
        throw std::runtime_error("No positive eigenvalue found during circle fit");
      }

      const auto A_star = eigenvectors.col(smallest_positive_idx);
      const auto solve_ok = arma::solve(A, Y, A_star);
      if (!solve_ok) {
        throw std::runtime_error("Linear solve failed during circle fit");
      }
    }

    if (std::abs(A(0)) < 1e-15) {
      throw std::runtime_error("Degenerate circle fit: near-zero quadratic term");
    }

    const auto a = -A(1) / (2.0 * A(0));
    const auto b = -A(2) / (2.0 * A(0));
    auto radius_sq = (A(1) * A(1) + A(2) * A(2) - 4.0 * A(0) * A(3)) / (4.0 * A(0) * A(0));
    if (radius_sq < 0.0 && radius_sq > -1e-12) {
      radius_sq = 0.0;
    }
    if (radius_sq < 0.0) {
      throw std::runtime_error("Degenerate circle fit: negative radius squared");
    }

    const auto sigma_sq = [&]() {
      auto sum = 0.0;
      for (const auto & point : points) {
        const auto x = point.x - x_mean;
        const auto y = point.y - y_mean;
        const auto residual = (x - a) * (x - a) + (y - b) * (y - b) - radius_sq;
        sum += residual * residual;
      }
      return sum / static_cast<double>(n);
    }();

    return {
      Circle{Point2D{a + x_mean, b + y_mean}, std::sqrt(radius_sq)},
      std::sqrt(sigma_sq)
    };
  }

  std::vector<Circle> CylinderDetector::detect(const std::vector<double> & ranges, double angle_increment)
  {
    const auto clusters = cluster(ranges, angle_increment, distance_threshold_);

    auto circles = std::vector<Circle>{};
    for (const auto & cluster : clusters) {
      if (cluster.size() >= 3) {
        try {
          const auto [circle, rmse] = fit_circle(cluster);

          // TODO filter out bad fits using rmse + matt's algo
          circles.push_back(circle);
        } catch (const std::exception & e) {
          // Circle fit failed for this cluster; skip it.
          continue;
        }
      }
    }

    return circles;
  }
}
