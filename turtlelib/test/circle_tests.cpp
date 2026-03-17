#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <numbers>
#include <vector>

#include "turtlelib/circle_fit.hpp"

using namespace turtlelib;
using namespace Catch::Matchers;

TEST_CASE("cluster splits points by distance threshold", "[lidar][cluster]")
{
  const auto ranges = std::vector<double>{1.0, 1.0, 5.0, 5.0};
  const auto angle_increment = 0.01;
  const auto distance_threshold = 0.2;

  const auto clusters = cluster(ranges, angle_increment, distance_threshold);

  REQUIRE(clusters.size() == 2);
  REQUIRE(clusters[0].size() == 2);
  REQUIRE(clusters[1].size() == 2);

  // First point should be near (1, 0)
  REQUIRE_THAT(clusters[0][0].x, WithinAbs(1.0, 1e-9));
  REQUIRE_THAT(clusters[0][0].y, WithinAbs(0.0, 1e-9));
}

TEST_CASE("cluster merges wrap-around endpoints", "[lidar][cluster]")
{
  // Points at index 0 and 3 are both at range 1.0 and should be spatially close
  // when angle_increment is ~120deg, causing wrap-around merge.
  const auto ranges = std::vector<double>{1.0, 5.0, 5.0, 1.0};
  const auto angle_increment = 2.0 * std::numbers::pi / 3.0;
  const auto distance_threshold = 0.5;

  const auto clusters = cluster(ranges, angle_increment, distance_threshold);

  // Wrap-around merge combines the last and first singleton clusters,
  // while the two far points remain separate clusters.
  REQUIRE(clusters.size() == 3);
  REQUIRE(clusters[0].size() == 2);
  REQUIRE(clusters[1].size() == 1);
  REQUIRE(clusters[2].size() == 1);
}
