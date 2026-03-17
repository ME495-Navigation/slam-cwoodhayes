#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <vector>

#include "turtlelib/lidar.hpp"

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
