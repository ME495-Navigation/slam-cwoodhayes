#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <vector>

#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/obstacles.hpp"

using namespace turtlelib;
using namespace Catch::Matchers;

TEST_CASE("Obstacles no collision - robot far from obstacle", "[Obstacles]")
{
    auto obs = Obstacles{{1.0}, {1.0}, 0.1};
    const auto collision_radius = 0.11;
    
    // Robot at origin, obstacle at (1.0, 1.0) with combined radius 0.21
    // Distance is sqrt(2) ≈ 1.414, which is > 0.21, so no collision
    auto pose = Transform2D{{0.0, 0.0}, 0.0};
    auto result = obs.collide(pose, collision_radius);
    
    REQUIRE_THAT(result.translation().x, WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.translation().y, WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.rotation(), WithinAbs(0.0, 1e-6));
    REQUIRE(obs.did_collide() == false);
}

TEST_CASE("Obstacles collision - robot overlapping obstacle", "[Obstacles]")
{
    auto obs = Obstacles{{1.0}, {0.0}, 0.1};
    const auto collision_radius = 0.11;
    
    // Robot at (0.95, 0.0), obstacle at (1.0, 0.0)
    // Distance is 0.05, which is < 0.21 (collision!)
    // Should push robot to x = 1.0 - 0.21 = 0.79
    auto pose = Transform2D{{0.95, 0.0}, 0.0};
    auto result = obs.collide(pose, collision_radius);
    
    REQUIRE_THAT(result.translation().x, WithinAbs(0.79, 1e-6));
    REQUIRE_THAT(result.translation().y, WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.rotation(), WithinAbs(0.0, 1e-6));
    REQUIRE(obs.did_collide() == true);
}

TEST_CASE("Obstacles collision - robot at angle", "[Obstacles]")
{
    auto obs = Obstacles{{1.0}, {1.0}, 0.1};
    const auto collision_radius = 0.11;
    
    // Robot at (0.95, 0.95), obstacle at (1.0, 1.0)
    // Distance is sqrt(2 * 0.05^2) ≈ 0.0707, which is < 0.21 (collision!)
    // Should push robot away along the line from obstacle to robot
    auto pose = Transform2D{{0.95, 0.95}, 0.5};
    auto result = obs.collide(pose, collision_radius);
    
    // Direction vector from obstacle to robot: (-0.05, -0.05), normalized: (-1/sqrt(2), -1/sqrt(2))
    // New position: (1.0, 1.0) - 0.21 * (-1/sqrt(2), -1/sqrt(2)) 
    //             = (1.0, 1.0) - (-0.1485, -0.1485)
    //             = (0.8515, 0.8515)
    const auto expected_offset = 0.21 / std::sqrt(2.0);
    REQUIRE_THAT(result.translation().x, WithinAbs(1.0 - expected_offset, 1e-3));
    REQUIRE_THAT(result.translation().y, WithinAbs(1.0 - expected_offset, 1e-3));
    REQUIRE_THAT(result.rotation(), WithinAbs(0.5, 1e-6));  // Rotation unchanged
    REQUIRE(obs.did_collide() == true);
}

TEST_CASE("Obstacles multiple obstacles - one collision", "[Obstacles]")
{
    auto obs = Obstacles{{1.0, 5.0}, {0.0, 0.0}, 0.1};
    const auto collision_radius = 0.11;
    
    // Robot at (0.95, 0.0), close to first obstacle at (1.0, 0.0)
    // Far from second obstacle at (5.0, 0.0)
    auto pose = Transform2D{{0.95, 0.0}, 0.0};
    auto result = obs.collide(pose, collision_radius);
    
    REQUIRE_THAT(result.translation().x, WithinAbs(0.79, 1e-6));
    REQUIRE_THAT(result.translation().y, WithinAbs(0.0, 1e-6));
    REQUIRE(obs.did_collide() == true);
}

TEST_CASE("Obstacles multiple obstacles - no collision", "[Obstacles]")
{
    auto obs = Obstacles{{1.0, -1.0, 0.0}, {1.0, 1.0, 5.0}, 0.1};
    const auto collision_radius = 0.11;
    
    // Robot at origin, all obstacles far enough away
    auto pose = Transform2D{{0.0, 0.0}, 0.0};
    auto result = obs.collide(pose, collision_radius);
    
    REQUIRE_THAT(result.translation().x, WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.translation().y, WithinAbs(0.0, 1e-6));
    REQUIRE(obs.did_collide() == false);
}

TEST_CASE("Obstacles edge case - robot exactly at boundary", "[Obstacles]")
{
    auto obs = Obstacles{{1.0}, {0.0}, 0.1};
    const auto collision_radius = 0.11;
    
    // Place robot just outside collision boundary (distance slightly > 0.21)
    auto pose = Transform2D{{0.78, 0.0}, 0.0};
    auto result = obs.collide(pose, collision_radius);
    
    // No collision since distance is greater than radius sum
    REQUIRE_THAT(result.translation().x, WithinAbs(0.78, 1e-6));
    REQUIRE_THAT(result.translation().y, WithinAbs(0.0, 1e-6));
    REQUIRE(obs.did_collide() == false);
}

TEST_CASE("Obstacles did_collide state tracking", "[Obstacles]")
{
    auto obs = Obstacles{{1.0}, {0.0}, 0.1};
    const auto collision_radius = 0.11;
    
    // First check - collision
    auto pose1 = Transform2D{{0.95, 0.0}, 0.0};
    obs.collide(pose1, collision_radius);
    REQUIRE(obs.did_collide() == true);
    
    // Second check - no collision (should reset flag)
    auto pose2 = Transform2D{{0.0, 0.0}, 0.0};
    obs.collide(pose2, collision_radius);
    REQUIRE(obs.did_collide() == false);
}

TEST_CASE("Obstacles empty obstacle list", "[Obstacles]")
{
    auto obs = Obstacles{{}, {}, 0.1};
    const auto collision_radius = 0.11;
    
    // No obstacles, so no collision possible
    auto pose = Transform2D{{0.5, 0.5}, 1.0};
    auto result = obs.collide(pose, collision_radius);
    
    REQUIRE_THAT(result.translation().x, WithinAbs(0.5, 1e-6));
    REQUIRE_THAT(result.translation().y, WithinAbs(0.5, 1e-6));
    REQUIRE_THAT(result.rotation(), WithinAbs(1.0, 1e-6));
    REQUIRE(obs.did_collide() == false);
}
