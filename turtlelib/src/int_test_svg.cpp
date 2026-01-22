/// @file
/// @brief Quick and dirty test of the Svg class

#include "turtlelib/svg.hpp"
#include <print>
#include <numbers>

int main() {
    using namespace turtlelib;

    // Create an Svg canvas with x bounds [-5, 5] and y bounds [-5, 5]
    Svg svg({-5.0, 5.0}, {-5.0, 5.0});

    // Draw 2 points
    auto p1_id = svg.draw(Point2D{1.0, 2.0}, "blue");
    auto p2_id = svg.draw(Point2D{-3.0, 1.5}, "green");

    // Draw 2 vectors
    auto v1_id = svg.draw(Vector2D{2.0, 3.0}, "pink");
    auto v2_id = svg.draw(Vector2D{-1.5, 2.5}, "purple");

    // Draw 2 coordinate frames
    auto f1_id = svg.draw(Transform2D{{0.0, 0.0}, 0.0}, "W");  // Identity at origin
    auto f2_id = svg.draw(Transform2D{{2.0, 1.0}, std::numbers::pi / 4}, "B");  // Frame rotated 45 degrees at (2, 1)

    // Write to file with default SvgSpec
    svg.write_file("test_output.svg");

    std::print("SVG file written to test_output.svg\n");

    return 0;
}