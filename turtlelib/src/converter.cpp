/// \brief Quick integration test program for angle conversion code

#include "turtlelib/angle.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <print>
#include <numbers>

int main() {

    auto rad = turtlelib::normalize_angle(-5.5 * std::numbers::pi);
    std::print("rad = {}pi", rad / std::numbers::pi);
}