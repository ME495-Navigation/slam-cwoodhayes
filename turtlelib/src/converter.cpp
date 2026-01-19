/// \brief Quick integration test program for angle conversion code

#include "turtlelib/angle.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <print>
#include <numbers>

int main() {
    while (true) {
        std::print("Enter an angle: <angle> <deg|rad>, (CTRL-D to exit)\n");
        double angle;
        double converted_angle;
        std::string unit;
        std::string other_unit;

        if (!(std::cin >> angle >> unit)) {
            std::print("Exiting.\n");
            break;
        }
        if (unit == "deg") {
            converted_angle = turtlelib::deg2rad(angle);
            other_unit = "rad";
        }
        else if (unit == "rad") {
            converted_angle = turtlelib::rad2deg(angle);
            other_unit = "deg";
        }
        else {
            std::print("Invalid input: please enter <angle> <deg|rad>, (CTRL-D to exit)");
        }

        std::print("{} {} is {} {}.\n", angle, unit, converted_angle, other_unit);
    }

    return 0;
}