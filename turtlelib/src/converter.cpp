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
        constexpr std::string_view errstr = "Invalid input: please enter <angle> <deg|rad>, (CTRL-D to exit)\n";

        if (!(std::cin >> angle >> unit)) {
            if (std::cin.eof()) {
                std::print("Exiting.\n");
                break;
            } 
            else {
                std::print(errstr);
                std::cin.clear();
                std::cin.ignore(10000, '\n');
            }
            continue;
        }
        if (unit == "deg") {
            converted_angle = turtlelib::normalize_angle(turtlelib::deg2rad(angle));
            other_unit = "rad";
        }
        else if (unit == "rad") {
            converted_angle = turtlelib::rad2deg(turtlelib::normalize_angle(angle));
            other_unit = "deg";
        }
        else {
            std::print(errstr);
            continue;
        }

        std::print("{} {} is {} {}.\n", angle, unit, converted_angle, other_unit);
    }

    return 0;
}