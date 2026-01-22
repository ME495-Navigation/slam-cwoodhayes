/// \file
/// \brief UI-based demonstration of SVG generation + geometry library

#include "turtlelib/se2d.hpp"

#include <iostream>
#include <print>

using namespace std;


turtlelib::Transform2D prompt_for_tf(std::string tf_name) {
    double x, y, omega;

    auto example_tf = turtlelib::Transform2D({1, 2}, 3);
    auto example_fmt = std::format("{}", example_tf);
    turtlelib::Transform2D tf;

    cerr << "Please enter transform " << tf_name << ":\n";
    cerr << "Example format: " << example_fmt << "\n";
    cerr << "input here: ";
    cin >> tf;
    cin.ignore(10000, '\n');

    
    print(cerr, "You entered {}.\n\n", tf);
    return tf;
}

turtlelib::Point2D prompt_for_point(std::string point_name) {
    auto example_pt = turtlelib::Point2D{1.5, 2.5};
    auto example_fmt = std::format("{}", example_pt);
    turtlelib::Point2D pt;

    cerr << "Please enter point " << point_name << ":\n";
    cerr << "Example format: " << example_fmt << "\n";
    cerr << "input here: ";
    cin >> pt;
    cin.ignore(10000, '\n');

    print(cerr, "You entered {}.\n\n", pt);
    return pt;
}

int main() {
    auto Tab = prompt_for_tf("T_ab");
    auto Tbc = prompt_for_tf("T_bc");

    auto Tba = Tab.inv();
    auto Tcb = Tbc.inv();
    auto Tac = Tab * Tbc;
    auto Tca = Tac.inv();

    auto pa = prompt_for_point("p_a (in frame {{a}})");

    return 0;
}