/// \file
/// \brief UI-based demonstration of SVG generation + geometry library

#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"

#include <iostream>
#include <print>

using namespace std;


turtlelib::Transform2D prompt_for_tf(std::string tf_name) {
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

turtlelib::Vector2D prompt_for_vector(std::string vector_name) {
    auto example_vec = turtlelib::Vector2D{1.5, 2.5};
    auto example_fmt = std::format("{}", example_vec);
    turtlelib::Vector2D vec;

    cerr << "Please enter vector " << vector_name << ":\n";
    cerr << "Example format: " << example_fmt << "\n";
    cerr << "input here: ";
    cin >> vec;
    cin.ignore(10000, '\n');

    print(cerr, "You entered {}.\n\n", vec);
    return vec;
}

/// @brief normalize a vector (not part of the Vector2D class due to homework's API requirements)
/// @param v vector
/// @return normalized vector
turtlelib::Vector2D normalize_vec(const turtlelib::Vector2D& v) {
    auto len = sqrt(v.x * v.x + v.y * v.y);
    return turtlelib::Vector2D({v.x / len, v.y / len});
}

int main() {
    /*
    Create a file called frame_main.cpp, which will compile into an executable called frame_main. Here is what the program should do:

    Prompt the user to enter two transforms: T_ab and T_bc
    .
    Compute and output Tab, Tba, Tbc, Tcb, Tac, and Tca and draw each frame 
    in the svg file (with frame {a} located at (0, 0)).
    Prompt the user to enter a point pa in Frame {a}
    Compute pa's location in frames b and c and and output the locations of all 3 points
    Use purple to draw pa, brown to draw pb, and orange to draw pc
    Prompt the user to enter a vector vb in frame b

    Normalize the vector to form vb_hat
    Draw vb_hat with the tail located at in frame b, in brown.
    Draw vbhat with tail located at in frame c, in black.

    Output vb expressed in frame and frame coordinates
        Draw  va with the tail at in frame b, in purple.
    Draw va with the tail at in frame c, in orange

    Output the drawing to /tmp/frames.svg.
    All outputs that are there to prompt the user should be written to stderr
    All outputs that are in response to a calculation should be written to stdout
     */
    auto Tab = prompt_for_tf("T_ab");
    auto Tbc = prompt_for_tf("T_bc");

    auto Tba = Tab.inv();
    auto Tcb = Tbc.inv();
    auto Tac = Tab * Tbc;
    auto Tca = Tac.inv();

    // Output transform information
    print("Tab: {}\n", Tab);
    print("Tba: {}\n", Tba);
    print("Tbc: {}\n", Tbc);
    print("Tcb: {}\n", Tcb);
    print("Tac: {}\n", Tac);
    print("Tca: {}\n\n", Tca);

    // Create SVG and draw frames
    turtlelib::Svg svg({-5.0, 5.0}, {-5.0, 5.0});
    svg.draw(turtlelib::Transform2D({0.0, 0.0}, 0.0), "a");
    svg.draw(Tab, "b");
    svg.draw(Tac, "c");

    // Prompt for point in frame a
    auto pa = prompt_for_point("p_a (in frame {a})");
    
    // Transform point to frames b and c
    auto pb = Tba(pa);
    auto pc = Tca(pa);

    print("p_a: {}\n", pa);
    print("p_b: {}\n", pb);
    print("p_c: {}\n\n", pc);

    // Draw the points
    svg.draw(pa, "purple");
    svg.draw(pb, "brown");
    svg.draw(pc, "orange");

    // Prompt for vector in frame b
    auto vb = prompt_for_vector("v_b (in frame {b})");

    // Normalize vector
    auto vb_hat = normalize_vec(vb);

    // Draw vectors with tails at other frame origins
    // vb_hat with tail at b
    svg.draw_in_frame(vb_hat, Tab, "brown");
    // vb with tail at b
    svg.draw_in_frame(vb, Tab, "black");

    // Transform vector to other frames, output, and draw
    auto va = Tab(vb);
    auto vc = Tcb(vb);
    print("v_a: {}\n", va);
    print("v_c: {}\n\n", vc);
    svg.draw(va, "purple");
    svg.draw_in_frame(vc, Tac, "orange");

    // Write to file
    svg.write_file("/tmp/frames.svg");
    print(cerr, "\nSVG written to /tmp/frames.svg\n");

    return 0;
}