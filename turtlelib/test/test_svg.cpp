#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "turtlelib/svg.hpp"

using namespace turtlelib;


TEST_CASE("SVG stringify", "[Conor]")
{
    constexpr auto expected_output = R"'''(<svg width="8.500000in" height="11.000000in" viewBox="0.000000 0.000000 816.000000 1056.000000" xmlns="http://www.w3.org/2000/svg">

<defs>
  <marker style="overflow:visible" id="Arrow1Sstart" refX="0.0" refY="0.0" orient="auto">
       <path transform="scale(0.2) translate(6,0)" style="fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt" d="M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z "/>
    </marker>
</defs>
        

<g id="4">
    <line x1="504.000000" x2="408.000000" y1="528.000000" y2="528.000000" stroke="red" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
    <line x1="408.000000" x2="408.000000" y1="432.000000" y2="528.000000" stroke="green" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
    <text x="413.000000" y="523.000000">{W}</text>
</g>
        

<line id="2" x1="600.000000" x2="408.000000" y1="240.000000" y2="528.000000" stroke="pink" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
        

<g id="5">
    <line x1="667.882251" x2="600.000000" y1="364.117749" y2="432.000000" stroke="red" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
    <line x1="532.117749" x2="600.000000" y1="364.117749" y2="432.000000" stroke="green" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
    <text x="605.000000" y="427.000000">{B}</text>
</g>
        

<line id="3" x1="264.000000" x2="408.000000" y1="288.000000" y2="528.000000" stroke="purple" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
        

<circle id="1" cx="120.0" cy="384.0" r="3" stroke="green" fill="green" stroke-width="1"/>
        

<circle id="0" cx="504.0" cy="336.0" r="3" stroke="blue" fill="blue" stroke-width="1"/>
        
</svg>
)'''";
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

    // get the string output
    auto output = svg.to_string();
    REQUIRE(output == expected_output);
}
