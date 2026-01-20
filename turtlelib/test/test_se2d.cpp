// ############## Begin_Citation[0] ####################
#include <catch2/catch_test_macros.hpp>
#include "turtlelib/se2d.hpp"
#include "turtlelib/angle.hpp"

TEST_CASE("Create a twist with format 'w x y' and 'x <unit> x y' and <w x y>, with radians and degrees", "[Miguel]")
{
    std::stringstream stream{"1 1 1"};
    turtlelib::Twist2D tw1;
    stream >> tw1;
    REQUIRE(tw1.omega == 1.0);
    REQUIRE(tw1.x == 1.0);
    REQUIRE(tw1.y == 1.0);

    std::stringstream stream2 {"1 d 1 1"};
    turtlelib::Twist2D tw2;
    stream2 >> tw2;

    REQUIRE(tw2.omega == turtlelib::deg2rad(1.0));
    REQUIRE(tw2.x == 1.0);
    REQUIRE(tw2.y == 1.0);

    std::stringstream stream3 {"<-2 d, 3, 2>"};
    turtlelib::Twist2D tw3;
    stream3 >> tw3;

    REQUIRE(tw3.omega == turtlelib::deg2rad(-2));
    REQUIRE(tw3.x == 3.0);
    REQUIRE(tw3.y == 2.0);
}

TEST_CASE("Create transforms", "Transform2D")
{
    // Check that default constructor is correct
    turtlelib::Transform2D tf;
    REQUIRE(tf.tw.omega == 0);
    REQUIRE(tf.tw.x == 0);
    REQUIRE(tf.tw.y == 0);

    // Check that rotate constructor is correct
    double rotate {1};
    turtlelib::Transform2D tf2(rotate);
    REQUIRE(tf2.tw.omega == 1);
    REQUIRE(tf2.tw.x == 0);
    REQUIRE(tf2.tw.y == 0);

    // Check trans constructor
    turtlelib::Vector2D vc;
    vc.x = 1;
    vc.y = 2;
    turtlelib::Transform2D tf3(vc);
    REQUIRE(tf3.tw.omega == 0);
    REQUIRE(tf3.tw.x == 1);
    REQUIRE(tf3.tw.y == 2);

    // Check rot and trans
    turtlelib::Transform2D tf4(vc, rotate);
    REQUIRE(tf2.tw.omega == 1);
    REQUIRE(tf3.tw.x == 1);
    REQUIRE(tf3.tw.y == 2);
}

TEST_CASE("Transform a point", "Transform2D")
{
    // origin with no transformation
    turtlelib::Transform2D tf;
    turtlelib::Point2D pt;
    auto pt2 = tf(pt);
    REQUIRE(pt2.x == pt.x);
    REQUIRE(pt2.y == pt.y);

    // origin with pure rotation
    double rot = turtlelib::deg2rad(45);
    turtlelib::Transform2D tf2(rot);
    auto pt3 = tf2(pt2);
    REQUIRE(pt3.x == pt2.x);
    REQUIRE(pt3.y == pt2.y);

    // origin with pure translation
    turtlelib::Vector2D oo;
    oo.x = 1;
    oo.y = -1;
    turtlelib::Transform2D tf4(oo);
    auto pt4 = tf4(pt3);
    REQUIRE(pt4.x == oo.x);
    REQUIRE(pt4.y == oo.y);

    // from this point, translate and rotate
    turtlelib::Transform2D tf5(oo, rot);
    auto pt5 = tf5(pt4);
    REQUIRE(pt5.x == 1+std::sqrt(2));
    REQUIRE(pt5.y == -1);
}

TEST_CASE("Transform a vector", "Transform2D")
{
    // origin with no transformation
    turtlelib::Transform2D tf;
    turtlelib::Vector2D vc;
    auto vc2 = tf(vc);
    REQUIRE(vc2.x == vc.x);
    REQUIRE(vc2.y == vc.y);

    // origin with pure rotation
    double rot = turtlelib::deg2rad(45);
    turtlelib::Transform2D tf2(rot);
    auto vc3 = tf2(vc2);
    REQUIRE(vc3.x == vc2.x);
    REQUIRE(vc3.y == vc2.y);

    // origin with pure translation
    turtlelib::Vector2D oo;
    oo.x = 1;
    oo.y = -1;
    turtlelib::Transform2D tf4(oo);
    auto vc4 = tf4(vc3);
    REQUIRE(vc4.x == oo.x);
    REQUIRE(vc4.y == oo.y);

    // from this point, translate and rotate
    turtlelib::Transform2D tf5(oo, rot);
    auto vc5 = tf5(vc4);
    REQUIRE(vc5.x == 1 + std::sqrt(2));
    REQUIRE(vc5.y == -1);
}

// TODO: need test for transforming a Twist2D


// ############## End_Citation[0] ####################