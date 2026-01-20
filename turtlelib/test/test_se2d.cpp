// ############## Begin_Citation[0] ####################
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "turtlelib/se2d.hpp"
#include "turtlelib/angle.hpp"

using namespace turtlelib;
using namespace Catch::Matchers;

TEST_CASE("Twist2D operator>>", "[Miguel]")
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

TEST_CASE("test constructors")
{
    // Check that default constructor is correct
    turtlelib::Transform2D tf;

    double rotate {1};
    turtlelib::Transform2D tf2(rotate);
    REQUIRE(tf2.rotation() == 1);
    REQUIRE(tf2.translation().x == 0.0); 
    REQUIRE(tf2.translation().y == 0.0);

    // Check trans constructor
    turtlelib::Vector2D vc;
    vc.x = 1;
    vc.y = 2;
    turtlelib::Transform2D tf3(vc);
    REQUIRE(tf3.rotation() == 0);
    REQUIRE(tf3.translation().x == 1); 
    REQUIRE(tf3.translation().y == 2);
    // Check rot and trans
    turtlelib::Transform2D tf4(vc, rotate);
    REQUIRE(tf4.rotation() == 1);
    REQUIRE(tf4.translation().x == 1); 
    REQUIRE(tf4.translation().y == 2);
}

TEST_CASE("Transform a point")
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

TEST_CASE("Transform a vector")
{
    SECTION("origin, no transformation") {
        turtlelib::Transform2D tf;
        turtlelib::Vector2D vc;
        // origin with no transformation
        auto vc2 = tf(vc);
        REQUIRE(vc2.x == 0);
        REQUIRE(vc2.y == 0);
    }

    SECTION("origin with pure rotation") {
        double rot = turtlelib::deg2rad(45);
        turtlelib::Transform2D tf(rot);
        turtlelib::Vector2D vc;
        auto vc2 = tf(vc);
        REQUIRE(vc2.x == vc.x);
        REQUIRE(vc2.y == vc.y);
    }

    SECTION("origin with pure translation") {
        // translation is ignored by vectors

        turtlelib::Vector2D vc;
        turtlelib::Vector2D oo;
        oo.x = 1;
        oo.y = -1;
        turtlelib::Transform2D tf(oo);
        auto vc2 = tf(vc);
        REQUIRE(vc2.x == 0);
        REQUIRE(vc2.y == 0);
    }

    SECTION("translate and rotate") {
        // translation is ignored
        double rot = turtlelib::deg2rad(45);
        turtlelib::Vector2D oo;
        oo.x = 1;
        oo.y = -1;
        turtlelib::Transform2D tf(oo, rot);
        turtlelib::Vector2D vc {1, 1};
        auto vc2 = tf(vc);
        
        REQUIRE_THAT(vc2.x, WithinAbs(0.0, 0.00001));
        REQUIRE_THAT(vc2.y, WithinAbs(std::sqrt(2), 0.00001));
    }
}

// ############## End_Citation[0] ####################

TEST_CASE("Transform a twist") {
    auto v = Vector2D{1,2};
    auto tf = Transform2D(v, std::numbers::pi);

    SECTION("Transform a null twist.") {
        // should do nothing
        auto tw = Twist2D();
        
        auto out = tf(tw);
        REQUIRE(out.x == 0);
        REQUIRE(out.y == 0);
        REQUIRE(out.omega == 0);
    }

    SECTION("Transform a non-null twist") {
        // validated using kevin's modern robotics library
        auto tw = Twist2D {std::numbers::pi, 3, 4};

        // output should be (in SE(3)):
        // [[ 0.   ]
        //  [ 0.   ]
        //  [-3.142]
        //  [-9.283]
        //  [ 7.142]
        //  [-0.   ]]

        auto out = tf(tw);

        REQUIRE_THAT(out.omega, WithinAbs(std::numbers::pi, 0.001));
        REQUIRE_THAT(out.x, WithinAbs(-9.283, 0.001));
        REQUIRE_THAT(out.y, WithinAbs(7.142, 0.001));
    }
}

TEST_CASE("Transform2D operator>>")
{
    SECTION("no brackets, radians") {
        std::stringstream stream{"3.14 rad 1 2"};
        turtlelib::Transform2D tf;
        stream >> tf;
        REQUIRE(tf.rotation() == 3.14);
        REQUIRE(tf.translation().x == 1.0);
        REQUIRE(tf.translation().y == 2.0);
    }

    SECTION("no brackets, degrees") {
        std::stringstream stream{"0 d 1 2"};
        turtlelib::Transform2D tf;
        stream >> tf;
        REQUIRE(tf.rotation() == 0.0);
        REQUIRE(tf.translation().x == 1.0);
        REQUIRE(tf.translation().y == 2.0);
    }

    SECTION("brackets, radians") {
        std::stringstream stream{"{3.14 r, 1, 2}"};
        turtlelib::Transform2D tf;
        stream >> tf;
        REQUIRE(!stream.fail());
        REQUIRE(tf.translation().x == 1.0);
        REQUIRE(tf.translation().y == 2.0);
        REQUIRE_THAT(tf.rotation(),  WithinAbs(3.14, 0.01));
    }

}
