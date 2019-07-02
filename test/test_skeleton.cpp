#include "catch.hpp"

#include "skeleton.h"

#define VEQUAL(a,b) (((a)-(b)).norm())==Approx(0.0)

TEST_CASE("Segment","[skeleton]")
{
    Point p = Point::Zero();
    double l = 5.0;
    UnitVector v = UnitVector(1.0,0.0,0.0);
    UnitVector n = UnitVector(0.0,1.0,0.0);

    SECTION("Constructor")
    {
        Skeleton_ptr seg = std::shared_ptr<Segment>(new Segment(p,v,l,n));

        REQUIRE(VEQUAL(seg->get_start_point(),p));
        REQUIRE(VEQUAL(seg->get_end_point(),Point(5.0,0.0,0.0)));
        REQUIRE(VEQUAL(seg->get_point(2.5),Point(2.5,0.0,0.0)));

        REQUIRE(VEQUAL(seg->get_tangent(0.0),UnitVector(1.0,0.0,0.0)));
        REQUIRE(VEQUAL(seg->get_tangent(2.0),UnitVector(1.0,0.0,0.0)));
        REQUIRE(VEQUAL(seg->get_tangent(5.0),UnitVector(1.0,0.0,0.0)));

        REQUIRE(VEQUAL(seg->get_normal(0.0),UnitVector(0.0,1.0,0.0)));
        REQUIRE(VEQUAL(seg->get_normal(2.0),UnitVector(0.0,1.0,0.0)));
        REQUIRE(VEQUAL(seg->get_normal(5.0),UnitVector(0.0,1.0,0.0)));

        REQUIRE(VEQUAL(seg->get_binormal(0.0),UnitVector(0.0,0.0,1.0)));
        REQUIRE(VEQUAL(seg->get_binormal(2.0),UnitVector(0.0,0.0,1.0)));
        REQUIRE(VEQUAL(seg->get_binormal(5.0),UnitVector(0.0,0.0,1.0)));

        REQUIRE(seg->get_distance(Point( 3.3, 0.0, 0.0))==Approx(0.0));
        REQUIRE(seg->get_distance(Point( 0.0, 0.0, 0.0))==Approx(0.0));
        REQUIRE(seg->get_distance(Point( 5.0, 0.0, 0.0))==Approx(0.0));
        REQUIRE(seg->get_distance(Point(-1.0, 0.0, 0.0))==Approx(1.0));
        REQUIRE(seg->get_distance(Point( 6.0, 0.0, 0.0))==Approx(1.0));
        REQUIRE(seg->get_distance(Point(-3.0, 4.0, 0.0))==Approx(5.0));
        REQUIRE(seg->get_distance(Point(-3.0, 0.0, 4.0))==Approx(5.0));
        REQUIRE(seg->get_distance(Point( 8.0,-4.0, 0.0))==Approx(5.0));
        REQUIRE(seg->get_distance(Point( 8.0, 0.0,-4.0))==Approx(5.0));
        REQUIRE(seg->get_distance(Point( 1.0, 1.0, 0.0))==Approx(1.0));
        REQUIRE(seg->get_distance(Point( 1.0, 0.0, 1.0))==Approx(1.0));
        REQUIRE(seg->get_distance(Point( 4.0, 2.0, 0.0))==Approx(2.0));
        REQUIRE(seg->get_distance(Point( 4.0, 0.0, 2.0))==Approx(2.0));
    }
}
