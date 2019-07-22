#include "catch.hpp"

#include "skeleton.h"

#define VEQUAL(a,b) (((a)-(b)).norm())==Approx(0.0)

TEST_CASE("Segment","[skeleton]")
{
    Point p = Point::Zero();
    double l = 5.0;
    UnitVector v = UnitVector(1.0,0.0,0.0);
    UnitVector n = UnitVector(0.0,1.0,0.0);
    UnitVector b = UnitVector(0.0,0.0,1.0);

    Skeleton_ptr seg = Segment_ptr(new Segment(p,v,l,n));

    SECTION("Points")
    {

        REQUIRE(VEQUAL(seg->get_start_point(),p));
        REQUIRE(VEQUAL(seg->get_end_point(),Point(5.0,0.0,0.0)));
        REQUIRE(VEQUAL(seg->get_point(2.5),Point(2.5,0.0,0.0)));
    }
    SECTION("Tangent")
    {
        REQUIRE(VEQUAL(seg->get_tangent(0.0),v));
        REQUIRE(VEQUAL(seg->get_tangent(2.0),v));
        REQUIRE(VEQUAL(seg->get_tangent(5.0),v));
    }
    SECTION("Normal")
    {
        REQUIRE(VEQUAL(seg->get_normal(0.0),n));
        REQUIRE(VEQUAL(seg->get_normal(2.0),n));
        REQUIRE(VEQUAL(seg->get_normal(5.0),n));
    }
    SECTION("Binormal")
    {
        REQUIRE(VEQUAL(seg->get_binormal(0.0),b));
        REQUIRE(VEQUAL(seg->get_binormal(2.0),b));
        REQUIRE(VEQUAL(seg->get_binormal(5.0),b));
    }
    SECTION("Frame")
    {
        Frame frm = seg->get_frame(0.0);
        REQUIRE(VEQUAL(frm.col(0),v));
        REQUIRE(VEQUAL(frm.col(1),n));
        REQUIRE(VEQUAL(frm.col(2),b));

        frm = seg->get_frame(l);
        REQUIRE(VEQUAL(frm.col(0),v));
        REQUIRE(VEQUAL(frm.col(1),n));
        REQUIRE(VEQUAL(frm.col(2),b));

        frm = seg->get_frame(2.5);
        REQUIRE(VEQUAL(frm.col(0),v));
        REQUIRE(VEQUAL(frm.col(1),n));
        REQUIRE(VEQUAL(frm.col(2),b));
    }
    SECTION("Distance")
    {
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

TEST_CASE("Arc","[skeleton]")
{
    // const Point c(3,2,0);
    // const UnitVector u = Vector(1,1,0).normalized();
    // const UnitVector v = u.cross(Vector(-1,2,0)).normalized();
    // const double r = 5;
    // const double phi = PI_/3;

    const Point c(2,0,0);
    const UnitVector u = Vector(1,0,0).normalized();
    const UnitVector v = u.cross(Vector(0,1,0)).normalized();
    const double r = 5;
    const double phi = PI_/3;

    const auto arc = Arc_ptr(new Arc(c,u,v,r,phi));

    SECTION("Get points")
    {
        int n = 10;
        for(int i=0;i<=n;i++)
        {
            const double t = i*arc->l/n;
            const auto& p =  arc->get_point(t);

            REQUIRE((p-c).norm()==Approx(r).margin(TOL));
            REQUIRE((p-c).dot(arc->b)==Approx(0).margin(TOL));
            REQUIRE(atan2(arc->v.dot(p-c),arc->u.dot(p-c))==Approx(t/r).margin(TOL));
            REQUIRE((p-c).dot(arc->get_tangent(t))==Approx(0).margin(TOL));
        }
    }
}
