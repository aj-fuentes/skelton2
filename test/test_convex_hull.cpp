#include "catch.hpp"

#include "convex_hull.h"

#define VEQUAL(a,b) (((a)-(b)).norm())==Approx(0.0)

TEST_CASE("ConvexHull","[convex_hull]")
{
    std::vector<Point> points{Point(1,0,0),Point(0,1,0),Point(0,0,1),Point(-1,-1,-1)};

    SECTION("Constructor")
    {
        ConvexHull chull(points);
        REQUIRE(VEQUAL(chull.get_node(0),points[0]));
        REQUIRE(VEQUAL(chull.get_node(1),points[1]));
        REQUIRE(VEQUAL(chull.get_node(2),points[2]));
        REQUIRE(VEQUAL(chull.get_node(3),points[3]));
    }

    SECTION("Compute")
    {
        ConvexHull chull(points);
        chull.compute();
    }
}
