#include "catch.hpp"

#include "convex_hull.h"

#define VEQUAL(a,b) (((a)-(b)).norm())==Approx(0.0)

TEST_CASE("ConvexHull","[convex_hull]")
{

    SECTION("Constructor")
    {
        std::vector<Point> points{Point(0,0,0),Point(1,0,0),Point(0,1,0),Point(0,0,1),Point(1,1,1)};
        ConvexHull chull(points);
        REQUIRE(VEQUAL(chull.get_node(0),points[0]));
        REQUIRE(VEQUAL(chull.get_node(1),points[1]));
        REQUIRE(VEQUAL(chull.get_node(2),points[2]));
        REQUIRE(VEQUAL(chull.get_node(3),points[3]));
    }

    SECTION("Convex hull of a tetrahedron")
    {
        ConvexHull chull({{0,0,0},{1,0,0},{0,1,0},{0,0,1}});
        chull.compute();
    }

    SECTION("Convex hull of 5 vertices of a cube")
    {
        ConvexHull chull({{0,0,0},{1,0,0},{0,1,0},{0,0,1},{1,1,1}});
        chull.compute();
    }

    SECTION("Convex hull of a cube")
    {
        ConvexHull chull({
            {0,0,0},{1,0,0},{0,1,0},{0,0,1},{1,1,0},
            {0,0,1},{1,0,1},{0,1,1},{0,0,1},{1,1,1}
        });
        chull.compute();
    }

    SECTION("Convex hull of a cube with a point inside")
    {
        ConvexHull chull({
            Point(0,0,0),Point(1,0,0),Point(0,1,0),Point(0,0,1),Point(1,1,0),
            Point(0.5,0.5,0.5),
            Point(0,0,1),Point(1,0,1),Point(0,1,1),Point(0,0,1),Point(1,1,1),
        });
        chull.compute();
    }

    SECTION("Convex hull of a triangle")
    {
        ConvexHull chull({
            Point(0,0,0),Point(1,0,0),Point(0,1,0)
        });
        chull.compute();
    }
}
