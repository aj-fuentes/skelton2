#include "catch.hpp"

#include "convex_hull.h"

TEST_CASE("ConvexHull","[convex_hull]")
{
    std::vector<Point> points{Point(1,0,0),Point(0,1,0),Point(0,0,1),Point(-1,-1,-1)};

    SECTION("Constructor")
    {
        ConvexHull chull(points);
    }
}
