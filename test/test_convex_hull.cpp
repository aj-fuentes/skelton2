#include "catch.hpp"

#include "convex_hull.h"
#include <set>

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
        ConvexHull chull({{-1,-1,0},{1,0,0},{0,1,0},{0,0,1}});
        chull.compute();

        REQUIRE(chull.is_planar()==false);
    }

    SECTION("Convex hull of 5 vertices")
    {
        ConvexHull chull({{-1,-1,0},{1,0,0},{0,1,0},{0,0,1},{1,1,1}});
        chull.compute();

        REQUIRE(chull.is_planar()==false);
    }

    SECTION("Convex hull of a cube")
    {
        ConvexHull chull({
            {-1,-1,-1},{1,-1,-1},{-1,1,-1},{-1,-1,1},{1,1,-1},
            {-1,-1,1},{1,-1,1},{-1,1,1},{-1,-1,1},{1,1,1}
        });
        chull.compute();

        REQUIRE(chull.is_planar()==false);
    }
}
TEST_CASE("ConvexHull planar","[convex_hull]")
{
    SECTION("Convex hull of a triangle")
    {
        std::vector<Point> points = {Point(1,0,0),Point(0,1,0),Point(-1,-1,0).normalized()};
        ConvexHull chull(points);
        chull.compute();

        REQUIRE(chull.is_planar());

        auto edges = chull.get_incident_edges(0);
        REQUIRE(edges==std::vector<Edge>{Edge(0,1),Edge(0,2)});

        edges = chull.get_incident_edges(1);
        REQUIRE(edges==std::vector<Edge>{Edge(0,1),Edge(1,2)});

        edges = chull.get_incident_edges(2);
        REQUIRE(edges==std::vector<Edge>{Edge(1,2),Edge(0,2)});

        auto edual = chull.edge_dual(Edge(0,1));
        REQUIRE(edual.u==UnitVector(0,0,1));
        REQUIRE(edual.v.dot(points[0]-points[1])==Approx(0).margin(TOL));
        REQUIRE(edual.phi==Approx(PI_));

        edual = chull.edge_dual(Edge(0,2));
        REQUIRE(edual.u==UnitVector(0,0,1));
        REQUIRE(edual.v.dot(points[0]-points[2])==Approx(0).margin(TOL));
        REQUIRE(edual.phi==Approx(PI_));

        edual = chull.edge_dual(Edge(1,2));
        REQUIRE(edual.u==UnitVector(0,0,1));
        REQUIRE(edual.v.dot(points[1]-points[2])==Approx(0).margin(TOL));
        REQUIRE(edual.phi==Approx(PI_));


    }
}

TEST_CASE("ConvexHull 2 nodes","[convex_hull]")
{
    std::vector<Point> points = {Point(1,0,0),Point(0,1,0)};
    ConvexHull chull(points);
    chull.compute();

    REQUIRE(chull.is_planar());

    auto edges = chull.get_incident_edges(0);
    REQUIRE(edges==std::vector<Edge>{Edge(0,1)});

    auto edual = chull.edge_dual(Edge(0,1));
    auto w = points[0]-points[1];
    REQUIRE(edual.u.dot(w)==Approx(0).margin(TOL));
    REQUIRE(edual.v.dot(w)==Approx(0).margin(TOL));
    REQUIRE(edual.phi==Approx(2*PI_));

}

TEST_CASE("ConvexHull 1 node","[convex_hull]")
{
    std::vector<Point> points = {Point(1,0,0)};
    ConvexHull chull(points);
    chull.compute();

    REQUIRE(chull.is_planar());

    auto edges = chull.get_incident_edges(0);
    REQUIRE(edges==std::vector<Edge>{Edge(0,0)});

    auto edual = chull.edge_dual(Edge(0,0));
    REQUIRE(edual.u.dot(points[0])==Approx(0).margin(TOL));
    REQUIRE(edual.v.dot(points[0])==Approx(0).margin(TOL));
    REQUIRE(edual.phi==Approx(2*PI_));

}
