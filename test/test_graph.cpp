#include "catch.hpp"

#include "graph.h"

TEST_CASE("Graph","[graph]")
{
    Graph g;

    REQUIRE(g.add_node(Point(0.0,0.0,0.0))==0);
    REQUIRE(g.add_node(Point(1.0,0.0,0.0))==1);
    REQUIRE(g.add_node(Point(0.0,1.0,0.0))==2);
    REQUIRE(g.add_node(Point(0.0,0.0,1.0))==3);

    g.add_edge(1,0);
    g.add_edge(2,0);
    g.add_edge(2,1);
    g.add_edge(3,0);

    SECTION("Test edge structure")
    {
        auto it = g.edges_begin();
        REQUIRE(*it++==Edge{0,1});
        REQUIRE(*it++==Edge{0,2});
        REQUIRE(*it++==Edge{1,2});
        REQUIRE(*it++==Edge{0,3});

        auto edges = g.get_incident_edges(0);
        REQUIRE(edges==std::vector<Edge>{Edge(0,1),Edge(0,2),Edge(0,3)});

        edges = g.get_incident_edges(1);
        REQUIRE(edges==std::vector<Edge>{Edge(0,1),Edge(1,2)});

        edges = g.get_incident_edges(2);
        REQUIRE(edges==std::vector<Edge>{Edge(0,2),Edge(1,2)});
    }
    SECTION("Test node classification")
    {
        REQUIRE(g.is_joint(0));
        REQUIRE(g.is_articulation(1));
        REQUIRE(g.is_articulation(2));
        REQUIRE(g.is_dangling(3));
    }
}

TEST_CASE("Graph test extreme situations","[graph]")
{
    Graph g;

    REQUIRE(g.add_node(Point(0.0,0.0,0.0))==0);
    REQUIRE(g.add_node(Point(1.0,0.0,0.0))==1);

    g.add_edge(0,0);
    g.add_edge(1,1);

    REQUIRE(g.is_dangling(0));
    REQUIRE(g.is_dangling(1));

    REQUIRE(g.get_incident_edges(0)==std::vector<Edge>{Edge(0,0)});
    REQUIRE(g.get_incident_edges(1)==std::vector<Edge>{Edge(1,1)});
}
