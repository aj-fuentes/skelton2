#include "catch.hpp"

#include "graph.h"

TEST_CASE("Graph","[graph]")
{
    Graph g;

    REQUIRE(g.add_node(Point(0.0,0.0,0.0))==0);
    REQUIRE(g.add_node(Point(1.0,0.0,0.0))==1);
    REQUIRE(g.add_node(Point(0.0,1.0,0.0))==2);

    g.add_edge(1,0);
    g.add_edge(2,0);
    g.add_edge(2,1);

    auto it = g.edges_begin();
    REQUIRE(*it++==Edge{0,1});
    REQUIRE(*it++==Edge{0,2});
    REQUIRE(*it++==Edge{1,2});

    auto it2 = g.incident_edges_begin(0);
    REQUIRE(*it2++==Edge{0,1});
    REQUIRE(*it2++==Edge{0,2});

    it2 = g.incident_edges_begin(1);
    REQUIRE(*it2++==Edge{0,1});
    REQUIRE(*it2++==Edge{1,2});

    it2 = g.incident_edges_begin(2);
    REQUIRE(*it2++==Edge{0,2});
    REQUIRE(*it2++==Edge{1,2});
}
