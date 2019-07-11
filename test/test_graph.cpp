#include "catch.hpp"

#include "graph.h"
#include <fstream>
#include <cstdio>

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
        REQUIRE(g.get_edges()==std::vector<Edge>{Edge(0,1),Edge(0,2),Edge(1,2),Edge(0,3)});

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

TEST_CASE("Graph test read from file","[graph]")
{
    std::string fname = "__test_graph_file__.obj";

    std::ofstream fout(fname);
    fout << "v 0.0 0.0 0.0" << std::endl;
    fout << "v 1.0 0.0 0.0" << std::endl;
    fout << "v 0.0 1.0 0.0" << std::endl;
    fout << "v 0.0 0.0 1.0" << std::endl;
    fout << "l 1 0" << std::endl;
    fout << "l 2 0" << std::endl;
    fout << "l 2 1" << std::endl;
    fout << "l 3 0" << std::endl;
    fout.close();

    Graph g;

    g.read_from_file(fname);

    std::vector<Point> points{Point(0.0,0.0,0.0),Point(1.0,0.0,0.0),Point(0.0,1.0,0.0),Point(0.0,0.0,1.0)};
    std::vector<Edge> edges{Edge(0,1),Edge(0,2),Edge(1,2),Edge(0,3)};

    REQUIRE(g.get_nodes()==points);
    REQUIRE(g.get_edges()==edges);

    // remove("__test_graph_file__.obj");

}
