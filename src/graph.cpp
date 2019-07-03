#include "graph.h"

int Graph::add_node(const Point& p)
{
    //TODO: check duplicates
    nodes.push_back(p);
    incident_edges.push_back(std::vector<Edge>());
    return nodes.size()-1;
}

void Graph::add_edge(int i, int j)
{
    //TODO: check duplicates
    edges.push_back(Edge{i,j});
    incident_edges[i].push_back(Edge{i,j});
    incident_edges[j].push_back(Edge{i,j});
}
