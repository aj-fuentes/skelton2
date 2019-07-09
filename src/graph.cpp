#include "graph.h"

std::ostream& operator <<(std::ostream& os,const Edge& e)
{
    return os << "edge(" << e.i << "," << e.j << ")";
}

int Graph::add_node(const Point& p)
{
    //TODO: check duplicates
    nodes.push_back(p);
    incident_edges.push_back(std::vector<Edge>());
    adjacent_nodes.push_back(std::set<int>());
    return nodes.size()-1;
}

const Point& Graph::get_node(int i) const {
    return nodes[i];
}

bool Graph::add_edge(int i, int j)
{
    //check if edge already present
    if(adjacent_nodes[i].find(j)!=adjacent_nodes[i].end())
        return false;

    Edge e(i,j);
    edges.push_back(e);
    incident_edges[i].push_back(e);
    incident_edges[j].push_back(e);
    adjacent_nodes[i].insert(j);
    adjacent_nodes[j].insert(i);

    return true;
}
