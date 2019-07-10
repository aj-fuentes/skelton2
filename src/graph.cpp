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

bool Graph::add_edge(int i, int j)
{
    Edge e(i,j);

    //check if edge already present
    if(adjacent_nodes[i].find(j)!=adjacent_nodes[i].end())
        return false;

    edges.push_back(e);

    incident_edges[i].push_back(e);
    adjacent_nodes[i].insert(j);

    if(i!=j) //self loops possible only for dangling nodes
    {
        incident_edges[j].push_back(e);
        adjacent_nodes[j].insert(i);
    }

    return true;
}
