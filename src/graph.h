#ifndef GRAPH_H
#define GRAPH_H

#include "base.h"
#include <set>
#include <ostream>

class Graph;
typedef std::shared_ptr<Graph> Graph_ptr;

struct Edge
{
    int i,j;
    Edge(int i, int j) :
        i(i<j? i : j), j(i<j? j : i)
    {}
    bool operator ==(const Edge& e) const
    {
        return i==e.i and j==e.j;
    }
    bool operator <(const Edge& e) const
    {
        return i<e.i or (i==e.i and j<e.j);
    }
    friend std::ostream& operator <<(std::ostream&,const Edge&);
};

class Graph
{
public:
    int add_node(const Point&);
    bool add_edge(int,int);
    void read_from_file(const std::string&);
    const Point& get_node(int i) const
    {
        return nodes[i];
    }
    int node_count() const
    {
        return nodes.size();
    }
    bool is_dangling(int i) const
    {
        return incident_edges[i].size()==1;
    }
    bool is_articulation(int i) const
    {
        return incident_edges[i].size()==2;
    }
    bool is_joint(int i) const {
        return incident_edges[i].size()>2;
    }

    const std::vector<Point>& get_nodes() const
    {
        return nodes;
    }
    const std::vector<Edge>& get_edges() const
    {
        return edges;
    }
    const std::vector<Edge>& get_incident_edges(int i) const
    {
        return incident_edges[i];
    }

protected:
    std::vector<Point> nodes;
    std::vector<Edge> edges;
    std::vector<std::vector<Edge>> incident_edges;
    std::vector<std::set<int>> adjacent_nodes;
};

#endif
