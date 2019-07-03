#ifndef GRAPH_H
#define GRAPH_H

#include "base.h"


class Graph;
typedef std::shared_ptr<Graph> Graph_ptr;

struct Edge
{
    int i,j;
    Edge(int i, int j) : i(i),j(j)
    {
        if(j<i) std::swap(this->i,this->j);
    }
    bool operator ==(const Edge& e) const {
        return i==e.i and j==e.j;
    }
};

class Graph
{
public:
    int add_node(const Point&);
    void add_edge(int,int);
    Point get_node(int) const;
    std::vector<Edge>::const_iterator edges_begin() const
    {
        return edges.begin();
    }
    std::vector<Edge>::const_iterator edges_end() const
    {
        return edges.end();
    }
    std::vector<Edge>::const_iterator incident_edges_begin(int i) const
    {
        return incident_edges[i].begin();
    }
    std::vector<Edge>::const_iterator incident_edges_end(int i) const {
        return incident_edges[i].end();
    }
private:
    std::vector<Point> nodes;
    std::vector<Edge> edges;
    std::vector<std::vector<Edge>> incident_edges;
};

#endif
