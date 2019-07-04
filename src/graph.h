#ifndef GRAPH_H
#define GRAPH_H

#include "base.h"
#include <set>


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
    bool operator <(const Edge& e) const {
        return i<e.i or (i==e.i and j<e.j);
    }
};

class Graph
{
public:
    int add_node(const Point&);
    bool add_edge(int,int);
    const Point get_node(int) const;
    std::vector<Point>::const_iterator nodes_begin() const
    {
        return nodes.begin();
    }
    std::vector<Point>::const_iterator nodes_end() const
    {
        return nodes.end();
    }
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
    std::vector<Edge>::const_iterator incident_edges_end(int i) const
    {
        return incident_edges[i].end();
    }
    std::set<int>::const_iterator adjacent_points_begin(int i) const
    {
        return adjacent_points[i].begin();
    }
    std::set<int>::const_iterator adjacent_points_end(int i) const
    {
        return adjacent_points[i].end();
    }
protected:
    std::vector<Point> nodes;
    std::vector<Edge> edges;
    std::vector<std::vector<Edge>> incident_edges;
    std::vector<std::set<int>> adjacent_points;
};

#endif
