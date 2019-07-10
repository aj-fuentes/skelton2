#ifndef SCAFFOLDER_H
#define SCAFFOLDER_H

#include "base.h"
#include "convex_hull.h"
#include <iostream>

#include <string>

class Scaffolder;

typedef std::shared_ptr<Scaffolder> Scaffolder_ptr;
typedef Edge GraphEdge;
typedef Edge ConvexHullEdge;

class Scaffolder {
public:
    static std::string cell_sum_variable(int,const GraphEdge&);
    static std::pair<int,const GraphEdge> parse_cell_sum_variable(const std::string&);

    static std::string arc_variable(int,const ConvexHullEdge&);
    static std::pair<int,const ConvexHullEdge> parse_arc_variable(const std::string&);

    Scaffolder(const Graph_ptr& graph) :
        g(graph), long_arc_angle(0.9*PI_), min_cell_quads(4), regular(false), symmetric(false)
    {}
    void compute();
    void save_to_file() const;

private:

    void compute_convex_hulls();
    void setup_mip(std::ostream&);
    void solve_mip(const std::ostream&);
    void compute_cells();

    int cell_sum_value(int i,const GraphEdge& e)
    {
        return var_values[cell_sum_variable(i,e)];
    }
    int arc_value(int i ,const ConvexHullEdge& e)
    {
        return var_values[arc_variable(i,e)];
    }

    const Graph_ptr g;
    std::vector<ConvexHull> chulls;
    std::map<std::string,int> var_values;
    std::map<std::pair<int,GraphEdge>,std::vector<Point>> cells;

    double long_arc_angle;
    int min_cell_quads;
    bool regular;
    bool symmetric;
    std::vector<std::vector<int>> symmetries;
};

#endif
