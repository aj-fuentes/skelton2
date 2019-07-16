#ifndef SCAFFOLDER_H
#define SCAFFOLDER_H

#include "base.h"
#include "convex_hull.h"

class Scaffolder;

typedef std::shared_ptr<Scaffolder> Scaffolder_ptr;
typedef Edge GraphEdge;
typedef Edge ConvexHullEdge;

class Mesher;

class Scaffolder {
    friend class Mesher;
public:
    static std::string cell_sum_variable(int,const GraphEdge&);
    static std::pair<int,const GraphEdge> parse_cell_sum_variable(const std::string&);

    static std::string arc_variable(int,const ConvexHullEdge&);
    static std::pair<int,const ConvexHullEdge> parse_arc_variable(const std::string&);

    static std::vector<std::pair<int,int>> math_cells(const std::vector<Point>&,
    const std::vector<Point>&, const Vector=Vector(0,0,0));

    Scaffolder(const Graph_ptr& graph) :
        g(graph), max_arc_angle(0.9*PI_), min_cell_quads(4), regular(false), symmetric(false),
        arc_based_optimal_solution(false), mip_lp_file("__scaffolder_mip_lp__.mod"),
        mip_sol_file("__scaffolder_mip_lp__.sol")
    {
        g->check_graph();
    }
    void set_regular(bool reg)
    {
        regular = reg;
    }
    void set_symmetric(bool sym)
    {
        symmetric = sym;
        if(not symmetric)
            symmetries.clear();
    }
    void add_symmetry(const std::vector<int>& s)
    {
        symmetries.push_back(s);
    }
    void set_arc_based_optimal_solution(bool arc_opt)
    {
        arc_based_optimal_solution = arc_opt;
    }
    void set_mip_lp_file(std::string s)
    {
        mip_lp_file = s;
    }
    void set_mip_sol_file(std::string s)
    {
        mip_sol_file = s;
    }
    void set_max_arc_angle(double phi)
    {
        max_arc_angle = phi;
    }

    void compute();
    void save_to_file(const std::string&,bool=false,bool=false) const;

private:

    void compute_convex_hulls();
    void setup_mip(std::ostream&);
    void solve_mip();
    void read_mip_solution();
    void compute_cells_match();
    void compute_cells();
    void compute_cell(int,int);

    int cell_sum_value(int i,const GraphEdge& e)
    {
        return var_values[cell_sum_variable(i,e)];
    }
    int arc_value(int i ,const ConvexHullEdge& e)
    {
        return var_values[arc_variable(i,e)];
    }
    int arc_min_subdiv(int i, const ConvexHullEdge& e)
    {
        return std::max(int(chulls[i].edge_dual(e).phi/max_arc_angle),1);
    }

    const Graph_ptr g;
    std::vector<ConvexHull> chulls;
    std::map<std::string,int> var_values;
    std::map<std::pair<int,GraphEdge>,std::vector<Point>> cells;
    std::map<GraphEdge,std::vector<std::pair<int,int>>> cells_match;

    double max_arc_angle;
    int min_cell_quads;
    bool regular;
    bool symmetric;
    bool arc_based_optimal_solution;
    std::vector<std::vector<int>> symmetries;

    std::string mip_lp_file;
    std::string mip_sol_file;
};

#endif
