#include "scaffolder.h"
#include <fstream>
#include <sstream>

std::string Scaffolder::cell_sum_variable(int i, const GraphEdge& e)
{
    return "q_" + std::to_string(i) + "_" + std::to_string(e.i) + "_" + std::to_string(e.j);
}
std::string Scaffolder::arc_variable(int i, const ConvexHullEdge& e)
{
    return cell_sum_variable(i,e).replace(0,1,"x");
}
std::pair<int,const GraphEdge> Scaffolder::parse_cell_sum_variable(const std::string& s)
{
    std::vector<int> v;
    int i=0;
    while(i<s.length())
    {
        if(s[i++]=='_')
        {
            int start=i;
            while(i<s.length() and s[i]!='_')
                i++;
            v.push_back(std::stoi(s.substr(start,i-start)));
        }
    }
    return {v[0],GraphEdge(v[1],v[2])};
}
std::pair<int,const ConvexHullEdge> Scaffolder::parse_arc_variable(const std::string& s)
{
    return parse_cell_sum_variable(s);
}

void Scaffolder::compute_convex_hulls()
{
    for(int i=0;i<g->node_count();i++)
    {
        Point node = g->get_node(i);
        std::vector<Point> points;
        for(auto e : g->get_incident_edges(i))
            points.push_back(g->get_node(e.i!=i? e.i : e.j)-node);
        chulls.push_back(ConvexHull(points,true));
    }
}

void Scaffolder::setup_mip(std::ostream& mip_lp)
{
    //setup cell variables
    for(auto e : g->get_edges())
    {
        // example: "var q_101_21_32, integer, >= 4;\n"
        mip_lp << "var " << cell_sum_variable(e.i,e) << ", integer, >= ";
        mip_lp << min_cell_quads << ";" << std::endl;
    }
    //setup arc variables
    for(int i=0;i<chulls.size();i++)
    {
        auto ch = chulls[i];
        for(auto e : ch.get_edges())
        {
            mip_lp << "var " << arc_variable(i,e) << ", integer, >= ";
            // TODO: put the actual number of subdivisions according to the length of the arc
            mip_lp << 1 << ";" << std::endl;
        }
    }
    //setup cell sum equations

    //setup cell compatibility equations
}

void Scaffolder::compute()
{
    compute_convex_hulls();
    std::stringstream mip_lp;
    setup_mip(mip_lp);
}
