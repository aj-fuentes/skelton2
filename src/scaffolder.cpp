#include "scaffolder.h"

#include <fstream>
#include <sstream>
#include <glpk.h>

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
    //setup arc variables
    std::vector<std::string> arc_variables;
    for(int i=0;i<chulls.size();i++)
    {
        auto ch = chulls[i];
        for(auto e : ch.get_edges())
        {
            arc_variables.push_back(arc_variable(i,e));
            mip_lp << "var " << arc_variable(i,e) << ", integer, >= ";
            // TODO: put the actual number of subdivisions according to the length of the arc
            mip_lp << arc_min_subdiv(i,e) << ";" << std::endl;
        }
    }

    //setup cell variables
    std::vector<std::string> cell_variables;
    for(auto e : g->get_edges())
    {
        // example: "var q_101_21_32, integer, >= 4;\n"
        cell_variables.push_back(cell_sum_variable(e.i,e));
        mip_lp << "var " << cell_sum_variable(e.i,e) << ", integer, >= ";
        mip_lp << min_cell_quads << ";" << std::endl;

        cell_variables.push_back(cell_sum_variable(e.j,e));
        mip_lp << "var " << cell_sum_variable(e.j,e) << ", integer, >= ";
        mip_lp << min_cell_quads << ";" << std::endl;
    }

    //extra variable for regular scaffolds
    if(regular)
        mip_lp << "var q, integer;" << std::endl;

    mip_lp << std::endl;

    //setup objective function
    mip_lp << "minimize quads_bound: ";
    for(int i=0;i<chulls.size();i++)
    {
        auto mult = g->is_joint(i)? "2*" : "";
        auto ch = chulls[i];
        for(auto e : ch.get_edges())
        {
            mip_lp << " + " << mult << arc_variable(i,e);
        }
    }
    mip_lp << ";" << std::endl;

    mip_lp << std::endl;

    //setup cell sum equations
    for(int i=0;i<chulls.size();i++) //i is the node in the skeleton graph
    {
        for(int j=0;j<chulls[i].get_nodes().size();j++) //j is the node in the convex hull
        {
            //j coincides with the j-th incident edge to the i-th node in the skeleton graph
            auto g_e = g->get_incident_edges(i)[j];
            mip_lp << "s.t. cell_sum_" << i << "_" << g_e.i <<"_" << g_e.j << ":";

            for(auto ch_e : chulls[i].get_incident_edges(j))
                mip_lp << " + " << arc_variable(i,ch_e);

            mip_lp << " = " << cell_sum_variable(i,g_e) << ";" << std::endl;
        }
    }

    //setup cell compatibility equations
    for(auto e : g->get_edges())
    {
        mip_lp << "s.t. compatibility_" << e.i << "_" << e.j << ": ";
        mip_lp << cell_sum_variable(e.i,e) << " = " << cell_sum_variable(e.j,e);
        mip_lp << ";" << std::endl;
        //setup regular equations
        if(regular)
        {
            mip_lp << "s.t. regular_" << e.i << "_" << e.j << ": ";
            mip_lp << cell_sum_variable(e.i,e) << " = q;" << std::endl;
        }
    }


    mip_lp << std::endl;

    //call solve
    mip_lp << "solve;" << std::endl;

    mip_lp << std::endl;

    //setup output solution
    // mip_lp << "printf: 'quads_bound %d\\n', quads_bound >> '" << mip_sol_file <<  "';";
    // mip_lp << std::endl;

    for(auto v : arc_variables)
    {
        mip_lp << "printf: '" << v << " %d\\n', " << v << " >> '" << mip_sol_file << "';";
        mip_lp << std::endl;
    }

    mip_lp << std::endl;

    mip_lp <<  "end;" << std::endl;
}

void Scaffolder::solve_mip()
{
    //clear output file
    std::ofstream fout(mip_sol_file);
    fout.close();

    // disable terminal output
    // TODO: setup log file
    glp_term_out(GLP_OFF);

    // create the mip
    auto mip = glp_create_prob();

    // create the model translator
    auto tran = glp_mpl_alloc_wksp();

    // read the model intro translator
    glp_mpl_read_model(tran, mip_lp_file.c_str(), 0);

    // generate the model
    glp_mpl_generate(tran, nullptr);

    // build the mip from the model
    glp_mpl_build_prob(tran, mip);

    // create and init params for MIP solver
    auto params = glp_iocp();
    glp_init_iocp(&params);
    params.presolve = GLP_ON;

    // solve the MIP
    glp_intopt(mip,&params);

    // save solution
    glp_mpl_postsolve(tran,mip,GLP_MIP);

    // free resources
    glp_mpl_free_wksp(tran);
    glp_delete_prob(mip);
}

void Scaffolder::read_mip_solution()
{

}

void Scaffolder::compute()
{
    compute_convex_hulls();

    std::stringstream mip_lp;
    setup_mip(mip_lp);

    //save model to file
    std::ofstream f(mip_lp_file);
    f << mip_lp.str();
    f.close();

    solve_mip();
    read_mip_solution();
}
