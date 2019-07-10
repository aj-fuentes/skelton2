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
    std::ifstream fin(mip_sol_file);
    std::string var;
    int val;
    while (fin >> var >> val)
    {
        var_values[var] = val;
    }
}

void Scaffolder::compute_cells()
{
    for(int i=0;i<chulls.size();i++) //i is the node in the skeleton graph
    {
        for(int j=0;j<chulls[i].get_nodes().size();j++) //j is the node in the convex hull
        {
            //j coincides with the j-th incident edge to the i-th node in the skeleton graph
            auto g_e = g->get_incident_edges(i)[j];

            std::pair<int,GraphEdge> key{i,g_e};
            cells[key] = std::vector<Point>();
            auto& points = cells[key];

            //compute cell points
            for(auto ch_e : chulls[i].get_incident_edges(j))
            {
                const int arc_subdiv = var_values[arc_variable(i,ch_e)];
                const EdgeDual e_dual = chulls[i].edge_dual(ch_e);
                if(points.empty())
                    points.push_back(e_dual.u);
                if((points[points.size()-1]-e_dual.u).norm()<TOL)
                    for(int k=1;k<=arc_subdiv;k++)
                        points.push_back(e_dual.get_point((k*e_dual.phi)/arc_subdiv));
                else
                {
                    assert((points[points.size()-1]-e_dual.get_point(e_dual.phi)).norm()<TOL);
                    for(int k=arc_subdiv-1;k>=0;k--)
                        points.push_back(e_dual.get_point((k*e_dual.phi)/arc_subdiv));
                }
            }
            points.pop_back(); //last point coincides with first one

            //sort cell points
            auto n = (g->get_node(g_e.j)-g->get_node(g_e.i)).normalized();
            if(i==g_e.j)
                n = -n;
            if(points[0].cross(points[1]).dot(n)<0)
                std::reverse(points.begin(),points.end());
        }
    }
}

void Scaffolder::compute_cells_match()
{
    for(auto e : g->get_edges())
    {
        auto& points1 = cells.at({e.i,e});
        auto& points2 = cells.at({e.j,e});
        int i = 0;
        double best_dist = 0.0;
        int n = points1.size();
        for(int j=0;j<n;j++)
            best_dist += (points1[j]-points2[n-1-j]).norm();
        double dist = 0.0;
        for(int k=1;k<n;k++)
        {
            dist = 0.0;
            for(int j=0;j<n;j++)
                dist += (points1[(j+k)%n]-points2[n-1-j]).norm();
            if(dist<best_dist)
            {
                best_dist = dist;
                i = k;
            }
        }
        std::vector<std::pair<int,int>> res;
        for(int j=0;j<n;j++)
            res.push_back({(j+i)%n,n-1-j});
        cells_match[e] = res;
    }
}

void Scaffolder::compute()
{
    compute_convex_hulls();

    std::stringstream mip_lp;
    setup_mip(mip_lp);

    //save model to file
    std::ofstream fout(mip_lp_file);
    fout << mip_lp.str();
    fout.close();

    solve_mip();
    read_mip_solution();

    compute_cells();
    compute_cells_match();
}

void Scaffolder::save_to_file(const std::string& fname) const
{

    std::vector<Point> points;
    std::vector<std::tuple<int,int,int,int>> quads;

    for(auto e : g->get_edges())
    {
        auto& match = cells_match.at(e);
        auto& cell1 = cells.at({e.i,e});
        auto& cell2 = cells.at({e.j,e});
        auto& base1 = g->get_node(e.i);
        auto& base2 = g->get_node(e.j);
        for(int i=0;i<match.size();i++)
        {
            auto& p1 = match[i];
            auto& p2 = match[(i+1)%match.size()];
            points.push_back(base1+cell1[p1.first]);
            points.push_back(base2+cell2[p1.second]);
            points.push_back(base2+cell2[p2.second]);
            points.push_back(base1+cell1[p2.first]);
            int k = points.size()-1;
            quads.push_back({k-3,k-2,k-1,k});
        }
    }

    std::ofstream fout(fname);
    for(auto& p : points)
    {
        fout << "v ";
        fout << p(0) << " ";
        fout << p(1) << " ";
        fout << p(2) << std::endl;
    }
    for(auto& q : quads)
    {
        fout << "f ";
        fout << std::get<0>(q) + 1 << " ";
        fout << std::get<1>(q) + 1 << " ";
        fout << std::get<2>(q) + 1 <<  " ";
        fout << std::get<3>(q) + 1 << std::endl;
    }
    fout.close();
}
