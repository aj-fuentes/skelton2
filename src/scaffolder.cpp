#include "scaffolder.h"

#include <deque>

#include <glpk.h>

// #define DEBUG_SCAFFOLDER

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
        chulls.emplace_back(points,true);
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
        std::string mult = g->is_joint(i)? "2*" : "";
        auto ch = chulls[i];
        for(auto e : ch.get_edges())
        {
            if(arc_based_optimal_solution)
            {
                std::stringstream ss;
                ss << 1.0/ch.edge_dual(e).phi << "*";
                mult = ss.str();
            }
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
    //delete model and solution files
    remove(mip_lp_file.c_str());
    remove(mip_sol_file.c_str());
}

void Scaffolder::compute_cell(int i, int j)
{
    //i is the node in the skeleton graph
    //j is the node in the convex hull
    //j coincides with the j-th incident edge to the i-th node in the skeleton graph
    auto g_e = g->get_incident_edges(i)[j];

    std::pair<int,GraphEdge> key{i,g_e};
    cells[key] = std::vector<Point>();

    //compute cell points
    std::deque<Point> c_points;
    for(auto ch_e : chulls[i].get_incident_edges(j))
    {
        const int arc_subdiv = var_values[arc_variable(i,ch_e)];
        const EdgeDual e_dual = chulls[i].edge_dual(ch_e);

        //compute points on dual arc
        std::deque<Point> e_points;
        for(int k=0;k<=arc_subdiv;k++)
            e_points.push_back(e_dual.get_point((k*e_dual.phi)/arc_subdiv));

        if(c_points.empty())
        {
            for(const auto& p : e_points)
                c_points.push_back(p);
        }
        else
        {
            //first and last points on cell and arc
            const auto& first_c = c_points[0];
            const auto& last_c = c_points[c_points.size()-1];
            const auto& first_e = e_points[0];
            const auto& last_e = e_points[e_points.size()-1];

            //compare points
            const bool ff = (first_c-first_e).norm()<TOL;
            const bool fl = (first_c-last_e).norm()<TOL;
            const bool lf = (last_c-first_e).norm()<TOL;
            const bool ll = (last_c-last_e).norm()<TOL;

            //assert that first or last point of next arc is equal to first or last point of cell
            //(two of these are true for the last edge/arc)
            if(not (ff or fl or lf or ll))
            {
                std::stringstream ss;
                ss << "Error: cannot construct closed cell (arcs not connected) " << std::endl;
                ss << "Edge list " << std::endl;
                for(auto out_e : chulls[i].get_incident_edges(j))
                    ss << out_e << ((out_e==ch_e)? "* " : " ") << chulls[i].edge_dual(out_e) << std::endl;

                throw std::logic_error(ss.str());
            }
            #ifdef DEBUG_SCAFFOLDER
                std::cout << "ff=" << ff << " ";
                std::cout << "fl=" << fl << " ";
                std::cout << "lf=" << lf << " ";
                std::cout << "ll=" << ll << std::endl;
                std::cout << "Points in cell" << std::endl;
                for(const auto& p : c_points)
                    std::cout << p.transpose() << std::endl;
                std::cout << "Points in arc" << std::endl;
                for(const auto& p : e_points)
                    std::cout << p.transpose() << std::endl;
            #endif

            //repeated code in the ifs below to correctly handle
            //the last edge (two paths are true)
            if(ff)
            {
                #ifdef DEBUG_SCAFFOLDER
                    std::cout << "Doing ff (only one must be done)" << std::endl;
                #endif
                //cel *---------+
                //arc *------x
                e_points.pop_front(); //fist point of arc is already in cell
                //arc  ------x
                std::move(e_points.begin(),e_points.end(),std::front_inserter(c_points));
                //cell x-------*-------+
            } else if(ll)
            {
                #ifdef DEBUG_SCAFFOLDER
                    std::cout << "Doing ll (only one must be done)" << std::endl;
                #endif
                //cel *---------+
                //arc    x------+
                e_points.pop_back(); //last point is already in cell
                //arc    x------
                std::move(e_points.rbegin(),e_points.rend(),std::back_inserter(c_points));
                //cell *---------+-------x
            } else if(lf)
            {
                #ifdef DEBUG_SCAFFOLDER
                    std::cout << "Doing lf (only one must be done)" << std::endl;
                #endif
                //cel *---------+
                //arc           +------x
                e_points.pop_front(); //fist point of arc is already in cell
                //arc            ------x
                std::move(e_points.begin(),e_points.end(),std::back_inserter(c_points));
                //cell *---------+-------x
            } else //fl=true
            {
                #ifdef DEBUG_SCAFFOLDER
                    std::cout << "Doing fl (only one must be done)" << std::endl;
                #endif
                //cell       *---------+
                //arc x------*
                e_points.pop_back(); //last point of arc is already in cell
                //arc x------
                std::move(e_points.rbegin(),e_points.rend(),std::front_inserter(c_points));
                //cell x-----*-------+
            }
        }
    }

    auto& points = cells[key];

    //fill points of this cell
    for(const auto& p : c_points)
        points.push_back(p);

    //assert last point coincides with first one
    if((points[points.size()-1]-points[0]).norm()>TOL)
    {
        std::stringstream ss;
        ss << "Error: cannot construct closed cell (last point not equal to first point) " << std::endl;
        ss << "Edge list " << std::endl;
        for(auto out_e : chulls[i].get_incident_edges(j))
            ss << out_e << " " << chulls[i].edge_dual(out_e) << std::endl;
        ss << "Cell points " << std::endl;
        for(auto p : points)
            ss << p.transpose() << std::endl;

        throw std::logic_error(ss.str());
    }
    points.pop_back(); //last point coincides with first one

    if(points.size()<3)
        throw std::logic_error("Error: cell must have at least three points");

    //sort cell points
    auto n = (g->get_node(g_e.j)-g->get_node(g_e.i)).normalized();
    if(i==g_e.j)
        n = -n;

    if(points[0].cross(points[1]).dot(n)>0)
        std::reverse(points.begin(),points.end());
}

void Scaffolder::compute_cells()
{
    //we do a first pass over the cells to compute non-danglings
    //then for danglings we project the connected cell, unless it is also a dangling

    std::vector<int> dangling_idxs;
    for(int i=0;i<chulls.size();i++) //i is the node in the skeleton graph
    {
        if(g->is_dangling(i)) //danglings processed later
            dangling_idxs.push_back(i);
        else
            for(int j=0;j<chulls[i].get_nodes().size();j++) //j is the node in the convex hull
                compute_cell(i,j);
    }
    for(int i : dangling_idxs)
    {
        const int j = 0; //index in the convex hull is always 0 for danlings
        const auto& e = g->get_incident_edges(i)[0];
        const int k = e.j!=i? e.j : e.i;//the node connected with this dangling
        if(g->is_dangling(k)) //if dangling, just keep the standard computation
            compute_cell(i,j);
        else
        {
            //take connected cell and project onto this node
            std::vector<Point> points;
            const UnitVector n = (g->get_node(k)-g->get_node(i)).normalized();
            for(const auto& p : cells.at({k,e}))
            {
                points.push_back(p-p.dot(n)*n);
            }
            //reverse the order of the projected cells to match the current node
            std::reverse(points.begin(),points.end());
            cells[std::pair<int,GraphEdge>{i,e}] = points;
        }
    }
}

std::vector<std::pair<int,int>> Scaffolder::math_cells(const std::vector<Point>& points1,
    const std::vector<Point>& points2, const Vector ev)
{
    int n = points1.size();

    //verify equal number of points
    if(n!=points2.size())
        throw std::logic_error("Erro: Can't match cells, they don't have the same number of points");

    if(n<3)
        throw std::logic_error("Erro: Each cell must have at least three points");

    //verify order of cells
    const UnitVector ev_n = ev.normalized();
    for(int j=1;j<n;j++)
    {
        if(points1[j].cross(points1[j-1]).dot(ev_n)<0)
            throw std::logic_error("Error in cell order");
        if(points2[j].cross(points2[j-1]).dot(ev_n)>0)
            throw std::logic_error("Error in cell order");
    }

    int i = 0;
    double best_dist = 0.0;
    //compute first distance candidate
    for(int j=0;j<n;j++)
        best_dist += (points1[j]-points2[n-1-j]+ev).norm();

    //find best distance
    double dist = 0.0;
    for(int k=1;k<n;k++)
    {
        dist = 0.0;
        for(int j=0;j<n;j++)
            dist += (points1[(j+k)%n]-points2[n-1-j]+ev).norm();
        if(dist<best_dist)
        {
            best_dist = dist;
            i = k;
        }
    }

    std::vector<std::pair<int,int>> res(n);
    for(int j=0;j<n;j++)
        res[j] = {(j+i)%n,n-1-j};

    return res;
}

void Scaffolder::compute_cells_match()
{
    for(auto e : g->get_edges())
    {
        const auto& points1 = cells.at({e.i,e});
        const auto& points2 = cells.at({e.j,e});
        //this vector is to move the points sufficiently far away to compute distances
        const auto ev_n = ((g->get_node(e.j)-g->get_node(e.i)).normalized());
        const auto ev = 5.0*ev_n;

        cells_match[e] = math_cells(points1,points2,ev);
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

void Scaffolder::save_to_file(const std::string& fname,bool triangulate,bool save_skeleton) const
{

    PointIndexer indexer;

    std::vector<std::tuple<int,int,int,int>> quads;
    std::vector<std::tuple<int,int,int>> tris;
    std::vector<std::pair<int,int>> skel;

    for(auto e : g->get_edges())
    {
        auto& match = cells_match.at(e);
        auto& cell1 = cells.at({e.i,e});
        auto& cell2 = cells.at({e.j,e});
        auto& base1 = g->get_node(e.i);
        auto& base2 = g->get_node(e.j);
        for(int i=0;i<match.size();i++)
        {
            auto& m1 = match[i];
            auto& m2 = match[(i+1)%match.size()];

            const Point p0 = base1+cell1[m1.first];
            const Point p1 = base2+cell2[m1.second];
            const Point p2 = base2+cell2[m2.second];
            const Point p3 = base1+cell1[m2.first];

            if(triangulate)
            {
                //decide what diagonal to take
                //assume p0 p1 p2 is a triangle, hence diagonal p2-p0
                const UnitVector n = (p1-p0).cross(p2-p0).normalized();
                if(n.dot(p3-p0)<0) //use this diagonal
                {
                    tris.push_back({
                        indexer.index(p0),
                        indexer.index(p1),
                        indexer.index(p2)
                    });
                    tris.push_back({
                        indexer.index(p0),
                        indexer.index(p2),
                        indexer.index(p3)
                    });
                }
                else //take the other diagonal, p1-p3
                {
                    tris.push_back({
                        indexer.index(p0),
                        indexer.index(p1),
                        indexer.index(p3)
                    });
                    tris.push_back({
                        indexer.index(p1),
                        indexer.index(p2),
                        indexer.index(p3)
                    });
                }
            }
            else //save the quads
            {
                quads.push_back({
                    indexer.index(p0),
                    indexer.index(p1),
                    indexer.index(p2),
                    indexer.index(p3),
                });

            }
        }
        if(g->is_dangling(e.i)) //for dangling nodes we save the the triangles at tips
        {
            for(int i=0;i<cell1.size();i++)
            {
                tris.push_back({
                    indexer.index(base1+cell1[i]),
                    indexer.index(base1+cell1[(i+1)%cell1.size()]),
                    indexer.index(base1+(g->get_node(e.i)-g->get_node(e.j)).normalized())
                });
            }
        }
        if(g->is_dangling(e.j)) //for dangling nodes we save the triangles at tips
        {
            for(int i=0;i<cell2.size();i++)
            {
                tris.push_back({
                    indexer.index(base2+cell2[i]),
                    indexer.index(base2+cell2[(i+1)%cell2.size()]),
                    indexer.index(base2+(g->get_node(e.j)-g->get_node(e.i)).normalized())
                });
            }
        }
        if(save_skeleton)
        {
            skel.push_back({
                indexer.index(g->get_node(e.i)),
                indexer.index(g->get_node(e.j))
            });
        }
    }

    //write everything to the file
    std::ofstream fout(fname);
    fout << std::fixed << std::setprecision(5);
    //first the points
    for(const auto& p : indexer.get_points())
    {
        fout << "v ";
        fout << p(0) << " ";
        fout << p(1) << " ";
        fout << p(2) << std::endl;
    }
    //then the quads
    for(const auto& q : quads) //if there are quads
    {
        fout << "f ";
        fout << std::get<0>(q) + 1 << " ";
        fout << std::get<1>(q) + 1 << " ";
        fout << std::get<2>(q) + 1 << " ";
        fout << std::get<3>(q) + 1 << std::endl;
    }
    //then triangles
    for(const auto& t : tris)
    {
        fout << "f ";
        fout << std::get<0>(t) + 1 << " ";
        fout << std::get<1>(t) + 1 << " ";
        fout << std::get<2>(t) + 1 << std::endl;
    }
    //finally save the skeleton, if any
    for(const auto& s : skel)
    {
        fout << "l ";
        fout << s.first + 1 << " ";
        fout << s.second + 1 << std::endl;
    }

    fout.close();
}
