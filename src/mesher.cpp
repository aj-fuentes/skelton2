#include "mesher.h"

#include <thread>

// #define DEBUG_MESHER
// #define DEBUG_MESHER_THREADS

std::pair<const std::vector<Point>&,const std::vector<Point>&> Mesher::get_cells(const std::vector<int> idxs) const
{
    const auto& i = idxs[0];
    const Edge e_i = Edge(i,idxs[1]);

    const auto& j = idxs[idxs.size()-1];
    const Edge e_j = Edge(j,idxs[idxs.size()-2]);

    return {scaff->cells.at({i,e_i}),scaff->cells.at({j,e_j})};
}

void Mesher::compute_cells_match()
{
    for(const auto& piece : pieces)
    {
        const auto& field = piece.first;

        const Frame F1t = field->skel->get_frame(0).transpose();
        const Frame F2t = field->skel->get_frame(field->skel->l).transpose();

        const auto& cells = get_cells(piece.second);

        const auto& cell_points1 = cells.first;
        const auto& cell_points2 = cells.second;

        const int n = cell_points1.size();

        std::vector<Point> points1(n);
        std::vector<Point> points2(n);

        for(int i=0;i<n;i++)
        {
            points1[i] = F1t*cell_points1[i];
            points2[i] = F2t*cell_points2[i];
        }

        cells_match.push_back(Scaffolder::math_cells(points1,points2));
    }
}

// void Mesher::compute()
// {
//     compute_cells_match();

//     const auto& g = scaff->g;
//     const auto& nodes = g->get_nodes();

//     meshlines = std::vector<std::vector<Meshline>>(pieces.size());

//     for(int i=0;i<pieces.size();i++)
//     {
//         const auto& piece = pieces[i];

//         const auto& field = piece.first;
//         const auto& idxs = piece.second;

//         const auto& cells = get_cells(idxs);

//         const auto& p = nodes[idxs[0]];
//         const auto& q = nodes[idxs[idxs.size()-1]];

//         for(const auto& match : cells_match[i])
//         {
//             const auto& u = cells.first[match.first];
//             const auto& v = cells.second[match.second];
//             meshlines[i].push_back(compute_meshline(field,p,u,q,v));
//         }
//     }

//     const auto& scaff_cells = scaff->cells;
//     for(int i=0;i<nodes.size();i++)
//     {
//         if(g->is_dangling(i))
//         {
//             const auto& p = nodes[i];
//             const Edge e = g->get_incident_edges(i)[0];
//             const UnitVector v = (p-nodes[e.i!=i? e.i : e.j]).normalized();
//             const auto& cell = scaff_cells.at({i,e});

//             auto tip_meshlines = std::vector<Meshline>(cell.size());
//             for(int j=0;j<cell.size();j++)
//                 tip_meshlines[j] = compute_tip(p,cell[j],v);

//             tips.push_back(tip_meshlines);
//         }
//     }
// }

void Mesher::compute()
{
    compute_cells_match();

    const auto& g = scaff->g;
    const auto& nodes = g->get_nodes();

    meshlines = std::vector<std::vector<Meshline>>(pieces.size());

    unsigned int n = std::thread::hardware_concurrency();
    if(n==0) n=1; //in case hardware_concurrency() fails to detect the right value

    #ifdef DEBUG_MESHER_THREADS
        std::cout << "Creating " << n << " meshline computing threads" << std::endl;
    #endif

    std::vector<std::thread> threads(n);
    for(int k=0;k<n;k++)
        threads[k] = std::thread([k,&n,&nodes,this](){
            for(int i=k;i<pieces.size();i+=n)
            {
                const auto& piece = pieces[i];

                const auto& field = piece.first;
                const auto& idxs = piece.second;

                const auto& cells = get_cells(idxs);

                const auto& p = nodes[idxs[0]];
                const auto& q = nodes[idxs[idxs.size()-1]];

                for(const auto& match : cells_match[i])
                {
                    const auto& u = cells.first[match.first];
                    const auto& v = cells.second[match.second];
                    meshlines[i].push_back(compute_meshline(field,p,u,q,v));
                }
            }
        });

    #ifdef DEBUG_MESHER_THREADS
        std::cout << "Joining threads" << std::endl;
    #endif

    for(auto& thread : threads)
        thread.join();

    #ifdef DEBUG_MESHER_THREADS
        std::cout << "All threads joined" << std::endl;
    #endif

    const auto& scaff_cells = scaff->cells;
    for(int i=0;i<nodes.size();i++)
    {
        if(g->is_dangling(i))
        {
            const auto& p = nodes[i];
            const Edge e = g->get_incident_edges(i)[0];
            const UnitVector v = (p-nodes[e.i!=i? e.i : e.j]).normalized();
            const auto& cell = scaff_cells.at({i,e});

            auto tip_meshlines = std::vector<Meshline>(cell.size());
            for(int j=0;j<cell.size();j++)
                tip_meshlines[j] = compute_tip(p,cell[j],v);

            tips.push_back(tip_meshlines);
        }
    }
}

Meshline Mesher::compute_meshline(const Field_ptr& field, const Point& p, const UnitVector& u, const Point& q, const UnitVector& v)
{

    int n = std::max(this->num_quads,1);
    if(max_quad_len>TOL)
        n = std::max(int(field->skel->l/max_quad_len),n);


    //frames at extremities of the skeleton
    const Frame F0 = field->skel->get_frame(0);
    const Frame F1 = field->skel->get_frame(field->skel->l);

    const UnitVector w0 = F0.transpose()*u;
    const UnitVector w1 = F1.transpose()*v;

    std::vector<Point> points(n+1);

    const double l = field->skel->l;
    for(int i=0;i<=n;i++)
    {
        const double t = ((double)i)*l/((double)n);
        const UnitVector wt = ((l-t)*w0 + t*w1).normalized();
        const double th = ((l-t)*field->th[0] + t*field->th[1])/l;
        const double cth = cos(th);
        const double sth = sin(th);
        const Frame F = field->skel->get_frame(t);
        Matrix R;
        R << 1,  0,   0,
             0,cth,-sth,
             0,sth, cth;
        const UnitVector w = F*R*wt;

        const Point pq = field->skel->get_point(t);

        points[i] = this->global_field->shoot_ray(pq,w,lv);
    }

    return points;
}

std::vector<Point> Mesher::compute_tip(const Point& p, const UnitVector& u, const UnitVector& v)
{

    int n = std::max(this->num_quads_tip,1);
    std::vector<Point> points(n+1);

    for(int i=0;i<=n;i++)
    {
        const UnitVector w = (((n-i)*u + i*v)/n).normalized();
        points[i] = this->global_field->shoot_ray(p,w,lv);
    }

    return points;
}

void Mesher::save_to_file(const std::string& fname) const
{

    PointIndexer indexer;
    std::vector<std::tuple<int,int,int,int>> quads;
    std::vector<std::tuple<int,int,int>> tris;

    #ifdef DEBUG_MESHER
        std::cout << "Number of pieces " << pieces.size() << std::endl;
        std::cout << "Number of meshline groups " << meshlines.size() << std::endl;
        std::cout << "Saving piece meshes" << std::endl;
    #endif

    for(const auto& mls : meshlines)
    {
        const int n = mls.size();
        #ifdef DEBUG_MESHER
            std::cout << "Number meshline in this group " << n << std::endl;
        #endif
        for(int i=0;i<n;i++)
        {
            const auto& ml1 = mls[i];
            const auto& ml2 = mls[(i+1)%n];

            const int nq = ml1.size();
            for(int j=0;j<nq-1;j++)
            {
                #ifdef DEBUG_MESHER
                    std::cout << "Creating quad " << j << std::endl;
                #endif

                const auto& p0 = ml1[j];
                const auto& p1 = ml1[j+1];
                const auto& p2 = ml2[j+1];
                const auto& p3 = ml2[j];

                quads.push_back({
                    indexer.index(p0),
                    indexer.index(p1),
                    indexer.index(p2),
                    indexer.index(p3),
                });
            }
        }
    }

    #ifdef DEBUG_MESHER
        std::cout << "Saving tips " << std::endl;
        std::cout << "Tips groups " << tips.size() << std::endl;
    #endif

    for(const auto& mls : tips)
    {
        const int n = mls.size();

        #ifdef DEBUG_MESHER
            std::cout << "Number meshline in this group " << n << std::endl;
        #endif

        for(int i=0;i<n;i++)
        {
            const auto& ml1 = mls[i];
            const auto& ml2 = mls[(i+1)%n];

            #ifdef DEBUG_MESHER
                if(ml1.size()!=ml2.size())
                    throw std::logic_error("Error: tip meshlines does not have same number of points");
            #endif

            const int nq = ml1.size()-1;
            for(int j=0;j<nq-1;j++)
            {
                #ifdef DEBUG_MESHER
                    std::cout << "Creating tip quad " << i << std::endl;
                #endif
                const auto& p0 = ml1[j];
                const auto& p1 = ml1[j+1];
                const auto& p2 = ml2[j+1];
                const auto& p3 = ml2[j];

                quads.push_back({
                    indexer.index(p3),
                    indexer.index(p2),
                    indexer.index(p1),
                    indexer.index(p0),
                });
            }

            const auto& q0 = ml1[nq-1];
            const auto& q1 = ml1[nq];
            const auto& q2 = ml2[nq-1];
            tris.push_back({
                indexer.index(q0),
                indexer.index(q2),
                indexer.index(q1)
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

    fout.close();
}
