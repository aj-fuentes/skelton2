#include "convex_hull.h"

#include <libqhull_r/libqhull_r.h>

ConvexHull::ConvexHull(std::vector<Point> points, bool compute_now=false)
{
    for(auto p : points)
    {
        add_node(p);
        point_coords.push_back(p(0));
        point_coords.push_back(p(1));
        point_coords.push_back(p(2));
    }
    if(compute_now)
    {
        compute();
    }
}

int ConvexHull::add_node(const Point& p)
{
    int idx = Graph::add_node(p);
    if(idx==nodes.size()-1)
        point_faces.push_back(std::vector<int>());
    return idx;
}

bool ConvexHull::add_edge(int i, int j)
{
    bool added = Graph::add_edge(i,j);
    if(added)
        edge_faces.push_back(std::vector<int>());
    return added;
}

void ConvexHull::compute()
{
    // qhT qh_qh;
    // qhT *qh= &qh_qh;

    // double *points = &point_coords[0];
    // int num_points = nodes.size();
    // int dim  = 3;
    // bool ismalloc = false; //instruct QHull to not release the memory of *points

    // //setup qhull
    // qh_init_B(qh,points,num_points,dim,ismalloc);
    // qh_initflags(qh,"qhull -A0.999");
    // // qh->postmerge_cos = 0.999;

    // //compute convex hull
    // qh_qhull(qh);
    // qh_check_output(qh);

    // //TODO: Check errors
    // //qh->errexit
    // process_qhull_results(qh);
}

void ConvexHull::process_qhull_results()
{
    //save faces
    //save normals
    //save edges
    //save adjacent faces

    // facetT *facet_list;
    // facetT *facet_tail;
    // facetT *facet_next;
    // for(facetT *face=qh->facet_list;face!=qh->facet_tail;face=face->next)
    // {
    //     Face f;
    //     for(vertexT *vertex= (vertexT*)face->vertices->e[0].p; vertex!=nullptr; vertex=)
    // }
}
