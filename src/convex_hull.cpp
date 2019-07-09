#include "convex_hull.h"

#include <iostream>
#include <cassert>
#include <numeric>
#include <cmath>

#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacet.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullVertex.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <libqhullcpp/QhullError.h>

#include <libqhull_r/libqhull_r.h>


// #define DEBUG_CONVEX_HULL

ConvexHull::ConvexHull(std::vector<Point> points, bool compute_now)
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
        node_faces.emplace_back();
    return idx;
}

bool ConvexHull::add_edge(int i, int j)
{
    bool added = Graph::add_edge(i,j);
    if(added)
        edge_faces.emplace(Edge(i,j),0);
    return added;
}

void ConvexHull::compute_planar() {
    const Point c = std::accumulate(nodes.begin(),nodes.end(),Point(0,0,0))/nodes.size();
    const UnitVector u = (nodes[0]-c).normalized();
    const UnitVector n = u.cross(nodes[1]-c).normalized();
    const UnitVector v = n.cross(u).normalized();

    std::vector<std::pair<double,int>> angles_idx;
    for(int i=0; i<nodes.size();++i)
    {
        auto p = nodes[i];
        angles_idx.push_back({std::atan2(v.dot(p-c),u.dot(p-c)),i});
    }
    std::sort(angles_idx.begin(),angles_idx.end());

    Face face0(nodes.size()),face1(nodes.size());
    for(int i=0; i<nodes.size(); ++i) {
        face0[i] = angles_idx[i].second;
        face1[nodes.size()-i-1] = angles_idx[i].second;
        auto j = (i+1)%nodes.size();
        Edge e(i,j);
        add_edge(e.i,e.j);
        edge_faces[e].push_back(0);
        edge_faces[e].push_back(1);
    }
    normals.push_back(n);
    normals.push_back(-n);
}

void ConvexHull::compute()
{
    orgQhull::Qhull qhull;
    const double *pointCoordinates = &point_coords[0];
    const int pointCount = nodes.size();
    const int pointDimension  = 3;

    try
    {
        qhull.runQhull("",pointDimension,pointCount,pointCoordinates,"-A0.999");
    } catch(const orgQhull::QhullError& e) {
        // std::cerr << e << std::endl;
        return compute_planar();
    }

    for(auto facet : qhull.facetList())
    {
        // if (not facet.isGood()) continue;

        Face new_face;
        faces.push_back(new_face);
        auto new_face_id = faces.size()-1;

        #ifdef DEBUG_CONVEX_HULL
            std::cout << "Face id=" << new_face_id << std::endl;
        #endif

        //setup the normal of this face
        normals.emplace_back(facet.hyperplane().coordinates());

        #ifdef DEBUG_CONVEX_HULL
            std::cout << "\tNormal=" << normals[new_face_id].transpose() << std::endl;
        #endif

        for(auto vertex : facet.vertices())
        {
            #ifdef DEBUG_CONVEX_HULL
                std::cout << "\tNode id=" << vertex.point().id();
                std::cout << " [" << Point(vertex.point().coordinates()).transpose() << "]" << std::endl;
            #endif

            auto point_id = vertex.point().id();
            new_face.push_back(point_id);
            node_faces[point_id].insert(new_face_id);
        }

        //build edges of this face, and add info to edge_faces
        for (int k=0;k<new_face.size();k++)
        {
            int i = new_face[k];
            int j = new_face[(k+1) % new_face.size()];
            Edge e(i,j);
            add_edge(e.i,e.j);
            if(edge_faces[e].size()==0)
                edge_faces[e].push_back(new_face_id);
            else if(edge_faces[e][1]!=new_face_id)
                edge_faces[e].push_back(new_face_id);
            assert(edge_faces[e][0]==new_face_id or edge_faces[e][1]==new_face_id);
        }
    }
    #ifdef DEBUG_CONVEX_HULL
        for( auto kv : edge_faces)
        {
            std::cout << "Edge (" << kv.first.i <<"," << kv.first.j << ") faces= ";
            std::cout << kv.second[0] << "," << kv.second[1] << std::endl;
            assert(kv.second.size()==2);
        }
    #endif
}

