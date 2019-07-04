#include "convex_hull.h"

#include <iostream>
#include <cassert>

#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacet.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullVertex.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <libqhullcpp/QhullRidge.h>


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
        node_faces.push_back(std::set<int>());
    return idx;
}

bool ConvexHull::add_edge(int i, int j)
{
    bool added = Graph::add_edge(i,j);
    if(added)
        edge_faces[Edge(i,j)]=std::vector<int>();
    return added;
}

void ConvexHull::compute()
{
    orgQhull::Qhull qhull;
    const double *pointCoordinates = &point_coords[0];
    int pointCount = nodes.size();
    int pointDimension  = 3;
    qhull.runQhull("",pointDimension,pointCount,pointCoordinates,"-A0.999");

    for(auto facet : qhull.facetList())
    {
        // if (not facet.isGood()) continue;

        auto new_face_id = facet.id()-1; //face IDs start at 1!!

        // std::cout << "Face id=" << new_face_id << std::endl;

        Face new_face;
        faces.push_back(new_face);

        //setup the normal of this face
        auto normal_coords = facet.hyperplane().coordinates();
        normals.push_back(UnitVector(normal_coords[0],normal_coords[1],normal_coords[2]));

        // std::cout << "\tNormal=" << normals[new_face_id].transpose() << std::endl;

        for(auto vertex : facet.vertices())
        {
            new_face.push_back(vertex.id());
            // std::cout << "\tVertex id=" << vertex.id() << std::endl;
            node_faces[vertex.id()].insert(new_face_id); //facet id starts at 1!!
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
    // for( auto kv : edge_faces)
    // {
    //     std::cout << "Edge (" << kv.first.i <<"," << kv.first.j << ") faces= ";
    //     std::cout << kv.second[0] << "," << kv.second[1] << std::endl;
    //     assert(kv.second.size()==2);
    // }
}

