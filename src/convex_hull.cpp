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

ConvexHull::ConvexHull(std::vector<Point> points, bool compute_now) : planar(false)
{
    for(auto p : points)
    {
        assert(p.norm()>TOL);
        p.normalize();
        add_node(p);
        point_coords.push_back(p(0));
        point_coords.push_back(p(1));
        point_coords.push_back(p(2));
    }
    barycenter = std::accumulate(nodes.begin(),nodes.end(),Point(0,0,0))/nodes.size();
    if(compute_now)
        compute();
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

    planar = true;

    if(nodes.size()<3)
    {
        Edge e = nodes.size()==2? Edge(0,1) : Edge(0,0);
        UnitVector n;
        if (nodes.size()==2)
        {
            //some adaptations for articulations
            n = nodes[0].cross(nodes[1]).normalized();
            faces.push_back({0,1});
            faces.push_back({0,1});
        }
        else
        {
            //this is a hack for dangling nodes
            n = Vector(1,0,0).cross(nodes[0]);
            if(n.norm()<TOL)
                n = Vector(0,1,0).cross(nodes[0]);
            n.normalize();
            faces.push_back({0,0});
            faces.push_back({0,0});
        }
        add_edge(e.i,e.j);
        normals.push_back(n);
        normals.push_back(-n);
        edge_faces[e] = {0,1};
    }
    else
    {
        const UnitVector u = nodes[0];
        const UnitVector n = u.cross(nodes[1]).normalized();
        const UnitVector v = n.cross(u).normalized();

        std::vector<std::pair<double,int>> angles_idx;
        for(int i=0; i<nodes.size();++i)
        {
            auto p = nodes[i];
            angles_idx.push_back({std::atan2(p.dot(v),p.dot(u)),i});
        }
        std::sort(angles_idx.begin(),angles_idx.end());

        Face face0(nodes.size());
        Face face1(nodes.size());
        for(int i=0; i<nodes.size(); ++i) {
            face0[i] = angles_idx[i].second;
            face1[nodes.size()-i-1] = angles_idx[i].second;
            auto j = (i+1)%nodes.size();
            Edge e(i,j);
            add_edge(e.i,e.j);
            edge_faces[e].push_back(0);
            edge_faces[e].push_back(1);
        }

        faces.push_back(face0);
        normals.push_back(n);

        faces.push_back(face1);
        normals.push_back(-n);
    }

    sort_incident_edges();
}

void ConvexHull::compute()
{
    assert(nodes.size()>0);

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

        //setup the normal of this face
        const UnitVector normal(facet.hyperplane().coordinates());
        const Point first_vertex(nodes[new_face[0]]);
        const Vector outward_vector(first_vertex-barycenter);
        if(normal.dot(outward_vector)<0) //in this case the normal is inverted
            normals.push_back(-normal);
        else
            normals.push_back(normal);

        #ifdef DEBUG_CONVEX_HULL
            std::cout << "\tNormal=" << normals[new_face_id].transpose() << std::endl;
        #endif

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
        for(auto kv : edge_faces)
        {
            std::cout << "Edge (" << kv.first.i <<"," << kv.first.j << ") faces= ";
            std::cout << kv.second[0] << "," << kv.second[1] << std::endl;
            assert(kv.second.size()==2);
        }
    #endif

    sort_incident_edges();
}

void ConvexHull::sort_incident_edges() {
    if(planar) return;
    for(int i=0;i<nodes.size();i++)
    {
        UnitVector n = nodes[i]; //nodes were normalized
        UnitVector u = nodes[*adjacent_nodes[i].begin()].normalized();
        UnitVector v = n.cross(u).normalized();

        sort(incident_edges[i].begin(),incident_edges[i].end(),[&,this](const Edge& e0, const Edge& e1){
            Vector w0 = nodes[e0.i==i? e0.j : e0.i]-n; //edge vectors
            Vector w1 = nodes[e1.i==i? e1.j : e1.i]-n;
            return std::atan2(w0.dot(v),w0.dot(u))<std::atan2(w1.dot(v),w1.dot(u));
        });
    }
}

EdgeDual ConvexHull::edge_dual(const Edge e) const
{
    const int i = edge_faces.at(e)[0];
    const int j = edge_faces.at(e)[1];
    const UnitVector u = normals[i];
    const UnitVector w = normals[j];

    if(nodes.size()==1) //dangling
    {
        assert(e.i==e.j and e.i==0);
        const UnitVector v = nodes[0].cross(u).normalized();
        return EdgeDual(u,v,2*PI_);
    }
    else if (nodes.size()==2) //articulation
    {
        const UnitVector v = (nodes[0]+nodes[1]).normalized();
        return EdgeDual(u,v,2*PI_);
    }
    else if(planar)
    {
        const Vector outward_vector = (0.5*(nodes[e.j]+nodes[e.i])-barycenter).normalized();
        UnitVector v = u.cross(nodes[e.j]-nodes[e.i]).normalized();
        if(outward_vector.dot(v)<0)
            v = -v;
        return EdgeDual(u,v,PI_);
    }
    else
    {
        const UnitVector v = (u.cross(w)).cross(u).normalized();
        const double phi = nodes.size()==2? 2*PI_ : std::atan2(w.dot(v),w.dot(u));
        return EdgeDual(u,v,phi);
    }
}
