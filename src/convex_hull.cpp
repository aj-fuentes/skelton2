#include "convex_hull.h"

#include <iostream>
#include <numeric>
#include <cmath>

#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacet.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullVertex.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <libqhullcpp/QhullRidge.h>
#include <libqhullcpp/QhullError.h>

#include <libqhull_r/libqhull_r.h>


// #define DEBUG_CONVEX_HULL

std::ostream& operator <<(std::ostream& os,const EdgeDual& e)
{
    return os << "edge_dual([" << e.u.transpose() << "]--[" << e.get_point(e.phi).transpose() << "]," << ")";
}

ConvexHull::ConvexHull(std::vector<Point> points, bool compute_now) : planar(false)
{
    for(auto p : points)
    {
        const auto q = p.normalized();
        add_node(q);
        point_coords.push_back(q(0));
        point_coords.push_back(q(1));
        point_coords.push_back(q(2));
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
        edge_faces[Edge(i,j)] = std::set<int>();
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
        const UnitVector u = (nodes[0]-barycenter).normalized();
        const UnitVector n = u.cross(nodes[1]-barycenter).normalized();
        const UnitVector v = n.cross(u).normalized();

        std::vector<std::pair<double,int>> angles_idx;
        for(int i=0; i<nodes.size();++i)
        {
            auto p = nodes[i];
            angles_idx.push_back({std::atan2(v.dot(p-barycenter),u.dot(p-barycenter)),i});
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
            edge_faces[e].insert(0);
            edge_faces[e].insert(1);
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
    if(nodes.size()==0)
        throw std::logic_error("Error: zero points to compute convex hull");

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

    #ifdef DEBUG_CONVEX_HULL
        std::cout << "Number of facets " << qhull.facetList().size() << std::endl;
    #endif

    for(auto facet : qhull.facetList())
    {
        // if (not facet.isGood()) continue;

        Face new_face;
        faces.push_back(new_face);
        auto new_face_id = faces.size()-1;

        #ifdef DEBUG_CONVEX_HULL
            std::cout << "Qhull Facet" << std::endl;
            std::cout << facet << std::endl;
            std::cout << "Face id=" << new_face_id << std::endl;
        #endif

        const auto& vertices = facet.vertices();
        for(const auto& vertex : vertices)
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
        if(vertices.size()==3)
        {
            //in a triangle all vertices are connected by edges
            for (int k=0;k<3;k++)
            {
                int i = new_face[k];
                int j = new_face[(k+1) % new_face.size()];
                const Edge e(i,j);
                add_edge(e.i,e.j);
                edge_faces[e].insert(new_face_id);
            }
        }
        else
        {
            //facet must have at least 4 vertices
            if(vertices.size()<3)
            {
                std::stringstream ss;
                ss << "The face " << new_face_id << " has only " << vertices.size() << " vertices";
                throw std::logic_error(ss.str());
            }
            const auto ridges = facet.ridges();
            //facet must have at least 4 ridges/edges
            if(ridges.size()<4)
            {
                std::stringstream ss;
                ss << "The face " << new_face_id << " has " << vertices.size() << " vertices" << std::endl;
                ss << "and " << ridges.size() << " edges";
                throw std::logic_error(ss.str());
            }
            for(const auto& ridge : ridges)
            {
                auto it = ridge.vertices().begin();
                int i = (*it).point().id();
                int j = (*(++it)).point().id();
                const Edge e(i,j);
                add_edge(e.i,e.j);
                edge_faces[e].insert(new_face_id);
            }
        }
    }

    for(auto p : edge_faces)
    {
        if(p.second.size()!=2)
        {
            std::stringstream ss;
            ss << p.first << " has " << p.second.size() << " adjacente faces" << std::endl;
            throw std::logic_error(ss.str());
        }
        #ifdef DEBUG_CONVEX_HULL
            {
                std::cout << "Edge (" << p.first.i <<"," << p.first.j << ") faces= ";
                auto it = p.second.begin();
                std::cout << *it << "," << *(++it) << std::endl;
            }
        #endif
    }

    //check number of incident edges
    for(int i=0;i<incident_edges.size();i++)
        if(incident_edges[i].size()<3)
        {
            std::stringstream ss;
            ss << "Point " << i << " has only ";
            ss << incident_edges[i].size() << " incident edges";
            ss << " in a non-planar convex hull (min 3)";
            throw std::logic_error(ss.str());
        }

    sort_incident_edges();
}

void ConvexHull::sort_incident_edges() {
    if(planar) return;

    #ifdef DEBUG_CONVEX_HULL
        std::cout << "Sorting incident edges " << std::endl;
    #endif

    // for(int i=0;i<nodes.size();i++)
    // {
        // const auto& n = nodes[i];
        // auto& edges = incident_edges[i];

        // std::cout << "Incident edges number " << edges.size() << std::endl;
        // if(edges.size()==0)
        // {
        //     std::cout << "Edges incident to node " << i << " are 0" <<std::endl;
        //     std::cout << "Node i = " << n.transpose() <<std::endl;
        // }
        // const auto& e0 = edges[0];
        // const int j = e0.i==i? e0.j : e0.i;
        // const auto u = nodes[j];
        // const auto v = n.cross(u).normalized();

        // std::vector<std::pair<double,int>> angle_idxs;
        // for(int k=0;k<edges.size();k++)
        // {
        //     const auto& e = edges[k];
        //     const auto w = (nodes[e.i==i? e.j : e.i] - n).normalized();
        //     angle_idxs.push_back(std::pair<double,int>{atan2(w.dot(v),w.dot(u)),k});
        // }
        // std::sort(angle_idxs.begin(),angle_idxs.end());

        // std::vector<Edge> sorted_edges;
        // std::vector<Edge> prev_edges(edges.begin(),edges.end());
        // edges.clear();
        // for(auto p : angle_idxs)
        //     edges.push_back(prev_edges[p.second]);
    // }
    for(int i=0;i<nodes.size();i++)
    {
        const auto& n = nodes[i]; //nodes were normalized
        const auto& e0 = edges[0];
        const auto u = (nodes[e0.i==i? e0.j : e0.i]-n).normalized();
        const auto v = n.cross(u).normalized();

        #ifdef DEBUG_CONVEX_HULL
            std::cout << "Edges incident to node " << i << " are " << incident_edges[i].size() <<std::endl;
            std::cout << "Node i = " << n.transpose() <<std::endl;
        #endif

        std::sort(incident_edges[i].begin(),incident_edges[i].end(),[&,this](const Edge& e0, const Edge& e1){
            const auto w0 = nodes[e0.i==i? e0.j : e0.i]-n; //edge vectors
            const auto w1 = nodes[e1.i==i? e1.j : e1.i]-n;
            return std::atan2(w0.dot(v),w0.dot(u))<std::atan2(w1.dot(v),w1.dot(u));
        });
    }
}

EdgeDual ConvexHull::edge_dual(const Edge& e) const
{
    auto it = edge_faces.at(e).begin();
    const int i = *it;
    const int j = *(++it);
    const UnitVector u = normals[i];
    const UnitVector w = normals[j];

    if(nodes.size()==1) //dangling
    {
        if(not (e.i==e.j and e.i==0))
            throw std::logic_error("Error: wrong dangling edge in convex hull");
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
