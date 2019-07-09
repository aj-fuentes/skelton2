#ifndef CONVEX_HULL_H
#define CONVEX_HULL_H

#include "graph.h"
#include <map>

class ConvexHull;
typedef std::shared_ptr<ConvexHull> ConvexHull_ptr;

struct EdgeDual
{
    const UnitVector u;
    const UnitVector v;
    const double phi;
    EdgeDual(UnitVector u, UnitVector v, double phi) :
        u(u), v(v), phi(phi)
    {}
};

typedef std::vector<int> Face;
class ConvexHull : public Graph {
public:
    ConvexHull(std::vector<Point>,bool=false);
    int add_node(const Point&);
    bool add_edge(int,int);
    void compute();
    EdgeDual edge_dual(const Edge) const;
    bool is_planar() const
    {
        return planar;
    }
    std::vector<Face>::const_iterator faces_begin() const
    {
        return faces.begin();
    }
    std::vector<Face>::const_iterator faces_end() const
    {
        return faces.end();
    }
private:
    void compute_planar();

    std::vector<double> point_coords; //used only for the qhull call
    //it's a list of all the coordinates of the nodes x0,y0,z0,x1,y1,z1...xn,yn,zn

    bool planar; //whether this convex hull is planar or not
    Point barycenter;

    std::vector<Face> faces;
    std::vector<UnitVector> normals; //of the faces
    std::map<const Edge,std::vector<int>> edge_faces; //two per edge

    std::vector<std::set<int>> node_faces; //faces incident to a node
};

#endif
