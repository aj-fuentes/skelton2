/** @file convex_hull.h Convex hull computation with extra functionality for
 * scaffolding.
 *
 * @author     Alvaro Fuentes
 */

#ifndef CONVEX_HULL_H
#define CONVEX_HULL_H

#include "graph.h"

class ConvexHull;
typedef std::shared_ptr<ConvexHull> ConvexHull_ptr;

/**
 * @brief      Dual information for edges in the convex hull.
 *
 *             The dual of an edge is an arc of circle on the plane
 *             perpendicular to the edge. It spans from the two outward normals
 *             of the faces defining the edge. In the case of 2D convex hulls,
 *             the dual is always an arc of great circle of length #PI_.
 */
struct EdgeDual
{
    /**
     * First unit vector defining the plane where the dual arc sits.
     *
     * The dual arc starts in a point on the ray defined by this vector. The two
     * unit vector defining the plane of the arc are orthonormal.
     *
     * @see        v
     */
    UnitVector u;

    /**
     * Second unit vector defining the plane where the dual arc sits.
     *
     * The two unit vector defining the plane of the arc are orthonormal.
     *
     * @see        u
     */
    UnitVector v;
    double phi;
    EdgeDual(UnitVector u, UnitVector v, double phi) :
        u(u), v(v), phi(phi)
    {
        if(phi<=0)
            throw std::logic_error("Error: edge dual with zero or negative angle span");
    }
    /**
     * @brief      Gets a point in the dual arc.
     *
     * @param[in]  t     Angle of the point.
     *
     * @return     The point on the arc.
     */
    Point get_point(double t) const
    {
        return u*cos(t) + v*sin(t);
    }

    friend std::ostream& operator <<(std::ostream&,const EdgeDual&);
};

/**
 * Face on the convex hull.
 */
typedef std::vector<int> Face;


/**
 * @brief      Class for convex hull computation.
 *
 *             This class handles correctly the convex hull of the intersection
 *             points on the sphere at joints. It is capable of computing 3D and
 *             2D convex hulls, as well as degenerated cases like in
 *             articulations or dangling nodes.
 *
 * @todo       Fix this class: it is not a Graph, it uses a graph.
 */
class ConvexHull {
public:

    /**
     * @brief      Constructor for convex hull objects
     *
     *             The input must be a set of intersection points on the surface
     *             of the unit sphere. Future implementations may check for
     *             this. In particular, since all the points are on the surface
     *             of the sphere, all of them are going to be vertices of the
     *             convex hull.
     *
     * @param[in]  points       Intersection points on the unit sphere.
     * @param[in]  compute_now  Computes the convex hull in the constructor.
     */
    ConvexHull(std::vector<Point> points,bool compute_now=false);

    /**
     * @brief      Compute the convex hull.
     */
    void compute();

    /**
     * @brief      Gets the dual of an edge on the convex hull.
     *
     * @param[in]  e     Edge of the convex hull.
     *
     * @return     Dual of the edge e.
     */
    EdgeDual edge_dual(const Edge& e) const;

    /**
     * @brief      Determines if this convex hull is planar.
     *
     * @return     True if this is a planar convex hull, False otherwise.
     */
    bool is_planar() const
    {
        return planar;
    }

    /**
     * @brief      Gets the nodes of the convex hull.
     *
     * @return     The nodes.
     */
    const std::vector<Point>& get_nodes() const
    {
        return g.get_nodes();
    }

    /**
     * @brief      Gets the edges of the convex hull.
     *
     * @return     The edges.
     */
    const std::vector<Edge>& get_edges() const
    {
        return g.get_edges();
    }

    /**
     * @brief      Gets the incident edges to the node at the given index.
     *
     * @param[in]  i     Index of the node.
     *
     * @return     The incident edges.
     */
    const std::vector<Edge>& get_incident_edges(int i) const
    {
        return g.get_incident_edges(i);
    }

    /**
     * @brief      Returns a node given the index.
     *
     * @param[in]  i     index of the node
     *
     * @return     The node.
     */
    Point get_node(int i) const
    {
        return g.get_node(i);
    }

private:

    /**
     * @brief      Adds a node to the convex hull graph.
     *
     * @param[in]  p     Node to be added.
     *
     * @return     The index of the node in the graph.
     */
    int add_node(const Point& p)
    {
       return g.add_node(p);
    }

    /**
     * @brief      Adds an edge to the convex hull graph.
     *
     * @param[in]  i     First index for the edge.
     * @param[in]  j     Second index for the edge.
     *
     * @return     Whether the edge was added or was already present in the graph.
     */
    bool add_edge(int i,int j)
    {
        if(g.add_edge(i,j))
        {
            edge_faces[Edge(i,j)] = std::set<int>();
            return true;
        }
        return false;
    }

    /**
     * @brief      Internal method to handle the computation of planar convex hulls.
     */
    void compute_planar();

    /**
     * @brief      Internal method to sort (counterclockwise) the incident edges
     *             of a node in the convex hull.
     */
    void sort_incident_edges();

    Graph g; //graph of the convex hull

    std::vector<double> point_coords; //used only for the qhull call
    //it's a list of all the coordinates of the nodes x0,y0,z0,x1,y1,z1...xn,yn,zn

    bool planar; //whether this convex hull is planar or not.
    Point barycenter; //a point INSIDE the convex hull.

    std::vector<Face> faces;
    std::vector<UnitVector> normals; //of the faces
    std::map<Edge,std::set<int>> edge_faces; //two per edge

};

#endif
