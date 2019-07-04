#ifndef CONVEX_HULL_H
#define CONVEX_HULL_H

#include "graph.h"
#include <libqhullcpp/Qhull.h>

class ConvexHull;
typedef std::shared_ptr<ConvexHull> ConvexHull_ptr;

typedef std::vector<int> Face;
class ConvexHull : public Graph {
public:
    ConvexHull(std::vector<Point>,bool);
    int add_node(const Point&);
    bool add_edge(int,int);
    void compute();
    std::vector<Face>::const_iterator faces_begin() const
    {
        return faces.begin();
    }
    std::vector<Face>::const_iterator faces_end() const
    {
        return faces.end();
    }
private:
    void process_qhull_results();

    std::vector<double> point_coords;
    std::vector<Face> faces;
    std::vector<UnitVector> normals;
    std::vector<std::vector<int>> edge_faces;
    std::vector<std::vector<int>> point_faces;
};

#endif
