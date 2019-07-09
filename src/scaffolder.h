#ifndef SCAFFOLDER_H
#define SCAFFOLDER_H

#include "base.h"
#include "convex_hull.h"

class Scaffolder;

typedef std::shared_ptr<Scaffolder> Scaffolder_ptr;

class Scaffolder {
public:
    Scaffolder(const Graph_ptr& graph) :
        g(graph)
    {}
    void compute();
    void save_to_file() const;
private:
    const Graph_ptr g;
    std::vector<ConvexHull> chulls;
};

#endif
