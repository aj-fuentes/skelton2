#include "scaffolder.h"

void Scaffolder::compute()
{
    for(int i=0;i<g->node_count();i++)
    {
        Point node = g->get_node(i);
        std::vector<Point> points;
        std::for_each(g->adjacent_nodes_begin(i),g->adjacent_nodes_end(i),[&node,&points,this](int j){
            points.push_back(g->get_node(j)-node);
        });
        chulls.push_back(ConvexHull(points,true));
    }



}
