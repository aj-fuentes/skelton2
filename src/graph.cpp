#include "graph.h"
#include <fstream>

std::ostream& operator <<(std::ostream& os,const Edge& e)
{
    return os << "edge(" << e.i << "," << e.j << ")";
}

int Graph::add_node(const Point& p)
{
    //TODO: check duplicates
    nodes.push_back(p);
    incident_edges.push_back(std::vector<Edge>());
    adjacent_nodes.push_back(std::set<int>());
    return nodes.size()-1;
}

bool Graph::add_edge(int i, int j)
{
    Edge e(i,j);

    //check if edge already present
    if(adjacent_nodes[i].find(j)!=adjacent_nodes[i].end())
        return false;

    edges.push_back(e);

    incident_edges[i].push_back(e);
    adjacent_nodes[i].insert(j);

    if(i!=j) //self loops possible only for dangling nodes
    {
        incident_edges[j].push_back(e);
        adjacent_nodes[j].insert(i);
    }

    return true;
}

void Graph::read_from_file(const std::string& fname)
{
    std::ifstream fin(fname);
    std::string line;
    while (std::getline(fin, line))
    {
        if(line.length()==0)
            continue;
        std::string type;
        std::istringstream lin(line);
        lin >> type;
        if(type.length()==0)
            continue;
        if(type[0]=='v')
        {
            double x,y,z;
            lin >> x >> y >> z;
            add_node(Point(x,y,z));
        } else if (type[0]=='l')
        {
            int idx;
            std::vector<int> idxs;
            while(lin >> idx)
            {
                idxs.push_back(idx);
            }
            for(int i=0;i<idxs.size()-1;i++)
                add_edge(idxs[i],idxs[i+1]);
        }
    }
    fin.close();
}
