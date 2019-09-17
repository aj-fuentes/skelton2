#include "graph.h"

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
    {
        // std::cerr << "Warning: duplicated edge in graph " << e << std::endl;
        return false;
    }

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

    std::vector<Point> points_read;
    std::vector<Edge> edges_read;
    PointIndexer indexer;

    std::ifstream fin(fname);

    if(not fin.good())
    {
        std::cerr << "Could not read skeleton file " << fname << std::endl;
    }
    else
    {
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
                points_read.push_back(Point(x,y,z));
            } else if (type[0]=='l')
            {
                int idx;
                std::vector<int> idxs;
                while(lin >> idx)
                    idxs.push_back(idx-1);
                for(int i=0;i<idxs.size()-1;i++)
                    edges_read.push_back(Edge(idxs[i],idxs[i+1]));
            }
        }
    }

    fin.close();

    for(const auto& p : points_read)
        indexer.index(p);
    for(const auto& p : indexer.get_points())
        add_node(p);
    for(const auto& e : edges_read)
        add_edge(indexer.index(points_read[e.i]),indexer.index(points_read[e.j]));

    check_graph();
}

void Graph::check_graph()
{

    //check for duplicated points
    PointIndexer indexer;
    for(int i=0;i<nodes.size();i++)
    {
        const int idx = indexer.index(nodes[i]);
        if(idx!=i)
        {
            std::stringstream ss;
            ss << "Error: duplicated nodes " << std::endl;
            ss << "The node " << i << "[" << nodes[i].transpose() <<"]";
            ss << " is already in the graph as node " << idx;
            throw std::domain_error(ss.str());
        }
    }
}
