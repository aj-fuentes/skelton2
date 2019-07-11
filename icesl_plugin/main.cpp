#include <string>
#include <iostream>
#include <cstdio>

#include "scaffolder.h"

int main(int argc, char** argv)
{
    std::string skeleton_file, output_file, temp_dir_path = "./";
    std::string usage_error = "skelton <skeleton_file> <output_file> <temp_dir_path>";
    if(argc<3)
    {
        std::cerr << usage_error << std::endl;
        return 1;
    }

    skeleton_file = argv[1];
    output_file = argv[2];
    if(argc>3)
        temp_dir_path = argv[3];
    if(temp_dir_path[temp_dir_path.length()-1]!='/')
        temp_dir_path = temp_dir_path + "/";

    auto g = Graph_ptr(new Graph());
    g->add_node(Point(0,0,0));
    g->add_node(Point(5,0,0));
    g->add_node(Point(0,5,0));
    g->add_edge(0,1);
    g->add_edge(0,2);
    g->add_edge(1,2);

    Scaffolder s(g);

    s.set_mip_lp_file(temp_dir_path + "__icesl_plugin__.mod");
    s.set_mip_sol_file(temp_dir_path + "__icesl_plugin__.sol");
    s.compute();

    s.save_to_file(temp_dir_path + "icesl_plugin.obj");

    return 0;
}

