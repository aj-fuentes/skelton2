#include <string>
#include <iostream>
#include <cstdio>

#include "scaffolder.h"

#define DEBUG_AXL_PLUGIN

int main(int argc, char** argv)
{
    std::string usage_error = "skelton_plugin <skeleton_file> <output_file_name> <temp_dir_path> <R> <max_angle>";
    if(argc!=6)
    {
        std::cerr << usage_error << std::endl;
        return 1;
    }

    std::string skeleton_file = argv[1];
    std::string output_file = argv[2];
    std::string temp_dir_path = argv[3];

    double R = std::stod(argv[4]);
    double max_angle = std::stod(argv[5]);

    auto g = Graph_ptr(new Graph());

    #ifdef DEBUG_AXL_PLUGIN
    std::cout << "Reading skeleton graph from " << skeleton_file << std::endl;
    #endif

    g->read_from_file(skeleton_file);

    #ifdef DEBUG_AXL_PLUGIN
    std::cout << "Skeleton graph read" << std::endl;
    #endif

    Scaffolder s(g);

    s.set_mip_lp_file(temp_dir_path + "__axl_plugin__.mod");
    s.set_mip_sol_file(temp_dir_path + "__axl_plugin__.sol");
    s.set_max_arc_angle(max_angle);
    s.compute();

    #ifdef DEBUG_AXL_PLUGIN
    std::cout << "Scaffold computed" << std::endl;
    #endif

    s.save_to_file(temp_dir_path + output_file,false,false,R);

    return 0;
}

