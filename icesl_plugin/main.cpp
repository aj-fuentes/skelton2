#include <string>
#include <iostream>
#include <cstdio>

#include "scaffolder.h"

// #define DEBUG_ICESL_PLUGIN

int main(int argc, char** argv)
{
    std::string skeleton_file, output_file, temp_dir_path = "./";
    std::string usage_error = "skelton <skeleton_file> <output_file_name> [<temp_dir_path>=\"./\"]";
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

    #ifdef DEBUG_ICESL_PLUGIN
    std::cout << "Readind skeleton graph from " << skeleton_file << std::endl;
    #endif

    g->read_from_file(skeleton_file);

    #ifdef DEBUG_ICESL_PLUGIN
    std::cout << "Skeleton graph read" << std::endl;
    #endif

    Scaffolder s(g);

    s.set_mip_lp_file(temp_dir_path + "__icesl_plugin__.mod");
    s.set_mip_sol_file(temp_dir_path + "__icesl_plugin__.sol");
    s.compute();

    #ifdef DEBUG_ICESL_PLUGIN
    std::cout << "Scaffold computed" << std::endl;
    #endif

    s.save_to_file(temp_dir_path + output_file);

    return 0;
}

