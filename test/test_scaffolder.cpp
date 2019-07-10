#include "catch.hpp"

#include "scaffolder.h"

TEST_CASE("Scaffolder variable utilities","[scaffolder]")
{
    REQUIRE(Scaffolder::cell_sum_variable(101,Edge(32,21))=="q_101_21_32");
    REQUIRE(Scaffolder::arc_variable(101,Edge(32,21))=="x_101_21_32");
    auto res = Scaffolder::parse_cell_sum_variable("q_101_21_32");
    REQUIRE(res.first==101);
    REQUIRE(res.second==Edge(32,21));
    auto res2 = Scaffolder::parse_cell_sum_variable("x_101_21_32");
    REQUIRE(res.first==101);
    REQUIRE(res.second==Edge(32,21));
}

TEST_CASE("Scaffolder triangle")
{
    auto g = Graph_ptr(new Graph());
    g->add_node(Point(0,0,0));
    g->add_node(Point(5,0,0));
    g->add_node(Point(0,5,0));
    g->add_edge(0,1);
    g->add_edge(0,2);
    g->add_edge(1,2);

    Scaffolder s(g);

    SECTION("Standard scaffold")
    {
        s.set_mip_lp_file("triangle_scaff.mod");
        s.set_mip_sol_file("triangle_scaff.sol");
        s.compute();
        s.save_to_file("triangle_scaff.obj");
    }

    SECTION("Standard scaffold max angle pi/3")
    {
        s.set_mip_lp_file("triangle_pi_3_scaff.mod");
        s.set_mip_sol_file("triangle_pi_3_scaff.sol");
        s.set_max_arc_angle(PI_/3.0);
        s.compute();
    }

    SECTION("Regular scaffold")
    {
        s.set_mip_lp_file("triangle_reg_scaff.mod");
        s.set_mip_sol_file("triangle_reg_scaff.sol");
        s.set_regular(true);
        s.compute();
    }
}
