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
        s.save_to_file("triangle_pi_3_scaff.obj");
    }

    SECTION("Regular scaffold")
    {
        s.set_mip_lp_file("triangle_reg_scaff.mod");
        s.set_mip_sol_file("triangle_reg_scaff.sol");
        s.set_regular(true);
        s.compute();
        s.save_to_file("triangle_reg_scaff.obj");
    }
}

TEST_CASE("Scaffolder 4 points")
{
    auto g = Graph_ptr(new Graph());
    g->add_node(Point(0,0,0));
    g->add_node(Point(5,0,0));
    g->add_node(Point(0,5,0));
    g->add_node(Point(0,0,5));
    g->add_edge(0,1);
    g->add_edge(1,2);
    g->add_edge(2,3);
    g->add_edge(3,0);

    Scaffolder s(g);

    SECTION("Standard scaffold")
    {
        s.set_mip_lp_file("4_points_scaff.mod");
        s.set_mip_sol_file("4_points_scaff.sol");
        s.compute();
        s.save_to_file("4_points_scaff.obj");
    }
}

TEST_CASE("Cube scaffold")
{
    auto g = Graph_ptr(new Graph());
    g->add_node(Point(0,0,0));
    g->add_node(Point(5,0,0));
    g->add_node(Point(5,5,0));
    g->add_node(Point(0,5,0));

    g->add_node(Point(0,0,5));
    g->add_node(Point(5,0,5));
    g->add_node(Point(5,5,5));
    g->add_node(Point(0,5,5));

    g->add_edge(0,1);
    g->add_edge(1,2);
    g->add_edge(2,3);
    g->add_edge(3,0);

    g->add_edge(4,5);
    g->add_edge(5,6);
    g->add_edge(6,7);
    g->add_edge(7,4);

    g->add_edge(0,4);
    g->add_edge(1,5);
    g->add_edge(2,6);
    g->add_edge(3,7);

    Scaffolder s(g);

    SECTION("Standard scaffold")
    {
        s.set_mip_lp_file("cube_scaff.mod");
        s.set_mip_sol_file("cube_scaff.sol");
        s.compute();
        s.save_to_file("cube_scaff.obj");
    }
    SECTION("Arc optimal scaffold")
    {
        s.set_mip_lp_file("cube_arc_reg_scaff.mod");
        s.set_mip_sol_file("cube_arc_reg_scaff.sol");
        s.set_regular(true);
        s.set_arc_based_optimal_solution(true);
        s.compute();
        s.save_to_file("cube_arc_reg_scaff.obj");
    }
}


TEST_CASE("Part of cube scaffold")
{
    auto g = Graph_ptr(new Graph());
    g->add_node(Point(0,0,0));
    g->add_node(Point(5,0,0));
    g->add_node(Point(0,5,0));

    g->add_node(Point(0,0,5));
    g->add_node(Point(5,0,5));
    g->add_node(Point(0,5,5));

    g->add_edge(0,1);
    g->add_edge(0,2);

    g->add_edge(3,4);
    g->add_edge(3,5);

    g->add_edge(0,3);

    Scaffolder s(g);

    SECTION("Standard scaffold")
    {
        s.set_mip_lp_file("part_cube_scaff.mod");
        s.set_mip_sol_file("part_cube_scaff.sol");
        s.compute();
        s.save_to_file("part_cube_scaff.obj");
    }
    SECTION("Regular scaffold")
    {
        s.set_mip_lp_file("part_cube_reg_scaff.mod");
        s.set_mip_sol_file("part_cube_reg_scaff.sol");
        s.compute();
        s.save_to_file("part_cube_reg_scaff.obj");
    }
    SECTION("Arc optimal scaffold")
    {
        s.set_mip_lp_file("part_cube_arc_reg_scaff.mod");
        s.set_mip_sol_file("part_cube_arc_reg_scaff.sol");
        s.set_regular(true);
        s.set_arc_based_optimal_solution(true);
        s.compute();
        s.save_to_file("part_cube_arc_reg_scaff.obj");
    }
}

TEST_CASE("Scaffolder 3sym")
{
    auto g = Graph_ptr(new Graph());
    g->add_node(Point(0.000000,0.000000,0.000000));
    g->add_node(Point(2.500000,0.000000,-5.000000));
    g->add_node(Point(2.500000,4.330127,0.000000));
    g->add_node(Point(-1.250000,2.165064,-5.000000));
    g->add_node(Point(-5.000000,0.000000,0.000000));
    g->add_node(Point(-1.250000,-2.165064,-5.000000));
    g->add_node(Point(2.500000,-4.330127,0.000000));
    g->add_node(Point(3.535534,0.000000,3.535534));
    g->add_node(Point(-1.767767,3.061862,3.535534));
    g->add_node(Point(-1.767767,-3.061862,3.535534));

    g->add_edge(0,1);
    g->add_edge(0,2);
    g->add_edge(0,3);
    g->add_edge(0,4);
    g->add_edge(0,5);
    g->add_edge(0,6);
    g->add_edge(0,7);
    g->add_edge(0,8);
    g->add_edge(0,9);
    g->add_edge(2,7);
    g->add_edge(4,8);
    g->add_edge(6,9);

    Scaffolder s(g);

    SECTION("Standard scaffold")
    {
        s.set_mip_lp_file("3sym1_scaff.mod");
        s.set_mip_sol_file("3sym1_scaff.sol");
        s.compute();
        s.save_to_file("3sym1_scaff.obj");
    }

    SECTION("Regular scaffold")
    {
        s.set_mip_lp_file("3sym1_reg_scaff.mod");
        s.set_mip_sol_file("3sym1_reg_scaff.sol");
        s.set_regular(true);
        s.compute();
        s.save_to_file("3sym1_reg_scaff.obj");
    }

    SECTION("Arc optimal scaffold")
    {
        s.set_mip_lp_file("3sym1_arc_reg_scaff.mod");
        s.set_mip_sol_file("3sym1_arc_reg_scaff.sol");
        s.set_arc_based_optimal_solution(true);
        s.set_regular(true);
        s.compute();
        s.save_to_file("3sym1_arc_reg_scaff.obj");
    }

}

TEST_CASE("Scaffolder 3-joint")
{
    auto g = Graph_ptr(new Graph());
    g->add_node(Point(0,0,0));
    g->add_node(Point(5,0,0));
    g->add_node(Point(5,5,0));
    g->add_node(Point(0,5,0));

    g->add_edge(0,1);
    g->add_edge(0,2);
    g->add_edge(0,3);

    Scaffolder s(g);

    SECTION("Standard scaffold")
    {
        s.set_mip_lp_file("3-joint_scaff.mod");
        s.set_mip_sol_file("3-joint_scaff.sol");
        s.compute();
        s.save_to_file("3-joint_scaff.obj");
    }
    SECTION("Regular scaffold")
    {
        s.set_mip_lp_file("3-joint_reg_scaff.mod");
        s.set_mip_sol_file("3-joint_reg_scaff.sol");
        s.set_regular(true);
        s.compute();
        s.save_to_file("3-joint_reg_scaff.obj");
    }
    SECTION("Arc optimal scaffold")
    {
        s.set_mip_lp_file("3-joint_arc_reg_scaff.mod");
        s.set_mip_sol_file("3-joint_arc_reg_scaff.sol");
        s.set_regular(true);
        s.set_arc_based_optimal_solution(true);
        s.compute();
        s.save_to_file("3-joint_arc_reg_scaff.obj");
    }
}


TEST_CASE("Scaffolder 4-joint")
{
    Graph_ptr g(new Graph());
    g->add_node(5*Point(0,0,0).normalized());
    g->add_node(5*Point(1,1,0).normalized());
    g->add_node(5*Point(0,1,1).normalized());
    g->add_node(5*Point(1,-1,0).normalized());
    g->add_node(5*Point(3,2,0).normalized());

    g->add_edge(0,1);
    g->add_edge(0,2);
    g->add_edge(0,3);
    g->add_edge(0,4);

    Scaffolder s(g);

    SECTION("Standard scaffold")
    {
        s.set_mip_lp_file("4-joint_scaff.mod");
        s.set_mip_sol_file("4-joint_scaff.sol");
        s.compute();
        s.save_to_file("4-joint_scaff.obj");
    }
    SECTION("Regular scaffold")
    {
        s.set_mip_lp_file("4-joint_reg_scaff.mod");
        s.set_mip_sol_file("4-joint_reg_scaff.sol");
        s.set_regular(true);
        s.compute();
        s.save_to_file("4-joint_reg_scaff.obj");
    }
    SECTION("Arc optimal scaffold")
    {
        s.set_mip_lp_file("4-joint_arc_reg_scaff.mod");
        s.set_mip_sol_file("4-joint_arc_reg_scaff.sol");
        s.set_regular(true);
        s.set_arc_based_optimal_solution(true);
        s.compute();
        s.save_to_file("4-joint_arc_reg_scaff.obj");
    }
}

TEST_CASE("Scaffolder 5-joint")
{
    Graph_ptr g(new Graph());
    g->add_node(5*Point(0,0,0).normalized());
    g->add_node(5*Point(1,1,0).normalized());
    g->add_node(5*Point(0,1,1).normalized());
    g->add_node(5*Point(1,-1,0).normalized());
    g->add_node(5*Point(3,2,0).normalized());
    g->add_node(5*Point(1,0,1).normalized());

    g->add_edge(0,1);
    g->add_edge(0,2);
    g->add_edge(0,3);
    g->add_edge(0,4);
    g->add_edge(0,5);

    Scaffolder s(g);

    SECTION("Standard scaffold")
    {
        s.set_mip_lp_file("5-joint_scaff.mod");
        s.set_mip_sol_file("5-joint_scaff.sol");
        s.compute();
        s.save_to_file("5-joint_scaff.obj");
    }
    SECTION("Regular scaffold")
    {
        s.set_mip_lp_file("5-joint_reg_scaff.mod");
        s.set_mip_sol_file("5-joint_reg_scaff.sol");
        s.set_regular(true);
        s.compute();
        s.save_to_file("5-joint_reg_scaff.obj");
    }
    SECTION("Arc optimal scaffold")
    {
        s.set_mip_lp_file("5-joint_arc_reg_scaff.mod");
        s.set_mip_sol_file("5-joint_arc_reg_scaff.sol");
        s.set_regular(true);
        s.set_arc_based_optimal_solution(true);
        s.compute();
        s.save_to_file("5-joint_arc_reg_scaff.obj");
    }
}
