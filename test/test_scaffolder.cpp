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
