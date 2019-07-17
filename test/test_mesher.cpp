#include "catch.hpp"

#include "mesher.h"

TEST_CASE("Mesher segment")
{
    const Point p(0,0,0);
    const UnitVector v(1,0,0);
    const double l = 5;
    const UnitVector n(0,1,0);
    const UnitVector y_(0,1,0);

    auto seg = Segment_ptr(new Segment(p,v,l,n));

    const Point q = seg->get_end_point();

    auto g = Graph_ptr(new Graph());
    g->add_node(p);
    g->add_node(q);
    g->add_edge(0,1);

    auto scaff = Scaffolder_ptr(new Scaffolder(g));

    auto field = Field_ptr(new SegmentField(seg,{1,1},{1,1},{1,1},{0,0}));
    PiecesParam pieces{{field,{0,1}}};

    auto mesher = Mesher_ptr(new Mesher(scaff,field,pieces,0.1));

    SECTION("Mesher meshlines")
    {
        mesher->num_quads = 2;
        auto points = mesher->compute_meshline(field,p,n,q,n);
        REQUIRE(points.size()==mesher->num_quads+1);
        for(auto p : points)
            REQUIRE(field->eval(p)==Approx(0.1).margin(1e-8));

        mesher->num_quads = 10;
        points = mesher->compute_meshline(field,p,y_,q,n);
        REQUIRE(points.size()==mesher->num_quads+1);
        for(auto p : points)
            REQUIRE(field->eval(p)==Approx(0.1).margin(1e-8));

        mesher->num_quads = 15;
        UnitVector u = Vector(5,-1,4).normalized();
        UnitVector v = Vector(3,-2,2).normalized();
        points = mesher->compute_meshline(field,p,u,q,v);
        REQUIRE(points.size()==mesher->num_quads+1);
        for(auto p : points)
            REQUIRE(field->eval(p)==Approx(0.1).margin(1e-8));
    }

    SECTION("Mesher mesh tips")
    {
        mesher->num_quads_tip = 2;
        auto points = mesher->compute_tip(p,n,-v);
        REQUIRE(points.size()==mesher->num_quads_tip+1);
        for(auto p : points)
            REQUIRE(field->eval(p)==Approx(0.1).margin(1e-8));
        auto lastp = (points)[points.size()-1];

        points = mesher->compute_tip(q,n,v);
        REQUIRE(points.size()==mesher->num_quads_tip+1);
        for(auto p : points)
            REQUIRE(field->eval(p)==Approx(0.1).margin(1e-8));

        mesher->num_quads_tip = 10;
        points = mesher->compute_tip(p,y_,-v);
        REQUIRE(points.size()==mesher->num_quads_tip+1);
        for(auto p : points)
            REQUIRE(field->eval(p)==Approx(0.1).margin(1e-8));

        mesher->num_quads_tip = 10;
        UnitVector y_(0,1,0);
        points = mesher->compute_tip(q,y_,v);
        REQUIRE(points.size()==mesher->num_quads_tip+1);
        for(auto p : points)
            REQUIRE(field->eval(p)==Approx(0.1).margin(1e-8));
    }

    SECTION("Mesher save segment")
    {
        mesher->num_quads_tip = 8;
        mesher->num_quads = 10;

        scaff->compute();
        mesher->compute();

        mesher->save_to_file("segment_mesh.obj");
    }

    SECTION("Mesher save minimal segment")
    {
        mesher->num_quads_tip = 1;
        mesher->num_quads = 1;

        scaff->compute();
        mesher->compute();

        mesher->save_to_file("segment_min_mesh.obj");
    }

    SECTION("Mesher save good segment")
    {
        scaff->set_max_arc_angle(PI_/10);

        mesher->num_quads_tip = 8;
        mesher->max_quad_len = 0.2;

        scaff->compute();
        mesher->compute();

        mesher->save_to_file("segment_good_mesh.obj");
    }

    SECTION("Mesher save long quad mesh")
    {
        scaff->set_max_arc_angle(PI_/10);

        mesher->num_quads_tip = 8;
        mesher->max_quad_len = 10;

        scaff->compute();
        mesher->compute();

        mesher->save_to_file("segment_long_quad_mesh.obj");
    }
}