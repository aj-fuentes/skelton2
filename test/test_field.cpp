#include "catch.hpp"

#include "skeleton.h"
#include "field.h"

TEST_CASE("SegmentField","[field]")
{
    Point p = Point::Zero();
    double l = 5.0;
    UnitVector v = UnitVector(1.0,0.0,0.0);
    UnitVector n = UnitVector(0.0,1.0,0.0);

    Segment_ptr seg = Segment_ptr(new Segment(p,v,l,n));

    SECTION("Evaluation")
    {
        FieldParams a{1.0,1.0};
        FieldParams b{1.0,1.0};
        FieldParams c{1.0,1.0};
        FieldParams th{0.0,0.0};

        Field_ptr field = Field_ptr(new SegmentField(seg,a,b,c,th));

        REQUIRE(field->eval(Point(0.0,0.0,0.0))==Approx(1.0));
        REQUIRE(field->eval(Point(5.0,0.0,0.0))==Approx(1.0));
        REQUIRE(field->eval(Point(2.5,0.0,0.0))==Approx(2.0));

        double a0 = Field::get_tangential_param(1.0,0.1);
        double b0 = Field::get_normal_param(1.0,0.1);

        a = FieldParams{a0,a0};
        b = FieldParams{b0,b0};
        c = FieldParams{b0,b0};

        field = Field_ptr(new SegmentField(seg,a,b,c,th));

        REQUIRE(field->eval(Point(0.0,0.0,0.0))==Approx(1.0));
        REQUIRE(field->eval(Point(5.0,0.0,0.0))==Approx(1.0));
        REQUIRE(field->eval(Point(2.5,0.0,0.0))==Approx(2.0));
    }

    SECTION("Constants and params")
    {
        REQUIRE(Field::get_omega_constant(0.1)==Approx(0.549359));
        REQUIRE(Field::get_eta_constant(0.1)==Approx(0.758359));

        REQUIRE(Field::get_tangential_param(1.0,0.1)==Approx(1.0/0.549359));
        REQUIRE(Field::get_normal_param(1.0,0.1)==Approx(1.0/0.758359));
    }

}
