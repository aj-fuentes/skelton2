#include "catch.hpp"

#include "skeleton.h"
#include "field.h"

#define VEQUAL(a,b) (((a)-(b)).norm())==Approx(0.0).margin(1e-8)

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
    }

    SECTION("Correct radii")
    {
        double a0 = Field::get_tangential_param(1.0,0.1);
        double b0 = Field::get_normal_param(1.0,0.1);

        FieldParams a = FieldParams{a0,a0};
        FieldParams b = FieldParams{b0,b0};
        FieldParams c = FieldParams{b0,b0};
        FieldParams th{0.0,0.0};

        Field_ptr field = Field_ptr(new SegmentField(seg,a,b,c,th));

        REQUIRE(field->eval(Point(-1.0,0.0,0.0))==Approx(0.1));
        REQUIRE(field->eval(Point( 6.0,0.0,0.0))==Approx(0.1));
        REQUIRE(field->eval(Point( 2.5,1.0,0.0))==Approx(0.1));
        REQUIRE(field->eval(Point( 2.5,0.0,1.0))==Approx(0.1));
    }

    SECTION("Gradient evaluation")
    {
        FieldParams a{1.0,1.0};
        FieldParams b{1.0,1.0};
        FieldParams c{1.0,1.0};
        FieldParams th{0.0,0.0};

        Field_ptr field = Field_ptr(new SegmentField(seg,a,b,c,th));

        Vector grad = field->gradient_eval(Point(0.0,0.0,0.0));
        REQUIRE(grad(0)==Approx(2.1875));
        REQUIRE(grad(1)==Approx(0.0));
        REQUIRE(grad(2)==Approx(0.0));

        grad = field->gradient_eval(Point(5.0,0.0,0.0));
        REQUIRE(grad(0)==Approx(-2.1875));
        REQUIRE(grad(1)==Approx(0.0));
        REQUIRE(grad(2)==Approx(0.0));

        grad = field->gradient_eval(Point(2.5,0.0,0.0));
        REQUIRE(grad(0)==Approx(0.0).margin(1.0e-15));
        REQUIRE(grad(1)==Approx(0.0));
        REQUIRE(grad(2)==Approx(0.0));
    }

    SECTION("Constants and params")
    {
        REQUIRE(Field::get_omega_constant(0.1)==Approx(0.549359));
        REQUIRE(Field::get_eta_constant(0.1)==Approx(0.758359));

        REQUIRE(Field::get_tangential_param(1.0,0.1)==Approx(1.0/0.549359));
        REQUIRE(Field::get_normal_param(1.0,0.1)==Approx(1.0/0.758359));
    }

    SECTION("Ray shooting")
    {
        double a0 = Field::get_tangential_param(1.0,0.1);
        double b0 = Field::get_normal_param(1.0,0.1);

        FieldParams a = FieldParams{a0,a0};
        FieldParams b = FieldParams{b0,b0};
        FieldParams c = FieldParams{b0,b0};
        FieldParams th{0.0,0.0};

        Field_ptr field = Field_ptr(new SegmentField(seg,a,b,c,th));

        Point q = field->shoot_ray(Point(2.5,0,0),UnitVector(0.0,0.0,1.0),0.1);
        REQUIRE(VEQUAL(q,Point(2.5,0,1.0)));

        q = field->shoot_ray(Point(2.5,0,0),UnitVector(0.0,1.0,0.0),0.1);
        REQUIRE(VEQUAL(q,Point(2.5,1.0,0.0)));

        q = field->shoot_ray(Point(0.0,0,0),UnitVector(-1.0,0.0,0.0),0.1);
        REQUIRE(VEQUAL(q,Point(-1.0,0,0.0)));

        q = field->shoot_ray(Point(5.0,0,0),UnitVector(1.0,0.0,0.0),0.1);
        REQUIRE(VEQUAL(q,Point(6.0,0,0.0)));
    }
}
