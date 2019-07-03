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
    FieldParams a{1.0,1.0};
    FieldParams b{1.0,1.0};
    FieldParams c{1.0,1.0};
    FieldParams th{0.0,0.0};

    SECTION("Constructor")
    {
        Field_ptr field = Field_ptr(new SegmentField(seg,a,b,c,th));
    }

    SECTION("Evalutaion")
    {

    }
}
