#ifndef FIELD_H
#define FIELD_H

#include "base.h"

class Field
{
public:
    Field(const Skeleton_ptr& skel, const Param2d& ra, const Param2d& rb, const Param2d& rc, const Param2d& rot) :
        skel(skel), ra(ra), rb(rb), rc(rc), rot(rot), gsl_ws_size(100), max_error(1.0e-8)
    {}
    virtual double eval(const Point) const = 0;
    virtual Vector gradient_eval(const Point) const = 0;
    virtual Point shoot_ray(const Point,const UnitVector,const double) const = 0;
    virtual ~Field()
    {}
protected:
    Skeleton_ptr skel;
    Param2d ra, rb, rc, rot;
    int gsl_ws_size;
    double max_error;
};

class SegmentField : public Field
{
public:
    SegmentField(const Skeleton_ptr& skel, const Param2d ra, const Param2d rb, const Param2d rc, const Param2d rot) : Field(skel,ra,rb,rc,rot)
    {}
    ~SegmentField()
    {}
};

#endif
