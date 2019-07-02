#ifndef SKELETON_H
#define SKELETON_H

#include "base.h"

class Skeleton {
public:
    Skeleton (const double l) : l(l)
    {}
    virtual Point get_point(const double) const = 0;
    virtual UnitVector get_tangent(const double) const = 0;
    virtual UnitVector get_normal(const double) const = 0;
    virtual UnitVector get_binormal(const double) const = 0;
    virtual double get_distance(const Point) const = 0;
    Point get_start_point() const
    {
        return get_point(0.0);
    }
    Point get_end_point() const
    {
        return get_point(l);
    }
    Frame get_frame(const double t) const {
        Frame f;
        f << get_tangent(t), get_normal(t), get_binormal(t);
        return f;
    }
    virtual ~Skeleton()
    {}
protected:
    double l;
};

class Segment : public Skeleton
{
public:
    Segment(const Point, const UnitVector, const double, const UnitVector);
    Point get_point(const double) const;
    UnitVector get_tangent(const double) const;
    UnitVector get_normal(const double) const;
    UnitVector get_binormal(const double) const;
    double get_distance(const Point) const;
    ~Segment()
    {}
private:
    Point p;
    UnitVector v,n,b;
};

#endif
