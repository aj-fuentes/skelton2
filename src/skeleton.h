#ifndef SKELETON_H
#define SKELETON_H

#include "base.h"

class Skeleton;
class Segment;
class Arc;
typedef std::shared_ptr<Skeleton> Skeleton_ptr;
typedef std::shared_ptr<Segment> Segment_ptr;
typedef std::shared_ptr<Arc> Arc_ptr;

class Skeleton {
public:
    Skeleton (double l) : l(l)
    {}
    virtual Point get_point(double) const = 0;
    virtual UnitVector get_tangent(double) const = 0;
    virtual UnitVector get_normal(double) const = 0;
    virtual UnitVector get_binormal(double) const = 0;
    virtual double get_distance(const Point&) const = 0;
    Point get_start_point() const
    {
        return get_point(0.0);
    }
    Point get_end_point() const
    {
        return get_point(l);
    }
    Frame get_frame(double t) const {
        Frame f;
        f << get_tangent(t), get_normal(t), get_binormal(t);
        return f;
    }

    const double l;
};

class Segment : public Skeleton
{
public:
    Segment(const Point&, const UnitVector&, double, const UnitVector&);
    Point get_point(double) const;
    UnitVector get_tangent(double) const;
    UnitVector get_normal(double) const;
    UnitVector get_binormal(double) const;
    double get_distance(const Point&) const;


    const Point p;
    const UnitVector v;
    const UnitVector n;
    const UnitVector b;
};

class Arc : public Skeleton
{
public:
    Arc(const Point&, const UnitVector&, const UnitVector&, double, double);
    Point get_point(double) const;
    UnitVector get_tangent(double) const;
    UnitVector get_normal(double) const;
    UnitVector get_binormal(double) const;
    double get_distance(const Point&) const;
    // std::vector<Point> tangential_polyline() const;

    const Point c;
    const UnitVector u;
    const UnitVector v;
    const UnitVector b;
    const double r;
    const double phi;
};

#endif
