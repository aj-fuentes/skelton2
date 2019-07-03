#ifndef SKELETON_H
#define SKELETON_H

#include "base.h"

class Skeleton;
class Segment;
typedef std::shared_ptr<Skeleton> Skeleton_ptr;
typedef std::shared_ptr<Segment> Segment_ptr;

class Skeleton {
    friend class Field;
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
    virtual ~Skeleton()
    {}
protected:
    double l;
};

class Segment : public Skeleton
{
    friend class SegmentField;
public:
    Segment(const Point&, const UnitVector&, double, const UnitVector&);
    Point get_point(double) const;
    UnitVector get_tangent(double) const;
    UnitVector get_normal(double) const;
    UnitVector get_binormal(double) const;
    double get_distance(const Point&) const;
    ~Segment()
    {}
private:
    Point p;
    UnitVector v,n,b;
};

#endif
