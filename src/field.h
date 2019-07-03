#ifndef FIELD_H
#define FIELD_H

#include "base.h"
#include "skeleton.h"

#include <gsl/gsl_integration.h>

class Field;
class SegmentField;
typedef std::shared_ptr<Field> Field_ptr;
typedef std::shared_ptr<SegmentField> SegmentField_ptr;

typedef std::vector<double> FieldParams;

extern "C"
{
    double integrand_function_wrapper(double,void*);
}

class Field
{
public:
    Field(const Skeleton_ptr& skel, const FieldParams& a, const FieldParams& b, const FieldParams& c, const FieldParams& th) :
        skel(skel), a(a), b(b), c(c), th(th), max_err(1.0e-8), gsl_ws_size(100)
    {}
    double eval(const Point&) const;
    virtual Vector gradient_eval(const Point&) const = 0;
    virtual Point shoot_ray(const Point&,const UnitVector&,double) const = 0;
    virtual double integrand_function(double,const Point&) const = 0;
    virtual ~Field()
    {}

    static double get_omega_constant(double);
    static double get_eta_constant(double);

    const Skeleton_ptr skel;
    const FieldParams a;
    const FieldParams b;
    const FieldParams c;
    const FieldParams th;
    double max_err;
    int gsl_ws_size;
};

class SegmentField : public Field
{
public:
    SegmentField(const Segment_ptr& skel, const FieldParams a, const FieldParams b, const FieldParams c, const FieldParams th) : Field(skel,a,b,c,th), seg(skel)
    {}
    Vector gradient_eval(const Point&) const;
    Point shoot_ray(const Point&,const UnitVector&,double) const;
    double integrand_function(double, const Point&) const;
    ~SegmentField()
    {}

    const Segment_ptr seg;
};

#endif
