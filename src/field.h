#ifndef FIELD_H
#define FIELD_H

#include "base.h"
#include "skeleton.h"

class Field;
class SegmentField;
typedef std::shared_ptr<Field> Field_ptr;
typedef std::shared_ptr<SegmentField> SegmentField_ptr;

typedef std::vector<double> FieldParams;

class Field
{
public:
    Field(const Skeleton_ptr& skel, const FieldParams& a, const FieldParams& b, const FieldParams& c, const FieldParams& th) :
        skel(skel), a(a), b(b), c(c), th(th), max_err(1.0e-8), gsl_ws_size(100)
    {}
    double eval(const Point&) const;
    Vector gradient_eval(const Point&) const;
    virtual Point shoot_ray(const Point&,const UnitVector&,double) const;
    virtual double integrand_function(double,const Point&) const = 0;
    virtual double integrand_derivative_function(double,const Point&,int) const = 0;

    static double get_omega_constant(double);
    static double get_eta_constant(double);
    static double get_tangential_param(double,double);
    static double get_normal_param(double,double);

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
    double integrand_function(double, const Point&) const;
    double integrand_derivative_function(double,const Point&,int) const;

    const Segment_ptr seg;
};

class ArcField : public Field
{
public:
    ArcField(const Arc_ptr& skel, const FieldParams a, const FieldParams b, const FieldParams c, const FieldParams th) : Field(skel,a,b,c,th), arc(skel)
    {}
    double integrand_function(double, const Point&) const;
    double integrand_derivative_function(double,const Point&,int) const;

    const Arc_ptr arc;
};

#endif
