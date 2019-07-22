#ifndef FIELD_H
#define FIELD_H

#include "base.h"
#include "skeleton.h"

class Field;
class SegmentField;
class ArcField;
class MultiField;

typedef std::shared_ptr<Field> Field_ptr;
typedef std::shared_ptr<SegmentField> SegmentField_ptr;
typedef std::shared_ptr<ArcField> ArcField_ptr;
typedef std::shared_ptr<MultiField> MultiField_ptr;


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
protected:

    virtual double max_radius(double lv) const
    {
        const double max_bc = std::max(std::max(b[0],b[1]),std::max(c[0],c[1]))*get_eta_constant(lv);
        // const double max_a = std::max(a[0],a[1])*get_omega_constant(lv);
        // return std::max(max_bc,max_a);
        return max_bc;
    }

    //base constructor for MultiField
    Field() : max_err(1.0e-8), gsl_ws_size(100)
    {}
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

class MultiField : public Field
{
public:
    MultiField(const std::vector<Field_ptr>& fields) :
        Field(), fields(fields)
    {}
    double integrand_function(double, const Point&) const;
    double integrand_derivative_function(double,const Point&,int) const;
    double max_radius(double) const;

private:
    const std::vector<Field_ptr> fields;
};

#endif
