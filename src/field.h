/** @file field.h Anisotropic convolution fields
 *
 * @author     Alvaro Fuentes
 */

#ifndef FIELD_H
#define FIELD_H

#include "base.h"
#include "skeleton.h"

class Field;
class SegmentField;
class ArcField;
class MultiField;

/**
 * Shared pointer to Field object
 */
typedef std::shared_ptr<Field> Field_ptr;

/**
 * Shared pointer to SegmentField object
 */
typedef std::shared_ptr<SegmentField> SegmentField_ptr;

/**
 * Shared pointer to ArcField object
 */
typedef std::shared_ptr<ArcField> ArcField_ptr;

/**
 * Shared pointer to MultiField object
 */
typedef std::shared_ptr<MultiField> MultiField_ptr;

/**
 * Parameter for Field constructor.
 *
 * Each parameter is a pair of values.
 */
typedef std::vector<double> FieldParams;

/**
 * @brief      This abstract class implements the basic functionality of an
 *             anisotropic convolution field.
 */
class Field
{
public:
    /**
     * @brief      Base constructor for anisotropic convolution fields.
     *
     *             An anisotropic convolution field needs 8 parameters. At each
     *             extremity of the supporting skeleton it needs 3 radii and and
     *             angle that defines the rotation with respect to the frame of
     *             the skeleton. In the parameters of the constructor (see
     *             FieldParams) the first item is associated to the first
     *             extremity of the skeleton, while the second item corresponds
     *             to the last extremity.
     *
     *             The radius parameters are related to the ACTUAL radius of the surface by the following equations:
     *             - actual_radius = radius_param/
     *
     * @param[in]  skel  The skeleton for the field
     * @param[in]  a     Tangential radii information.
     * @param[in]  b     Normal radii information.
     * @param[in]  c     Binormal radii information.
     * @param[in]  th    Rotation information.
     */
    Field(const Skeleton_ptr& skel, const FieldParams& a, const FieldParams& b, const FieldParams& c, const FieldParams& th) :
        skel(skel), a(a), b(b), c(c), th(th), max_err(1.0e-8), gsl_ws_size(100)
    {}
    virtual double eval(const Point&) const;
    virtual Vector gradient_eval(const Point&) const;
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
        double max_bc = std::max(std::max(b[0],b[1]),std::max(c[0],c[1]))*get_eta_constant(lv);
        double max_a = std::max(a[0],a[1])*get_omega_constant(lv);
        return std::max(max_bc,max_a);
        // return max_bc;
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
    double eval(const Point&) const override;
    Vector gradient_eval(const Point&) const override;
    double integrand_function(double, const Point&) const override
    {
        return 0.0;
    }
    double integrand_derivative_function(double,const Point&,int) const override
    {
        return 0.0;
    }
    double max_radius(double) const;

private:
    const std::vector<Field_ptr> fields;
};

#endif
