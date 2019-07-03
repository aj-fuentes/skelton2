#include "field.h"

#include <cmath>
#include <gsl/gsl_integration.h>
#include <gsl/gsl_roots.h>
#include <gsl/gsl_errno.h>

extern "C"
{
    struct params {
        const Point& x;
        const Field *field;
    };

    double integrand_function_wrapper(double t,void * extra)
    {
        const Field *field = ((params*) extra)->field;
        const Point& x = ((params*) extra)->x;
        return field->integrand_function(t,x);
    }
}

extern "C" {
    double omega_poly(double x, void *extra) {
        double lv = *((double*) extra);
        double x2=x*x;
        double x3=x2*x;
        double x5=x3*x2;
        double x7=x5*x2;
        return x - x3 + (3.0/5.0)*x5 - x7/7.0 - (1.0-lv)*16.0/35.0;
    }
}

double Field::get_omega_constant(double lv) {
    double extra = lv;

    gsl_function F;
    F.function = &omega_poly;
    F.params = &extra;

    gsl_root_fsolver *s = gsl_root_fsolver_alloc(gsl_root_fsolver_brent);
    gsl_root_fsolver_set(s, &F, 0.0, 1.0);

    int status;
    double root,x_lo,x_hi;
    int iter = 0, max_iter = 100;
    do
    {
        iter++;
        status = gsl_root_fsolver_iterate(s);
        root = gsl_root_fsolver_root(s);
        x_lo = gsl_root_fsolver_x_lower(s);
        x_hi = gsl_root_fsolver_x_upper(s);
        status = gsl_root_test_interval(x_lo, x_hi, 1.0e-8, 0.0);
        // if (status == GSL_SUCCESS)
        //     break;
    }
    while (status == GSL_CONTINUE && iter < max_iter);
    gsl_root_fsolver_free (s);
    return root;
}

double Field::get_eta_constant(double lv) {
    return sqrt(1.0-pow(lv*0.5,2.0/7.0));
}

double Field::get_tangential_param(double r, double lv) {
    return r/get_omega_constant(lv);
}

double Field::get_normal_param(double r, double lv) {
    return r/get_eta_constant(lv);
}

double Field::eval(const Point& x) const
{
    //setup integration with GSL
    gsl_function F;
    F.function = &integrand_function_wrapper;
    struct params extra = {x,this};
    F.params = &extra;

    gsl_integration_workspace * ws = gsl_integration_workspace_alloc(gsl_ws_size);

    double val;
    double err;

    //integrate
    int res = gsl_integration_qag (&F, 0.0e0, skel->l, max_err, max_err, gsl_ws_size, GSL_INTEG_GAUSS61, ws, &val, &err);

    gsl_integration_workspace_free(ws);

    return val*2.1875e0; //adjust to get value 1.0 at extremities
}

double SegmentField::integrand_function(double t, const Point& x) const
{

    double l = seg->l;
    double lt = l - t;

    double th = (this->th[0] * lt + this->th[1] * t)/l;
    double _cos = cos(th);
    double _sin = sin(th);

    Point XP = (x-seg->p);

    double XPN_ = XP.dot(seg->n);
    double XPB_ = XP.dot(seg->b);

    double XPT =  XP.dot(seg->v);
    double XPN =  XPN_ * _cos + XPB_ * _sin;
    double XPB = -XPN_ * _sin + XPB_ * _cos;

    double da = l / (this->a[0] * lt + this->a[1] * t);
    double a = da * (XPT - t);
    double b = l * (XPN) / (this->b[0] * lt + this->b[1] * t);
    double c = l * (XPB) / (this->c[0] * lt + this->c[1] * t);
    double d = 1.0e0 - (a * a + b * b + c * c);

    if (d < 0.0e0) return 0.0e0;
    else return d * d * d * da;
}

Vector SegmentField::gradient_eval(const Point& x) const
{
    return Vector::Zero();
}

Point SegmentField::shoot_ray(const Point& q, const UnitVector& w, const double lv) const
{
    return Point::Zero();
}
