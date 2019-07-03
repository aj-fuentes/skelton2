#include "field.h"

#include <cmath>

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

double Field::get_omega_constant(double lv) {
    // # = 0.5493568319351
    // p = lambda x: x-(x**3)+3.0/5.0*(x**5)-(x**7)/7.0 - (1-level_set)*16.0/35.0
    // _pr_brent2 = pr.Brentq(epsilon=1e-10)
    // _brent2 = lambda g,a,b: _pr_brent(g,a,b).x0
    // return _brent2(p,0,1)
    return 1.0;
}

double Field::get_eta_constant(double lv) {
    return sqrt(1.0-pow(lv*0.5,2.0/7.0));
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

    return val/2.1875e0; //adjust to get value 1.0 at extremities
}

double SegmentField::integrand_function(double t, const Point& x) const
{

    double l = seg->l;
    double lt = l - t;

    double th = (this->th[0] * lt + this->th[1] * t)/l;
    double _cos = cos(th);
    double _sin = sin(th);

    double XPN_ = (x-seg->p).dot(seg->n);
    double XPB_ = (x-seg->p).dot(seg->b);

    double XPT =  (x-seg->p).dot(seg->v);;
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
