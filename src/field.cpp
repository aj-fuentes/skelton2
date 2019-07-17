#include "field.h"

#include <cmath>
#include <gsl/gsl_integration.h>
#include <gsl/gsl_roots.h>
#include <gsl/gsl_errno.h>

extern "C"
{
    //parameters for function evaluation via GSL
    struct params
    {
        const Point x;
        const Field* field;
        int idx; //only used for gradient numerical integration
        const double lv; //only used for ray shooting
        const UnitVector u; //only used for ray shooting

        params(const Point& x, const Field* field) :
            x(x), field(field), idx(-1), lv(0.0), u(Point(0,0,0))
        {}
        params(const Point& x, const Field* field, const double lv, const UnitVector& u) :
            x(x), field(field), idx(-1), lv(lv), u(u)
        {}
    };

    double integrand_function_wrapper(double t,void* extra)
    {
        const Field *field = ((params*) extra)->field;
        const Point& x = ((params*) extra)->x;
        return field->integrand_function(t,x);
    }

    double integrand_derivative_wrapper(double t,void* extra)
    {
        const Field *field = ((params*) extra)->field;
        const Point& x = ((params*) extra)->x;
        int idx = ((params*) extra)->idx;
        return field->integrand_derivative_function(t,x,idx);
    }

    double ray_shooting_wrapper(double t, void* extra)
    {
        const Field *field = ((params*) extra)->field;
        const Point& x = ((params*) extra)->x;
        const double lv = ((params*) extra)->lv;
        const UnitVector& u = ((params*) extra)->u;
        return field->eval(x+t*u)-lv;
    }

    double omega_poly(double x, void *extra)
    {
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
    gsl_root_fsolver_free(s);
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
    struct params extra(x,this);
    F.params = &extra;

    double val;
    double err;

    gsl_integration_workspace * ws = gsl_integration_workspace_alloc(gsl_ws_size);

    //integrate
    int res = gsl_integration_qag (&F, 0.0e0, skel->l, max_err, max_err, gsl_ws_size, GSL_INTEG_GAUSS61, ws, &val, &err);

    gsl_integration_workspace_free(ws);

    return val*2.1875e0; //adjust to get value 1.0 at extremities
}

Vector Field::gradient_eval(const Point& x) const
{
    //setup integration with GSL
    gsl_function F;
    F.function = &integrand_derivative_wrapper;
    struct params extra(x,this);
    F.params = &extra;

    double val;
    double err;

    Vector grad;

    gsl_integration_workspace * ws;
    for(int i=0;i<3;i++)
    {
        extra.idx = i;
        ws = gsl_integration_workspace_alloc(gsl_ws_size);
        //integrate
        int res = gsl_integration_qag(&F, 0.0e0, skel->l, max_err, max_err, gsl_ws_size, GSL_INTEG_GAUSS61, ws, &val, &err);
        gsl_integration_workspace_free(ws);
        grad(i) = -6.0e0*val*2.1875e0;
    }

    return grad;
}

Point Field::shoot_ray(const Point& p,const UnitVector& u,double lv) const
{

    double x_lo = 0.0;
    double x_hi = std::max(std::max(b[0],b[1]),std::max(c[0],c[1]))*get_eta_constant(lv);
    auto positive = [&](double t){ return eval(p+t*u)>lv;};

    if(not positive(x_lo))
    {
        throw std::logic_error("Error: base shooting point is not inside the surface");
    }

    int max_iter = 100;
    while(positive(x_hi))
    {
        x_hi += x_hi;
        if(--max_iter==0)
        {
            throw std::logic_error("Error: Could not find a point ouside the surface on the shooting ray");
        }
    }

    //setup root finding with GSL
    gsl_function F;
    F.function = &ray_shooting_wrapper;
    struct params extra(p,this,lv,u);
    F.params = &extra;

    gsl_root_fsolver *s = gsl_root_fsolver_alloc(gsl_root_fsolver_brent);
    gsl_root_fsolver_set(s, &F, x_lo, x_hi);

    //find root with GSL
    int status;
    double root;
    int iter = 0;
    max_iter = 100;
    do
    {
        iter++;
        status = gsl_root_fsolver_iterate(s);
        root = gsl_root_fsolver_root(s);
        x_lo = gsl_root_fsolver_x_lower(s);
        x_hi = gsl_root_fsolver_x_upper(s);
        status = gsl_root_test_interval(x_lo, x_hi, 1.0e-8, 0.0);
    }
    while(status == GSL_CONTINUE && iter < max_iter);
    gsl_root_fsolver_free(s);

    if(status != GSL_SUCCESS)
    {
        throw std::logic_error("Error: could not find intersection point with the shooting ray");
    }

    return p+root*u;
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

double SegmentField::integrand_derivative_function(double t, const Point& x, int idx) const
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
    double db = l / (this->b[0] * lt + this->b[1] * t);
    double dc = l / (this->c[0] * lt + this->c[1] * t);

    double a = (XPT - t) * da;
    double b = (XPN    ) * db;
    double c = (XPB    ) * dc;

    double d = 1.0e0 - (a * a + b * b + c * c);
    if (d < 0.0e0) return 0.0e0;

    double Ni =  seg->n[idx] * _cos + seg->b[idx] * _sin;
    double Bi = -seg->n[idx] * _sin + seg->b[idx] * _cos;

    double val = a * da * (seg->v[idx]) + b * db * (Ni) + c * dc * (Bi);

    return d * d * val * da;
}

double ArcField::integrand_function(double t, const Point& x) const
{
    double r = arc->r;
    double l = arc->l;
    double lt = l - t;

    double st = sin(t/r);
    double ct = cos(t/r);

    const auto XC = x-arc->c;

    double XCu = XC.dot(arc->u);
    double XCv = XC.dot(arc->v);
    double XCuv = XC.dot(arc->b);

    double th = (this->th[0] * lt + this->th[1] * t)/l;
    double _cos = cos(th);
    double _sin = sin(th);

    double XCT  = -XCu*st + XCv*ct; //(X-C).T no need to rotate this after
    double XCN0 = -XCu*ct - XCv*st; //(X-C).N
    double XCB0 = XCuv;             //(X-C).B

    //rotate by angle th both N and B
    double XCN =  XCN0 * _cos + XCB0 * _sin;
    double XCB = -XCN0 * _sin + XCB0 * _cos;

    double da = l / (a[0] * lt + a[1] * t);
    double a = da * ( XCT         );
    double b =  l * ( XCN + r*_cos) / (this->b[0] * lt + this->b[1] * t);
    double c =  l * ( XCB - r*_sin) / (this->c[0] * lt + this->c[1] * t);
    double d = 1.0e0 - (a * a + b * b + c * c);
    if (d < 0.0e0) return 0.0e0;
    else return d * d * d * da;
}

double ArcField::integrand_derivative_function(double t, const Point& x, int idx) const
{
    double r = arc->r;
    double l = arc->l;
    double lt = l - t;

    double st = sin(t/r);
    double ct = cos(t/r);

    const auto XC = x-arc->c;

    double XCu = XC.dot(arc->u);
    double XCv = XC.dot(arc->v);
    double XCuv = XC.dot(arc->b);

    double th = (this->th[0]*lt + this->th[1]*t)/l;
    double _cos = cos(th);
    double _sin = sin(th);

    double XCT  = -XCu*st + XCv*ct; //(X-C).T no need to rotate this after
    double XCN0 = -XCu*ct - XCv*st; //(X-C).N
    double XCB0 = XCuv;             //(X-C).B

    //rotate by angle th both N and B
    double XCN =  XCN0 * _cos + XCB0 * _sin;
    double XCB = -XCN0 * _sin + XCB0 * _cos;

    double XPN = XCN + r*_cos;
    double XPB = XCB - r*_sin;

    double da = l / (this->a[0] * lt + this->a[1] * t);
    double db = l / (this->b[0] * lt + this->b[1] * t);
    double dc = l / (this->c[0] * lt + this->c[1] * t);

    double a = XCT * da;
    double b = XPN * db;
    double c = XPB * dc;

    double d = 1.0e0 - (a * a + b * b + c * c);
    if (d < 0.0e0) return 0.0e0;

    double Ti     = - arc->u[idx]*st + arc->v[idx]*ct;
    double Nderiv = - arc->u[idx]*ct - arc->v[idx]*st;

    //rotate the frame by th
    double Ni =  Nderiv * _cos + arc->b[idx] * _sin;;
    double Bi = -Nderiv * _sin + arc->b[idx] * _cos;

    double val = a * da * (Ti) + b * db * (Ni) + c * dc * (Bi);

    return d * d * val * da;
}
