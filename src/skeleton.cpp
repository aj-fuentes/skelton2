#include "skeleton.h"

Segment::Segment(const Point& p, const UnitVector& v, double l, const UnitVector& n) :
    Skeleton(l), p(p), v(v), n(n), b(v.cross(n).normalized())
{
    if (abs(v.norm()-1)>TOL) {
        std::stringstream ss;
        ss << "Segment creation error: v=" << v.transpose();
        ss << " is not a unit vector (norm=" << v.norm() << ")";
        throw new std::invalid_argument(ss.str());
    }
    if (abs(n.norm()-1)>TOL) {
        std::stringstream ss;
        ss << "Segment creation error: n=" << n.transpose();
        ss << " is not a unit vector (norm=" << n.norm() << ")";
        throw new std::invalid_argument(ss.str());
    }
    if (v.dot(n)>TOL)
    {
        std::stringstream ss;
        ss << "Segment creation error: v=" << v.transpose();
        ss << " is not perpendicular to n=" << n.transpose();
        throw new std::invalid_argument(ss.str());
    }
}

Point Segment::get_point(double t) const
{
    return p+t*v;
}

UnitVector Segment::get_tangent(double) const
{
    return v;
}

UnitVector Segment::get_normal(double) const
{
    return n;
}

UnitVector Segment::get_binormal(double) const
{
    return b;
}

double Segment::get_distance(const Point& x) const
{
    auto t = (x-p).dot(v);
    if (t<0.0) return (p-x).norm();
    else if (t>l) return (p-x+l*v).norm();
    else return (p-x+t*v).norm();
}

Arc::Arc(const Point& c, const UnitVector& u, const UnitVector& v, double r, double phi) :
    Skeleton(phi*r), c(c), u(u), v(v), b(u.cross(v).normalized()), r(r), phi(phi)
{
    if(abs(u.norm()-1)>TOL)
    {
        std::stringstream ss;
        ss << "Error: invalid Arc params, u=" << u.transpose();
        ss << " is not a unit vector (norm=" << u.norm() << ")";
        throw new std::invalid_argument(ss.str());
    }
    if(abs(v.norm()-1)>TOL)
    {
        std::stringstream ss;
        ss << "Error: invalid Arc params, v=" << v.transpose();
        ss << " is not a unit vector (norm=" << v.norm() << ")";
        throw new std::invalid_argument(ss.str());
    }
    if(v.dot(u)>TOL)
    {
        std::stringstream ss;
        ss << "Error: invalid Arc params, u=" << v.transpose();
        ss << " is not perpendicular to v=" << u.transpose();
        throw new std::invalid_argument(ss.str());
    }
    if(r<TOL)
    {
        std::stringstream ss;
        ss << "Error: invalid Arc params, r=" << r << "must be positive";
        throw new std::invalid_argument(ss.str());
    }
    if(phi<TOL or phi > PI_)
    {
        std::stringstream ss;
        ss << "Error: invalid Arc params, phi=" << r << "must be in the interval [0,pi]";
        throw new std::invalid_argument(ss.str());
    }
}

Point Arc::get_point(double t) const
{
    return c + r*cos(t/r)*u + r*sin(t/r)*v;
}

UnitVector Arc::get_tangent(double t) const
{
    return -sin(t/r)*u + cos(t/r)*v;
}

UnitVector Arc::get_normal(double t) const
{
    return -cos(t/r)*u - sin(t/r)*v;
}

UnitVector Arc::get_binormal(double) const
{
    return b;
}

double Arc::get_distance(const Point& x) const
{
    const double d0 = std::min((x-get_start_point()).norm(),(x-get_end_point()).norm());

    const double th = atan2(v.dot(x-c),u.dot(x-c));

    if (th>0.0 and th<phi)
        return std::min(d0,(x-get_point(th)).norm());
    return d0;
}

std::vector<Point> Arc::tangential_polyline() const
{
    const double d = r/cos(phi/4);
    return {
        get_start_point(),
        c +   d*cos(phi/4)*u +   d*sin(phi/4)*v,
        c + d*cos(3*phi/4)*u + d*sin(3*phi/4)*v,
        get_end_point()
    };
}

std::pair<Arc_ptr,Arc_ptr> Arc::build_biarc(const Point& a0, const UnitVector& u0, const Point& a1, const UnitVector& u1)
{
    const double a = (u0+u1).dot(u0+u1) - 4;//a = np.dot(t0+t1,t0+t1)-4.0
    const double b = 2*(u0+u1).dot(a0-a1);//b = 2.0*np.dot(t0+t1,A0-A1)
    const double c = (a0-a1).dot(a0-a1);//c = np.dot(A0-A1,A0-A1)

    //take positive root of discriminant
    double l = (-b + sqrt(b*b-4.0*a*c))/(2.0*a);
    if(l<0.0)
        l = (-b - sqrt(b*b-4.0*a*c))/(2.0*a);

    const auto L = a0 + l*u0;//L = A0+l*t0
    const auto N = a1 - l*u1;//N = A1-l*t1
    const auto M = (L+N)/2;//M = 0.5*(L+N)

    // a1 = get_arc_from_points(A0,L,M)
    // a2 = get_arc_from_points(M,N,A1)
    return std::pair<Arc_ptr,Arc_ptr>(build_arc_from_points(a0,L,M),build_arc_from_points(M,N,a1));
}

Arc_ptr Arc::build_arc_from_points(const Point& a, const Point& b, const Point& c)
{
    if(abs((a-b).norm()-(c-b).norm())>TOL)
    {
        std::logic_error("Error: cannot construct arc, distances from points to middle are not equal");
    }
    const UnitVector u0 = (a-b).normalized();
    const Vector w = c-b;
    const UnitVector v0 = u0.cross(w).cross(u0).normalized();
    const double th = atan2(w.dot(v0),w.dot(u0));
    const double x = (a-b).norm()/cos(th/2);

    const auto center = ((a+c)/2-b).normalized()*x + b;
    const double phi = PI_-th;
    const double r = (a-center).norm();
    const auto u = -u0;
    const auto v = -v0;

    return Arc_ptr(new Arc(center,u,v,r,phi));
}
