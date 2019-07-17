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
    Skeleton(phi*r), c(c), v(v), b(u.cross(v).normalized()), r(r), phi(phi)
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

