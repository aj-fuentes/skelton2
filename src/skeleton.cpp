#include "skeleton.h"

Segment::Segment(const Point& p, const UnitVector& v, double l, const UnitVector& n) :
    Skeleton(l), p(p), v(v), n(n), b(v.cross(n))
{
    if (abs(v.norm()-1)>TOL) {
        std::ostringstream msg;
        msg << "Segment creation error: v=" << v.transpose();
        msg << " is not a unit vector (norm=" << v.norm() << ")";
        throw new std::logic_error(msg.str());
    }
    if (abs(n.norm()-1)>TOL) {
        std::ostringstream msg;
        msg << "Segment creation error: n=" << n.transpose();
        msg << " is not a unit vector (norm=" << n.norm() << ")";
        throw new std::logic_error(msg.str());
    }
    if (v.dot(n)>TOL)
    {
        std::ostringstream msg;
        msg << "Segment creation error: v=" << v.transpose();
        msg << " is not perpendicular to n=" << n.transpose();
        throw new std::logic_error(msg.str());
    }
}

Point Segment::get_point(double t) const {
    return p+t*v;
}
UnitVector Segment::get_tangent(double) const {
    return v;
}
UnitVector Segment::get_normal(double) const{
    return n;
}
UnitVector Segment::get_binormal(double) const{
    return b;
}
double Segment::get_distance(const Point& x) const {
    auto t = (x-p).dot(v);
    if (t<0.0) return (p-x).norm();
    else if (t>l) return (p-x+l*v).norm();
    else return (p-x+t*v).norm();
}

