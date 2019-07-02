#include "field.h"

double Field::eval(const Point x) const
{
    return 0.0;
}
Vector Field::gradient_eval(const Point x) const
{
    return Vector::Zero();
}
Point Field::shoot_ray(const Point q,const UnitVector w, const double ls) const
{
    return Point::Zero();
}
