#ifndef BASE_H
#define BASE_H

#include <memory>
#include <vector>
#include <exception>
#include <sstream>
#include <string>

#include <eigen3/Eigen/Dense>

//Constants
#define TOL 1e-10
#define PI_ 3.14159265358979323846

//Predefinitions
// class Skeleton;
// class Field;

//Linear Algebra
typedef Eigen::Vector3d Point;
typedef Eigen::Vector3d Vector;
typedef Eigen::Vector3d UnitVector;
typedef Eigen::Matrix3d Matrix;
typedef Eigen::Matrix3d Frame;

struct ComparePoints
{
    bool operator() (const Point& a, const Point& b) const
    {
        if(abs(a(0)-b(0))<TOL) //first coords are equal
        {
            if(abs(a(1)-b(1))<TOL) //second coords are equal
            {
                if(a(2)<=b(2)-TOL) //last coord is less
                    return true;
            } else if (a(1)<=b(1)-TOL) //second coord is less
                return true;
        } else if (a(0)<=b(0)-TOL) //first coord is less
            return true;

        return false;
    }
};

//Pointers
// typedef std::shared_ptr<Skeleton> Skeleton_ptr;
// typedef std::shared_ptr<Field> Field_ptr;

//Arrays


//Utilities
inline bool is_perp(const Vector& a, const Vector& b)
{
    return abs(a.dot(b))<TOL;
}

inline bool is_unit(const Vector& a)
{
    return abs(a.norm()-1.0)<TOL;
}

inline Point project_to_plane(const Point& q, const UnitVector& n, const Point& p)
{
    return q-(q-p).dot(n)*n;
}

#endif
