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

//Predefinitions
class Skeleton;
class Field;

//Linear Algebra
typedef Eigen::Vector3d Point;
typedef Eigen::Vector3d Vector;
typedef Eigen::Vector3d UnitVector;
typedef Eigen::Vector2d Param2d;
typedef Eigen::Matrix3d Matrix;
typedef Eigen::Matrix3d Frame;

//Pointers
typedef std::shared_ptr<Skeleton> Skeleton_ptr;
typedef std::shared_ptr<Field> Field_ptr;

//Arrays


//Utilities
inline bool is_perp(const Vector& a, const Vector& b) {
    return abs(a.dot(b))<TOL;
}

inline bool is_unit(const Vector& a) {
    return abs(a.norm()-1.0)<TOL;
}

#endif
