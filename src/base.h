/** @file base.h Base include for all includes in this project.
 */

#ifndef BASE_H
#define BASE_H

#include <vector>
#include <map>
#include <set>
#include <memory>
#include <exception>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <numeric>

#include <eigen3/Eigen/Dense>

/** Tolerance used for most comparisons.
 */
#define TOL 1e-10

/** Value of pi with high precision.
 */
#define PI_ 3.14159265358979323846

/**
 * @brief      Point in 3D.
 *
 * @see        Vector
 * @see        UnitVector
 */
typedef Eigen::Vector3d Point;
/**
 * @brief      Vector in 3D.
 *
 *             Currently it is the same as Point but the implementation may
 *             evolve.
 *
 * @see        Point
 * @see        UnitVector
 */
typedef Eigen::Vector3d Vector;
/**
 * @brief      Unit vector (unit norm) in 3D.
 *
 *             Currently is the same as Vector, but in the future the unit norm
 *             may be enforced.
 *
 * @see        Vector
 * @see        Point
 */
typedef Eigen::Vector3d UnitVector;

/**
 * @brief      3x3 matrix.
 *
 * @see        Frame
 */
typedef Eigen::Matrix3d Matrix;
/**
 * @brief      Orthonormal matrix.
 *
 *             Currently is the same as Matrix, future implementation may
 *             enforce orthonormal property.
 *
 * @see        Matrix
 */
typedef Eigen::Matrix3d Frame;

/**
 * @brief      Stores points without duplications and returns their indexes.
 */
class PointIndexer
{
public:
    /**
     * @brief      Gets the points currently stored.
     *
     * @return     The vector of points.
     */
    const std::vector<Point>& get_points() const
    {
        return points;
    }

    /**
     * @brief      Gives the number of points currently stored.
     *
     * @return     The size of the current vector of points.
     * @see        get_points()
     */
    int size() const
    {
        return points.size();
    }

    /**
     * @brief      Indexes a point within the stored points. It adds the point
     *             to the storage if it is not on it already.
     *
     * @param[in]  p     The point to index.
     *
     * @return     The index (position) within the storage.
     * @see        get_points()
     */
    int index(const Point& p)
    {
        if(point_idxs.find(p)==point_idxs.end())
        {
            points.push_back(p);
            point_idxs[p] = points.size()-1;
        }
        return point_idxs[p];
    }

private:

    /**
     * @brief      Internal implementation of comparison for points.
     *
     *             Two points are compared lexicographically within a tolerance
     *             given by the macro TOL.
     */
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

    std::vector<Point> points;
    std::map<Point,int,ComparePoints> point_idxs;
};

#endif
