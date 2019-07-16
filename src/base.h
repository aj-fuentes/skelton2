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


class PointIndexer
{
public:
    const std::vector<Point>& get_points() const
    {
        return points;
    }
    int size() const
    {
        return points.size();
    }
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
