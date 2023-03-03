#define _USE_MATH_DEFINES
#include <cmath>
#include "common.h"

namespace xar {

const double PI = M_PI;
const double TOLERANCE = 1E-12;

double deg2rad(double degree)
{
    return degree*PI/180.0;
}

double rad2deg(double rad)
{
    return rad*180.0/PI;
}

double sqrt(double x, double tolerance)
{
    if ((x >= -tolerance) && (x < 0))
        x = 0.0;

    return std::sqrt(x);
}

double asin(double x, double tolerance)
{
    if ((x > 1.0) && (x <= (1.0+tolerance)))
        x = 1.0;
    else if ((x < -1.0) && (x >= (-1.0-tolerance)))
        x = -1.0;

    return std::asin(x);
}

double acos(double x, double tolerance)
{
    if ((x > 1.0) && (x <= (1.0+tolerance)))
        x = 1.0;
    else if ((x < -1.0) && (x >= (-1.0-tolerance)))
        x = -1.0;

    return std::acos(x);
}

Eigen::VectorXd unitize(const Eigen::VectorXd vector)
{
    Eigen::VectorXd unit_vector = Eigen::VectorXd(vector.rows());
    if (!vector.isZero())
    {
        double length = vector.norm();
        for (int i = 0; i < vector.rows(); ++i)
            unit_vector(i) = vector(i) / length;
    }

    return unit_vector;
}

Eigen::Vector4d vectHomo(const Eigen::Vector3d & free_vector)
{
    Eigen::Vector4d homo_free_vector;
    homo_free_vector(0) = free_vector(0);
    homo_free_vector(1) = free_vector(1);
    homo_free_vector(2) = free_vector(2);
    homo_free_vector(3) = 0.0;

    return homo_free_vector;
}

Eigen::Vector4d posHomo(const Eigen::Vector3d & pos_vector)
{
    Eigen::Vector4d homo_pos_vector;
    homo_pos_vector(0) = pos_vector(0);
    homo_pos_vector(1) = pos_vector(1);
    homo_pos_vector(2) = pos_vector(2);
    homo_pos_vector(3) = 1.0;

    return homo_pos_vector;
}

Eigen::Vector3d dehomo(const Eigen::Vector4d & homo_vector)
{
    Eigen::Vector3d vector;
    vector(0) = homo_vector(0);
    vector(1) = homo_vector(1);
    vector(2) = homo_vector(2);

    return vector;
}

}
