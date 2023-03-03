#include <cmath>
#include "orientation.h"

namespace xar {

RpyEuler::RpyEuler()
{
    (*this)(0) = 0.0;
    (*this)(1) = 0.0;
    (*this)(2) = 0.0;
}

RpyEuler::RpyEuler(const Eigen::Vector3d & vector)
{
    (*this)(0) = vector(0);
    (*this)(1) = vector(1);
    (*this)(2) = vector(2);
}

RpyEuler::RpyEuler(double roll, double pitch, double yaw)
{
    (*this)(0) = roll;
    (*this)(1) = pitch;
    (*this)(2) = yaw;
}

RpyEuler & RpyEuler::operator=(const Eigen::Vector3d & vector)
{
    (*this)(0) = vector(0);
    (*this)(1) = vector(1);
    (*this)(2) = vector(2);

    return (*this);
}

Eigen::Matrix3d RpyEuler::toMatrix()
{
    double roll = (*this)(0);
    double pitch = (*this)(1);
    double yaw = (*this)(2);

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix(0,0) = std::cos(roll)* std::cos(pitch);
    rotation_matrix(0,1) = std::cos(roll)*std::sin(pitch)*std::sin(yaw) - std::sin(roll)*std::cos(yaw);
    rotation_matrix(0,2) = std::cos(roll)*std::sin(pitch)*std::cos(yaw) + std::sin(roll)*std::sin(yaw);

    rotation_matrix(1,0) = std::sin(roll)*std::cos(pitch);
    rotation_matrix(1,1) = std::sin(roll)*std::sin(pitch)*std::sin(yaw) + std::cos(roll)*std::cos(yaw);
    rotation_matrix(1,2) = std::sin(roll)*std::sin(pitch)*std::cos(yaw) - std::cos(roll)*std::sin(yaw);

    rotation_matrix(2,0) = -std::sin(pitch);
    rotation_matrix(2,1) = std::cos(pitch)*std::sin(yaw);
    rotation_matrix(2,2) = std::cos(pitch)*std::cos(yaw);

    return rotation_matrix;
}

Quaternion RpyEuler::toQuat()
{
    return Quaternion(this->toMatrix());
}

ZyzEuler::ZyzEuler()
{
    (*this)(0) = 0.0;
    (*this)(1) = 0.0;
    (*this)(2) = 0.0;
}

ZyzEuler::ZyzEuler(const Eigen::Vector3d & vector)
{
    (*this)(0) = vector(0);
    (*this)(1) = vector(1);
    (*this)(2) = vector(2);
}

ZyzEuler::ZyzEuler(double phi, double theta, double psi)
{
    (*this)(0) = phi;
    (*this)(1) = theta;
    (*this)(2) = psi;
}

ZyzEuler & ZyzEuler::operator=(const Eigen::Vector3d & vector)
{
    (*this)(0) = vector(0);
    (*this)(1) = vector(1);
    (*this)(2) = vector(2);

    return (*this);
}

Eigen::Matrix3d ZyzEuler::toMatrix()
{
    Eigen::Matrix3d rotation_matrix;

    double phi = (*this)(0);
    double theta = (*this)(1);
    double psi = (*this)(2);

    rotation_matrix(0,0) = std::cos(phi)*std::cos(theta)*std::cos(psi) - std::sin(phi)*std::sin(psi);
    rotation_matrix(0,1) = -std::cos(phi)*std::cos(theta)*std::sin(psi) - std::sin(phi)*std::cos(psi);
    rotation_matrix(0,2) = std::cos(phi)*std::sin(theta);

    rotation_matrix(1,0) = std::sin(phi)*std::cos(theta)*std::cos(psi) + std::cos(phi)*std::sin(psi);
    rotation_matrix(1,1) = -std::sin(phi)*std::cos(theta)*std::sin(psi) + std::cos(phi)*std::cos(psi);
    rotation_matrix(1,2) = std::sin(phi)*std::sin(theta);

    rotation_matrix(2,0) = -std::sin(theta)*std::cos(psi);
    rotation_matrix(2,1) = std::sin(theta)*std::sin(psi);
    rotation_matrix(2,2) = std::cos(theta);

    return rotation_matrix;
}

Quaternion ZyzEuler::toQuat()
{
    return Quaternion(this->toMatrix());
}

RpyEuler matrix2rpyEuler(const Eigen::Matrix3d & matrix)
{
    return RpyEuler(matrix.eulerAngles(2,1,0));
}

ZyzEuler matrix2zyzEuler(const Eigen::Matrix3d & matrix)
{
    return ZyzEuler(matrix.eulerAngles(2,1,2));
}

}
