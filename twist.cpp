#include <iostream>
#include "common.h"
#include "twist.h"

namespace xar {

Twist::Twist() : Eigen::VectorXd(6)
{
    (*this)(0) = 0.0;
    (*this)(1) = 0.0;
    (*this)(2) = 0.0;
    (*this)(3) = 0.0;
    (*this)(4) = 0.0;
    (*this)(5) = 0.0;
}

Twist::Twist(const Eigen::VectorXd & vector)
{
    (*this)(0) = vector(0);
    (*this)(1) = vector(1);
    (*this)(2) = vector(2);
    (*this)(3) = vector(3);
    (*this)(4) = vector(4);
    (*this)(5) = vector(5);
}

Twist::Twist(const Eigen::Vector3d & omega, const Eigen::Vector3d & point) : Eigen::VectorXd(6)
{
    if (!omega.isZero())		// Rotation axis is not a zero vector. Revolute joint.
    {
        Eigen::Vector3d omg = unitize(omega);
        Eigen::Vector3d vel = point.cross(omg);
        (*this)(0) = vel(0);
        (*this)(1) = vel(1);
        (*this)(2) = vel(2);
        (*this)(3) = omg(0);
        (*this)(4) = omg(1);
        (*this)(5) = omg(2);
    }
    else		// Rotation axis is a zero vector. Prismatic joint.
    {
        Eigen::Vector3d slide_direction = unitize(point);
        (*this)(0) = slide_direction[0];
        (*this)(1) = slide_direction[1];
        (*this)(2) = slide_direction[2];
        (*this)(3) = 0.0;
        (*this)(4) = 0.0;
        (*this)(5) = 0.0;
    }
}

Twist & Twist::operator=(const Eigen::VectorXd & vector)
{
    (*this)(0) = vector(0);
    (*this)(1) = vector(1);
    (*this)(2) = vector(2);
    (*this)(3) = vector(3);
    (*this)(4) = vector(4);
    (*this)(5) = vector(5);

    return (*this);
}

Eigen::Vector3d Twist::vel() const
{
    return Eigen::Vector3d((*this)(0), (*this)(1), (*this)(2));
}

Eigen::Vector3d Twist::omg() const
{
    return Eigen::Vector3d((*this)(3), (*this)(4), (*this)(5));
}

Eigen::Matrix4d Twist::wedge()
{
    Eigen::Matrix4d matrix;
    matrix(0,1) = -(*this)(5);
    matrix(0,2) = (*this)(4);
    matrix(0,3) = (*this)(0);
    matrix(1,2) = -(*this)(3);
    matrix(1,0) = (*this)(5);
    matrix(1,3) = (*this)(1);
    matrix(2,0) = -(*this)(4);
    matrix(2,1) = (*this)(3);
    matrix(2,3) = (*this)(2);

    return matrix;
}

Twist operator*(double scalar, const Twist & twist)
{
    Twist result = twist;
    result(0) *= scalar;
    result(1) *= scalar;
    result(2) *= scalar;
    result(3) *= scalar;
    result(4) *= scalar;
    result(5) *= scalar;

    return result;
}

Twist operator*(const Twist & twist, double scalar)
{
    return (scalar*twist);
}

Twist operator/(const Twist & twist, double scalar)
{
    Twist result = twist;
    result(0) /= scalar;
    result(1) /= scalar;
    result(2) /= scalar;
    result(3) /= scalar;
    result(4) /= scalar;
    result(5) /= scalar;

    return result;
}

Twist operator+(const Eigen::MatrixXd & matrix, const Twist & twist)
{
    Twist result = twist;
    result(0) = matrix(0,0) + twist(0);
    result(1) = matrix(1,0) + twist(1);
    result(2) = matrix(2,0) + twist(2);
    result(3) = matrix(3,0) + twist(3);
    result(4) = matrix(4,0) + twist(4);
    result(5) = matrix(5,0) + twist(5);

    return result;
}

Twist operator-(const Eigen::MatrixXd & matrix, const Twist & twist)
{
    Twist result = twist;
    result(0) = matrix(0,0) - twist(0);
    result(1) = matrix(1,0) - twist(1);
    result(2) = matrix(2,0) - twist(2);
    result(3) = matrix(3,0) - twist(3);
    result(4) = matrix(4,0) - twist(4);
    result(5) = matrix(5,0) - twist(5);

    return result;
}

Twist operator*(const Eigen::MatrixXd & matrix, const Twist & twist)
{
    return Twist(matrix*Eigen::MatrixXd(twist));
}

Twist operator+(const Twist & twist, const Eigen::MatrixXd & matrix)
{
    return (matrix+twist);
}

Twist operator-(const Twist & twist, const Eigen::MatrixXd & matrix)
{
    return Eigen::MatrixXd(-matrix)+twist;
}

Eigen::Matrix3d wedge(const Eigen::Vector3d & omega)
{
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
    matrix(0,1) = -omega[2];
    matrix(0,2) = omega[1];
    matrix(1,2) = -omega[0];
    matrix(1,0) = omega[2];
    matrix(2,0) = -omega[1];
    matrix(2,1) = omega[0];

    return matrix;
}

Eigen::Matrix4d wedge(const Twist & twist)
{
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Zero();
    matrix.block(0,0,3,3) = wedge(twist.omg());
    matrix.block(0,3,3,1) = wedge(twist.vel());

    return matrix;
}

Eigen::Vector3d vee(const Eigen::Matrix3d & matrix)
{
    return Eigen::Vector3d(-matrix(1,2), matrix(0,2), -matrix(0,1));
}

Twist vee(const Eigen::Matrix4d & matrix)
{
    Twist twist;
    twist(0) = matrix(0,3);
    twist(1) = matrix(1,3);
    twist(2) = matrix(2,3);
    twist(3) = -matrix(1,2);
    twist(4) = matrix(0,2);
    twist(5) = -matrix(0,1);

    return twist;
}

Eigen::Matrix3d expMap(const Eigen::Vector3d & omega)
{
    return expMap(Eigen::Vector3d(unitize(omega)), omega.norm());
}

Eigen::Matrix3d expMap(const Eigen::Vector3d & omega, double theta)
{
    Eigen::Vector3d axis = unitize(omega);
    return (Eigen::Matrix3d::Identity() + std::sin(theta)*wedge(axis) + (1-std::cos(theta))*(wedge(axis)*wedge(axis)));
}

Eigen::Matrix4d expMap(const Twist & twist)
{
    if (twist.omg().isZero())		// Prismatic joint.
        return expMap(twist, twist.vel().norm());
    else
        return expMap(twist, twist.omg().norm());
}

Eigen::Matrix4d expMap(const Twist & twist, double theta)
{
    Eigen::Matrix4d mat_transf = Eigen::Matrix4d::Identity();
    if (twist.omg().isZero())		// Prismatic joint.
    {
        mat_transf.block(0,3,3,1) = unitize(twist.vel()) * theta;
    }
    else
    {
        // Unitize the twist.
        Twist twist_unit = twist / twist.omg().norm();
        Eigen::Matrix3d R = expMap(twist_unit.omg(), theta);
        Eigen::Vector3d p = (Eigen::Matrix3d::Identity()-R)*twist_unit.omg().cross(twist_unit.vel()) +
            twist_unit.omg()*twist_unit.omg().transpose()*twist_unit.vel()*theta;

        mat_transf.block(0,0,3,3) = R;
        mat_transf.block(0,3,3,1) = p;
    }

    return mat_transf;
}

Eigen::Matrix4d invTransf(const Eigen::Matrix4d & matrix)
{
    Eigen::Matrix4d inverse_matrix = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R = matrix.block(0, 0, 3, 3);
    Eigen::Vector3d p = matrix.block(0, 3, 3, 1);
    inverse_matrix.block(0, 0, 3, 3) =  R.transpose();
    inverse_matrix.block(0, 3, 3, 1) = -R.transpose()*p;

    return inverse_matrix;
}


Twist lieBracket(const Twist & twist_1, const Twist & twist_2)
{
    return vee(Eigen::Matrix4d(wedge(twist_1)*wedge(twist_2) - wedge(twist_2)*wedge(twist_1)));
}

Twist lieBrace(const Twist & twist_1, const Twist & twist_2)
{
    Twist result;
    result.block(0, 0, 3, 1) = twist_1.omg().cross(twist_2.vel());
    result.block(3, 0, 3, 1) = twist_1.omg().cross(twist_2.omg()) + twist_1.vel().cross(twist_2.vel());

    return result;
}

Eigen::MatrixXd Ad(const Eigen::Matrix4d & matrix)
{
    Eigen::MatrixXd adjoint_matrix = Eigen::MatrixXd::Identity(6,6);
    Eigen::Matrix3d R = matrix.block(0,0,3,3);
    Eigen::Vector3d p = matrix.block(0,3,3,1);

    adjoint_matrix.block(0,0,3,3) = R;
    adjoint_matrix.block(3,3,3,3) = R;
    adjoint_matrix.block(0,3,3,3) = wedge(p)*R;

    return adjoint_matrix;
}

Eigen::MatrixXd invAd(const Eigen::MatrixXd & matrix)
{
    Eigen::Matrix3d R = matrix.block(0,0,3,3);
    Eigen::Matrix3d pR = matrix.block(0,3,3,3);
    Eigen::MatrixXd inv_ad = Eigen::MatrixXd::Zero(6,6);
    inv_ad.block(0,0,3,3) = R.transpose();
    inv_ad.block(0,3,3,3) = -R.transpose()*pR*R.transpose();
    inv_ad.block(3,3,3,3) = R.transpose();

    return inv_ad;
}

Eigen::Matrix4d prodOfExp(const std::vector<Twist> & twists, const Eigen::VectorXd & joint_angle,
        const Eigen::Matrix4d & pose_end_effector_home)
{
    if (twists.size() == joint_angle.rows())
    {
        Eigen::Matrix4d pose_end_effector = pose_end_effector_home;
        for (int i = twists.size()-1; i >= 0; --i)
            pose_end_effector = expMap(twists[i], joint_angle[i]) * pose_end_effector;

        return pose_end_effector;
    }
    else
    {
        std::cerr << "Dimensions conflict." << std::endl;
        exit(1);
    }
}

Eigen::Vector3d prodOfExp(const std::vector<Twist> & twists, const Eigen::VectorXd & joint_angle,
        const Eigen::Vector3d & pos_end_effector_home)
{
    Eigen::Matrix4d pose_home = Eigen::Matrix4d::Identity();
    pose_home.block(0,3,3,1) = pos_end_effector_home;
    Eigen::Matrix4d pose = prodOfExp(twists, joint_angle, pose_home);

    return Eigen::Vector3d(pose_home.block(0,3,3,1));
}

}
