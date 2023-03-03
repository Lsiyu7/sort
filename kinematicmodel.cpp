#include <iostream>
#include "kinematicmodel.h"

namespace xar {

KinematicModel::KinematicModel()
{ }

KinematicModel::~KinematicModel()
{ }

void KinematicModel::setTwists(const std::vector<Twist> & twists)
{
    twists_ = twists;
}

void KinematicModel::setEndEffectorHome(const Eigen::Matrix4d & pose_end_effector_home)
{
    pose_end_effector_home_ = pose_end_effector_home;
}

void KinematicModel::calEndEffector(const Eigen::VectorXd & joint_angle, Eigen::Matrix4d & pose_end_effector)
{
    pose_end_effector = prodOfExp(twists_, joint_angle, pose_end_effector_home_);
}

void KinematicModel::calEndEffector(const Eigen::VectorXd & joint_angle, Eigen::Vector3d & pos_end_effector)
{
    pos_end_effector = prodOfExp(twists_, joint_angle, Eigen::Vector3d(pose_end_effector_home_.block(0,3,3,1)));
}

void KinematicModel::calSpatialJacobian(const Eigen::VectorXd & joint_angle, Eigen::MatrixXd & spatial_jacobian)
{
    spatial_jacobian = xar::calSpatialJacobian(twists_, joint_angle);
}

void KinematicModel::calJacobian(const Eigen::VectorXd & joint_angle, Eigen::MatrixXd & jacobian)
{
    Eigen::Vector3d pos_end;
    this->calEndEffector(joint_angle, pos_end);
    jacobian = xar::calJacobian(twists_, joint_angle, pos_end);
}

void KinematicModel::calDotSpatialJacobian(const Eigen::VectorXd & joint_angle, const Eigen::VectorXd & joint_velocity,
        Eigen::MatrixXd & dot_spatial_jacobian)
{
    dot_spatial_jacobian = xar::calDotSpatialJacobian(twists_, joint_angle, joint_velocity);
}

void KinematicModel::calDotJacobian(const Eigen::VectorXd & joint_angle, const Eigen::VectorXd & joint_velocity,
        Eigen::MatrixXd & dot_jacobian)
{
    Eigen::Vector3d pos_end;
    calEndEffector(joint_angle, pos_end);
    dot_jacobian = xar::calDotJacobian(twists_, joint_angle, joint_velocity, pos_end);
}

const std::vector<Twist> & KinematicModel::getTwists() const
{
    return twists_;
}

const Eigen::Matrix4d KinematicModel::getEndEffectorHome()
{
    return pose_end_effector_home_;
}

Eigen::MatrixXd calSpatialJacobian(const std::vector<Twist> & twists, const Eigen::VectorXd & joint_angles)
{
    if (twists.size() == joint_angles.rows()) {
        Eigen::MatrixXd spatial_jacobian = Eigen::MatrixXd::Zero(6, twists.size());
        for(unsigned int i = 0; i < twists.size(); i++) {
            Eigen::Matrix4d mat_transf = Eigen::Matrix4d::Identity();
            for (unsigned int j = 0; j < i; ++j) {
                mat_transf *= expMap(twists[j], joint_angles[j]);
            }
            spatial_jacobian.block(0, i, 6, 1) = Ad(mat_transf) * twists[i];
        }

        return spatial_jacobian;
    } else {
        std::cerr << "Error! Unconsistent dimensions." << std::endl;
        exit(1);
    }
}

Eigen::MatrixXd calJacobian(const std::vector<Twist> & twists, const Eigen::VectorXd & joint_angles, const Eigen::Vector3d & pos_end)
{
    Eigen::MatrixXd spatial_jacobian = calSpatialJacobian(twists, joint_angles);
    Eigen::MatrixXd jacobian = spatial_jacobian;
    jacobian.block(0, 0, 3, jacobian.cols()) = spatial_jacobian.block(0,0,3,spatial_jacobian.cols()) -
            wedge(pos_end)*spatial_jacobian.block(3,0,3,spatial_jacobian.cols());

    return jacobian;
}

Eigen::MatrixXd calDotSpatialJacobian(const std::vector<Twist> & twists, const Eigen::VectorXd & joint_angles, const Eigen::VectorXd & joint_velocities)
{
    std::vector<Eigen::Matrix4d> transf_list(twists.size()-1);
    for (unsigned int i = 0; i < transf_list.size(); ++i)
    {
        transf_list[i] = expMap(twists[i], joint_angles[i]);
    }

    Eigen::MatrixXd dot_spatial_jacobian = Eigen::MatrixXd::Zero(6, twists.size());
    for (unsigned int j = 0; j < twists.size(); ++j)
    {
        Twist twist;
        for (unsigned int k = 0; k < j; ++k) {
            Eigen::Matrix4d mat_former = Eigen::Matrix4d::Identity();
            for (unsigned int m = 0; m < k; ++m) {
                mat_former *= transf_list[m];
            }
            Eigen::Matrix4d mat_latter = Eigen::Matrix4d::Identity();
            for (unsigned int m = k; m < j; ++m) {
                mat_latter *= transf_list[m];
            }
            twist += Ad(mat_former) * lieBracket(Twist(twists[k]*joint_velocities[k]), Twist(Ad(mat_latter)*twists[j]));
        }

        dot_spatial_jacobian.block(0, j, 6, 1) =  twist;
    }

    return dot_spatial_jacobian;
}

Eigen::MatrixXd calDotJacobian(const std::vector<Twist> & twists, const Eigen::VectorXd & joint_angles,
        const Eigen::VectorXd & joint_velocities, const Eigen::Vector3d & pos_end)
{
    Eigen::MatrixXd spatial_jacobian = calSpatialJacobian(twists, joint_angles);
    Eigen::MatrixXd jacobian = calJacobian(twists, joint_angles, pos_end);
    Eigen::Vector3d vel_end =  (jacobian * Eigen::MatrixXd(joint_velocities)).block(0,0,3,1);
    Eigen::MatrixXd dot_spatial_jacobian = calDotSpatialJacobian(twists, joint_angles, joint_velocities);

    Eigen::MatrixXd dot_jacobian = dot_spatial_jacobian;
    dot_jacobian.block(0, 0, 3, dot_jacobian.cols()) = dot_spatial_jacobian.block(0, 0, 3, dot_spatial_jacobian.cols())
            - wedge(vel_end)*spatial_jacobian.block(3, 0, 3, dot_spatial_jacobian.cols())
            - wedge(pos_end)*dot_spatial_jacobian.block(3, 0, 3, dot_spatial_jacobian.cols());

    return dot_jacobian;
}

}
