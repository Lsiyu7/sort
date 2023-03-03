#ifndef KINEMATICMODEL_H
#define KINEMATICMODEL_H

#include <vector>
#include "twist.h"

namespace xar {

/*!
 * \class KinematicModel
 * \brief KinematicModel is for calculating robot's kinematics.
 * \details KinematicModel is for calculating robot's kinematics, including the end-effector's pose, position,
 * spatial jacobian matrix, jacobian matrix, and the differentiation of the jacobian matrix. Before calculate these
 * kinematic properties, user should set the twists and the end-effector's pose at home configuration, and provide
 * necessary variables.
 */
class KinematicModel
{
public:
    KinematicModel();
    ~KinematicModel();

    /*!
     * \fn void KinematicModel::setTwists(const std::vector<Twist> & ptrTwistList)
     * \brief Set the robot's twists.
     * \param[in] twists A list of Twist.
     */
    void setTwists(const std::vector<Twist> & twists);

    /*!
     * \fn void KinematicModel::setEndEffectorHome(const Eigen::Matrix4d & pose_end_effector_home)
     * \brief Set the end-effector's pose at home (zero) configuration.
     * \param[in] pose_end_effector_home A 4x4 matrix denoting the end-effector's pose.
     */
    void setEndEffectorHome(const Eigen::Matrix4d & pose_end_effector_home);

    /*!
     * \fn void KinematicModel::calEndEffector(const Eigen::VectorXd & joint_angle, Eigen::Matrix4d & pose_end_effector)
     * \brief Calculate the end-effector's pose at the given joint configuraton.
     * \param[in] joint_angles A set joint angles.
     * \param[out] pose_end_effector A 4x4 matrix denoting the end-effector's pose.
     */
    void calEndEffector(const Eigen::VectorXd & joint_angle, Eigen::Matrix4d & pose_end_effector);

    /*!
     * \fn void KinematicModel::calEndEffector(const Eigen::VectorXd & joint_angle, Eigen::Vector3d & pos_end_effector)
     * \brief Calculate the end-effector's position at the given joint configuraton.
     * \param[in] joint_angles A set joint angles.
     * \param[out] pos_end_effector A 3x1 vector denoting the end-effector's position.
     */
    void calEndEffector(const Eigen::VectorXd & joint_angle, Eigen::Vector3d & pos_end_effector);

    /*!
     * \fn void KinematicModel::calSpatialJacobian(const Eigen::VectorXd & joint_angle, Eigen::MatrixXd & spatial_jacobian)
     * \brief Calculate the spatial jacobian matrix.
     * \param[in] joint_angles A set of joint angles.
     * \param[out] spatial_jacobian The spatial jacobian matrix.
     */
    void calSpatialJacobian(const Eigen::VectorXd & joint_angle, Eigen::MatrixXd & spatial_jacobian);

    /*!
     * \fn void KinematicModel::calSpatialJacobian(const Eigen::VectorXd & joint_angle, Eigen::MatrixXd & jacobian)
     * \brief Calculate the jacobian matrix.
     * \param[in] joint_angles A set of joint angles.
     * \param[out] jacobian The jacobian matrix.
     */
    void calJacobian(const Eigen::VectorXd & joint_angle, Eigen::MatrixXd & jacobian);

    /*!
     * \fn void KinematicModel::calDotSpatialJacobian(const Eigen::VectorXd & joint_angle, const Eigen::VectorXd & joint_velocity,
            Eigen::MatrixXd & dot_spatial_jacobian)
     * \brief Calculate the differentiation of the spatial jacobian matrix.
     * \param[in] joint_angles A vector denoting joint angles.
     * \param[in] joint_velocities A vector denoting joint velosities.
     * \param[out] dot_spatial_jacobian The differentiation of the spatial jacobian matrix.
     */
    void calDotSpatialJacobian(const Eigen::VectorXd & joint_angle, const Eigen::VectorXd & joint_velocity,
            Eigen::MatrixXd & dot_spatial_jacobian);

    /*!
     * \fn void KinematicModel::calDotSpatialJacobian(const Eigen::VectorXd & joint_angle, const Eigen::VectorXd & joint_velocity,
            Eigen::MatrixXd & dot_spatial_jacobian)
     * \brief Calculate the differentiation of the jacobian matrix.
     * \param[in] joint_angles A vector denoting joint angles.
     * \param[in] joint_velocities A vector denoting joint velosities.
     * \param[out] dot_jacobian The differentiation of the jacobian matrix.
     */
    void calDotJacobian(const Eigen::VectorXd &joint_angle, const Eigen::VectorXd & joint_velocity,
            Eigen::MatrixXd & dot_jacobian);

    const std::vector<Twist> & getTwists() const;
    const Eigen::Matrix4d getEndEffectorHome();

private:
    std::vector<Twist> twists_;
    Eigen::Matrix4d pose_end_effector_home_;
};

Eigen::MatrixXd calSpatialJacobian(const std::vector<Twist> & twists, const Eigen::VectorXd & joint_angles);
Eigen::MatrixXd calJacobian(const std::vector<Twist> & twists, const Eigen::VectorXd & joint_angles, const Eigen::Vector3d & pos_end);
Eigen::MatrixXd calDotSpatialJacobian(const std::vector<Twist> & twists, const Eigen::VectorXd & joint_angles, const Eigen::VectorXd & joint_velocities);
Eigen::MatrixXd calDotJacobian(const std::vector<Twist> & twists, const Eigen::VectorXd &joint_angles,
        const Eigen::VectorXd & joint_velocities, const Eigen::Vector3d & pos_end);

}

#endif // KINEMATICMODEL_H
