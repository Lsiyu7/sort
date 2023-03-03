#ifndef DYNAMICMODEL_H
#define DYNAMICMODEL_H

#include <vector>
#include "twist.h"

namespace xar {

/*!
 * \class DyanmicModel
 * \brief DynamicModel is for calculating robot's dynamics.
 * \details KinematicModel is for calculating robot's dynamics, including the 3 matrices in the dynamic equation.
 * Before calculation, user should set the necessary variables for the robot.
 */
class DynamicModel
{
public:
    DynamicModel();
    ~DynamicModel();

    /*!
     * \fn void DynamicModel::setTwists(const std::vector<Twist> * twistList)
     * \brief Set the robot's twists.
     * \param[in] twists A list of Twist.
     */
    void setTwists(const std::vector<Twist> & twists);

    /*!
     * \fn void DynamicModel::setPoses(const std::vector<Matrix> * poseList)
     * \brief Set the pose at home (zero) configuration for each link.
     * \param[in] link_poses A list of 4x4 matrices denoting the links' poses.
     */
    void setLinkPoses(const std::vector<Eigen::Matrix4d> & link_poses);

    /*!
     * \fn void DynamicModel::setLinkMass(const std::vector<double> & link_mass)
     * \brief Set the mass for each link.
     * \param[in] link_mass The links' mass.
     */
    void setLinkMass(const std::vector<double> & link_mass);

    /*!
     * \fn void DynamicModel::setInertias(const std::vector<Eigen::MatrixXd> & link_inertia)
     * \brief Set the inertia matrix for each link. Every inertia matrix is a 6x6 matric, with respect ot the link's
     * own frame.
     * \param[in] link_inertia The links' inertia matrices.
     */
    void setLinkInertia(const std::vector<Eigen::MatrixXd> & link_inertia);

    /*!
     * \fn void DynamicModel::setGravity(const Eigen::Vector3d & gravity)
     * \brief Set the gravity.
     * \param[in] gravity The gravity.
     */
    void setGravity(const Eigen::Vector3d & gravity);

    /*!
     * \fn void DynamicModel::calDynamics(const Eigen::VectorXd & joint_angles, const Eigen::VectorXd & joint_velocities,
            Eigen::MatrixXd & mat_m, Eigen::MatrixXd & mat_c, Eigen::MatrixXd & mat_n
     * \brief Calculate the dynamic equation.
     * \param[in] joint_angles A vector denoting joint angles.
     * \param[in] joint_velocities A vector denoting joint velosities.
     * \param[out] mat_m The M matrix in the dynamic equation.
     * \param[out] mat_c The C matrix in the dynamic equation.
     * \param[out] mat_n The N matrix in the dynamic equation.
    */
    void calDynamics(const Eigen::VectorXd & joint_angles, const Eigen::VectorXd & joint_velocities,
            Eigen::MatrixXd & mat_m, Eigen::MatrixXd & mat_c, Eigen::MatrixXd & mat_n);

    const std::vector<Twist> & getTwists();
    const std::vector<Eigen::Matrix4d> & getLinkPoses();
    const std::vector<double> & getLinkMass();
    const std::vector<Eigen::MatrixXd> & getLinkInertia();
    const Eigen::Vector3d & getGravity();

private:
    std::vector<Twist> twists_;
    std::vector<Eigen::Matrix4d> link_poses_;
    std::vector<Eigen::MatrixXd> link_inertia_;
    std::vector<double> link_mass_;
    std::vector<Eigen::MatrixXd> link_inertia_prime_;
    Eigen::Vector3d gravity_;

    // Auxiliary functions.
    const Eigen::MatrixXd AdIJ(int idx_i, int idx_j, const Eigen::VectorXd & joint_angles);
    double calMatMIJ(int idx_i, int idx_j, const Eigen::VectorXd & joint_angles);
    double calPDiffMIJK(int idx_i, int idx_j, int idx_k, const Eigen::VectorXd & joint_angles);
    const Eigen::MatrixXd calMatM(const Eigen::VectorXd & joint_angles);
    const Eigen::MatrixXd calMatC(const Eigen::VectorXd & joint_angles, const Eigen::VectorXd & joint_velocities);
    const Eigen::MatrixXd calMatN(const Eigen::VectorXd & joint_angles);
};

}

#endif // DYNAMICMODEL_H
