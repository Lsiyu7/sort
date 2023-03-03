#ifndef TWIST_H
#define TWIST_H

#include <vector>
#include <Eigen/Dense>

namespace xar {

/*!
 * \class Twist
 * \brief Twist is for representing the twist (motion) and screw (force).
 * \detail Represent the linear and angular velocities (motion) or force and moment (screw) using a 6x1 vector.
 * The first 3 rows denotes the linear velocity or force; the last 3 rows denotes the angular velocity or moment.
 */
class Twist : public Eigen::VectorXd
{
public:
    Twist();
    Twist(const Eigen::VectorXd &vector);

    /*!
     * \fn Twist::Twist()
     * \brief Generate a twist based a rotation axis and a point through which the rotation axis passes.
     */
    Twist(const Eigen::Vector3d & omega, const Eigen::Vector3d & point);

    Twist & operator=(const Eigen::VectorXd & vector);

    /*!
     * \fn Eigen::Vector3d Twist::omg()
     * \brief Get the angular part of the twist.
     * \return Angular velocity.
     */
    Eigen::Vector3d omg() const;

    /*!
     * \fn Eigen::Vector3d Twist::vel()
     * \brief Get the linear part of the twist.
     * \return Linear velocity.
     */
    Eigen::Vector3d vel() const;

    /*!
     * \fn Eigen::Vector3d Twist::vel()
     * \brief Get the matrix representation of the twist.
     * \return A 4x4 matrix denoting the twist.
     */
    Eigen::Matrix4d wedge();
};

Twist operator*(double scalar, const Twist & twist);
Twist operator*(const Twist & twist, double scalar);
Twist operator/(const Twist & twist, double scalar);

Twist operator+(const Eigen::MatrixXd & matrix, const Twist & twist);
Twist operator-(const Eigen::MatrixXd & matrix, const Twist & twist);
Twist operator*(const Eigen::MatrixXd & matrix, const Twist & twist);

Twist operator+(const Twist & twist, const Eigen::MatrixXd & matrix);
Twist operator-(const Twist & twist, const Eigen::MatrixXd & matrix);

Eigen::Matrix3d wedge(const Eigen::Vector3d & omega);
Eigen::Matrix4d wedge(const Twist & twist);
Eigen::Vector3d vee(const Eigen::Matrix3d & matrix);
Twist vee(const Eigen::Matrix4d & matrix);

Eigen::Matrix3d expMap(const Eigen::Vector3d & omega);
Eigen::Matrix3d expMap(const Eigen::Vector3d & omega, double theta);
Eigen::Matrix4d expMap(const Twist & twist);
Eigen::Matrix4d expMap(const Twist & twist, double theta);

/*!
 * \fn Eigen::Matrix4d invTransf(const Eigen::Matrix4d & matrixA)
 * \brief Calculate the inverse of an SE(3) transformation matrix.
 * \param[in] matrix A 4x4 SE(3) transformation matrix.
 * \return Inverse of the given matrix.
 */
Eigen::Matrix4d invTransf(const Eigen::Matrix4d & matrix);

/*!
 * \fn Twist lieBracket(const Twist & twist_1, const Twist & twist_2)
 * \brief Lie bracket operator, muliplication between twist.
 * \param [in] twist_1 The 1st twist.
 * \param [in] twist_2 The 2nd twist.
 * \return Operation result.
 */
Twist lieBracket(const Twist & twist_1, const Twist & twist_2);

/*!
 * \fn Twist lieBrace(const Twist & twist_1, const Twist & twist_2)
 * \brief Lie brace operator, mulplication between twist and screw.
 * \param [in] twist_1 A twist or a screw.
 * \param [in] twist_2 A screw or a twist.
 * \return Operation result.
 */
Twist lieBrace(const Twist & twist_1, const Twist & twist_2);

/*!
 * \fn Eigen::MatrixXd Ad(const Eigen::MatrixXd & matrix)
 * \brief Calculate the adjoint transformation matrix of a given SE(3).
 * \param matrix A 4x4 transformation matrix.
 * \return The adjoint transformation of the given matrix.
 */
Eigen::MatrixXd Ad(const Eigen::Matrix4d & matrix);

/*!
 * \fn Eigen::MatrixXd invAd(const Eigen::MatrixXd & matrix)
 * \brief Calculate the inverse of an ajoint transformation matrix, based its property.
 * \param matrix An adjoint matrix from an SE(3) transformation.
 * \return The inverse of the given adjoint transformation matrix.
 */
Eigen::MatrixXd invAd(const Eigen::MatrixXd & matrix);

/*!
 * \fn Eigen::Matrix4d prodOfExp(const std::vector<Twist> & twists, const Eigen::VectorXd & joint_angle,
 * 		const Eigen::Matrix4d & pose_end_effector_home)
 * \brief Calculate the end-effector's pose, based on a set of joint angles.
 * \param [in] twists A list of twists denoting the structure of the robot.
 * \param [in] joint_angle A set of joint angles denoting the joint configuration.
 * \param [in] pose_end_effector_home A 4x4 matrix denoting the pose of the end-effector at home (zero) configuration.
 * \return The end-effector's pose at the joint configuration.
 */
Eigen::Matrix4d prodOfExp(const std::vector<Twist> & twists, const Eigen::VectorXd & joint_angle,
        const Eigen::Matrix4d & pose_end_effector_home = Eigen::Matrix4d::Identity());

/*!
 * \fn Eigen::Vector3d prodOfExp(const std::vector<Twist> & twists, const Eigen::VectorXd & joint_angle,
 * 		const Eigen::Vector3d & pos_end_effector_home)
 * \brief Calculate the end-effector's position, based on a set of joint angles.
 * \param [in] twists A list of twists denoting the structure of the robot.
 * \param [in] joint_angle A set of joint angles denoting the joint configuration.
 * \param [in] pos_end_effector_home A 3x1 vector denoting the position of the end-effector at home (zero) configuration.
 * \return The end-effector's position at the joint configuration.
 */
Eigen::Vector3d prodOfExp(const std::vector<Twist> & twists, const Eigen::VectorXd & joint_angle,
        const Eigen::Vector3d & pos_end_effector_home);

}

#endif // TWIST_H
