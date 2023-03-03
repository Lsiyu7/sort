#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace xar {

typedef Eigen::Quaterniond Quaternion;
typedef Eigen::AngleAxisd ExpCoord;

/*!
 * \class RpyEuler
 * \brief Roll-Pitch-Yaw Euler angle.
 * \detail Perform the rotations as the following sequence:
 * 1) Rotate the reference frame by the angle $\psi$ about axis X (Yaw);
 * 2) Rotate the reference frame by the angle $\theta$ about axis Y (Pitch);
 * 3) Rotate the reference frame by the angle $\phi$ about axis Z (Roll).
 * Hence, the compound rotation matrix is:
 * $\mathcal{R} = \mathcal{R}_z (\phi) * \mathcal{R}_y (\theta) * \mathcal{R}_x (\psi)$.
 */
class RpyEuler : public Eigen::Vector3d
{
public:
    RpyEuler();
    RpyEuler(const Eigen::Vector3d & vector);
    RpyEuler(double roll, double pitch, double yaw);

    RpyEuler & operator=(const Eigen::Vector3d & vector);

    /*!
     * \fn Eigen::Matrix3d toMatrix()
     * \brief Convert the RPY Euler angle to a 3x3 rotation matrix.
     * \return The corresponding rotation matrix.
     */
    Eigen::Matrix3d toMatrix();

    /*!
     * \fn Quaternion toQuat()
     * \brief Convert the RPY Euler angle to a unit quaternion.
     * \return The corresponding unit quaternion.
     */
    Quaternion toQuat();
};


/*!
 * \class ZyzEuler
 * \brief Z-Y-Z Euler angle.
 * \detail Perform the rotations as the following sequence:
 * 1) Rotate the reference frame by the angle $\phi$ about axis $Z$;
 * 2) Rotate the reference frame by the angle $\theta$ about axis $Y^\prime$;
 * 3) Rotate the reference frame by the angle $\psi$ about axis $Z^{\prime\prime}$.
 * Hence, the compound rotation matrix is:
 * $\mathcal{R} = \mathcal{R}_z (\phi) * \mathcal{R}_{y^\prime} (\theta) * \mathcal{R}_{z^{\prime\prime}} (\psi)$.
 */
class ZyzEuler : public Eigen::Vector3d
{
public:
    ZyzEuler();
    ZyzEuler(const Eigen::Vector3d & vector);
    ZyzEuler(double alpha, double beta, double gamma);

    ZyzEuler & operator=(const Eigen::Vector3d & vector);

    /*!
     * \fn Eigen::Matrix3d toMatrix()
     * \brief Convert the Z-Y-Z Euler angle to a 3x3 rotation matrix.
     * \return The corresponding rotation matrix.
     */
    Eigen::Matrix3d toMatrix();

    /*!
     * \fn Eigen::Matrix3d toMatrix()
     * \brief Convert the Z-Y-Z Euler angle to a unit quaternion.
     * \return The corresponding unit quaternion.
     */
    Quaternion toQuat();
};

/*!
 * \fn RpyEuler matrix2rpyEuler(const Eigen::Matrix3d & matrix)
 * \brief Convert rotation matrix to Roll-Pitch-Yaw Euler angle.
 * \param [in] matrix Rotation matrix.
 * \return The corresponding Roll-Pitch-Yaw Euler angle.
 */
RpyEuler matrix2rpyEuler(const Eigen::Matrix3d & matrix);

/*!
 * \fn ZyzEuler matrix2zyzEuler(const Eigen::Matrix3d & matrix)
 * \brief Convert rotation matrix to Z-Y-Z Euler angle.
 * \param [in] matrix Rotation matrix.
 * \return The corresponding Z-Y-Z Euler angle.
 */
ZyzEuler matrix2zyzEuler(const Eigen::Matrix3d & matrix);

}

#endif // ORIENTATION_H
