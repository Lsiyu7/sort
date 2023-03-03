#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>

namespace xar {

extern const double PI;
extern const double TOLERANCE;

double deg2rad(double degree);
double rad2deg(double rad);

/*!
 * \fn double sqrt(double x, double tolerance = TOLERANCE)
 * \brief Calculate the square root under some tolerance.
 * \details Calculate the square root under some tolerance. During the calculation in robotics, some values which
 * are theoretically not less than 0 minght below 0, due to the calculation error. To counqure this, values which
 * are in the range of [-tolerance, 0] will be regarded to be 0; while values which are < -tolerance will be regarded
 * minus.
 * \param x A scalar.
 * \param tolerance The tolerance. If 'x' is in [-tolerance, 0], it will be set to 0.
 * \return The square root.
 */
double sqrt(double x, double tolerance = TOLERANCE);

/*!
 * \fn double asin(double x, double tolerance = TOLERANCE)
 * \brief Arcsine function with some tolerance.
 * \details Arcsine function with some tolerance. During the calculation in robotics, Value of sine function might
 * exceed [-1, 1]. To counqure this, values which are in the range of [-1-tolerance, 1+tolerance] are acceptable.
 * If the value is in [-1-tolerance, -1], it will be set to -1; if the value is in [1, 1+tolerance], it will be set to 1.
 * \param x A scalar.
 * \param tolerance The tolerance.
 * \return The arcsine value.
 */
double asin(double x, double tolerance = TOLERANCE);

/*!
 * \fn double acos(double x, double tolerance = TOLERANCE)
 * \brief Arccosine function with some tolerance.
 * \details Arccosine function with some tolerance. During the calculation in robotics, Value of cosine function might
 * exceed [-1, 1]. To counqure this, values which are in the range of [-1-tolerance, 1+tolerance] are acceptable.
 * If the value is in [-1-tolerance, -1], it will be set to -1; if the value is in [1, 1+tolerance], it will be set to 1.
 * \param x A scalar.
 * \param tolerance The tolerance.
 * \return The arccosine value.
 */
double acos(double x, double tolerance = TOLERANCE);

Eigen::VectorXd unitize(const Eigen::VectorXd vector);

Eigen::Vector4d vectHomo(const Eigen::Vector3d & free_vector);
Eigen::Vector4d posHomo(const Eigen::Vector3d & pos_vector);
Eigen::Vector3d dehomo(const Eigen::Vector4d & homo_vector);

}



#endif // COMMON_H
