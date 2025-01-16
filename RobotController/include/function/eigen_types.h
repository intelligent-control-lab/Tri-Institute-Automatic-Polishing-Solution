#ifndef _EIGEN_TYPES_H
#define _EIGEN_TYPES_H

#include <vector>
#include <Eigen/Dense>


using RotMat = typename Eigen::Matrix<double, 3, 3>;


using HomoMat = typename Eigen::Matrix<double, 4, 4>;


using Vec2 = typename Eigen::Matrix<double, 2, 1>;


using Vec3 = typename Eigen::Matrix<double, 3, 1>;


using Vec4 = typename Eigen::Matrix<double, 4, 1>;


using Vec6 = typename Eigen::Matrix<double, 6, 1>;


using Vec7 = typename Eigen::Matrix<double, 7, 1>;


using Mat3 = typename Eigen::Matrix<double, 3, 3>;


using Mat4 = typename Eigen::Matrix<double, 4, 4>;


using Quat = typename Eigen::Quaternion<double>;


using SVec = typename Eigen::Matrix<double, 6, 1>;


using SXform = typename Eigen::Matrix<double, 6, 6>;


using Mat6 = typename Eigen::Matrix<double, 6, 6>;


using Mat36 = typename Eigen::Matrix<double, 3, 6>;



using Mat37 = typename Eigen::Matrix<double, 3, 7>;


using Mat67 = typename Eigen::Matrix<double, 6, 7>;


using Mat76 = typename Eigen::Matrix<double, 7, 6>;


using Mat7 = typename Eigen::Matrix<double, 7, 7>;


using Mat12 = typename Eigen::Matrix<double, 12, 12>;


using Mat34 = Eigen::Matrix<double, 3, 4>;


using Mat23 = Eigen::Matrix<double, 2, 3>;


using Mat4 = typename Eigen::Matrix<double, 4, 4>;


using DVec = typename Eigen::Matrix<double, Eigen::Dynamic, 1>;


using DMat = typename Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;


using D6Mat = typename Eigen::Matrix<double, 6, Eigen::Dynamic>;


using D3Mat = typename Eigen::Matrix<double, 3, Eigen::Dynamic>;

bool EigenVec6ToArray(const Vec6 vector,double* array);
Vec6 ArrayToEigenVec6(const double* array);

bool EigenVec7ToArray(const Vec7 vector,double* array);
Vec7 ArrayToEigenVec7(const double* array);

void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped);
#endif
