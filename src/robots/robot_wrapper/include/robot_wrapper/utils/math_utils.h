#ifndef AS64_ROBOT_WRAPPER_MATH_UTILS_H
#define AS64_ROBOT_WRAPPER_MATH_UTILS_H

#include <cmath>
#include <armadillo>
#include <Eigen/Dense>

namespace rw_
{

Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm);

arma::vec rotm2quat(const arma::mat &rotm);

arma::mat get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime);

arma::vec quatProd(const arma::vec &quat1, const arma::vec &quat2);

arma::vec quatExp(const arma::vec &v_rot, double zero_tol=1e-16);

arma::vec quatLog(const arma::vec &quat, double zero_tol=1e-16);

arma::vec quatInv(const arma::vec &quat);

}

#endif // AS64_ROBOT_WRAPPER_MATH_UTILS_H