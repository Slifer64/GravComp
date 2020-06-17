#ifndef ROBO_LIB_SINGULAR_VALUE_FILTER_H
#define ROBO_LIB_SINGULAR_VALUE_FILTER_H

#include <cmath>
#include <Eigen/Dense>
#include <armadillo>

namespace as64_
{

namespace robo_
{

/**
 * Implements singular value filtering on the eigenvalues of a matrix based on
 * the approach in "Redundant Inverse Kinematics: Experimental Comparative Review
 * and Two Enhancements, 2012 IEEE/RSJ International Conference on Intelligent Robots and Systems"
 */
class SingularValueFilter
{
public:
  /**
  * @brief Filters the eigenvalues of a matrix so that the minimum is 'sigma_min'.
           The greater the value of the shape factor, the less the error between
           the actual and filtered eigenvalues. However, the following constraints
           must hold: shape_f > sigma_min  and  shape_f*sigma_min < 2
  * @param sigma_min, minimum allowable eigen value.
  * @param shape_f, the greater the shape factor the closer are the filtered eigenvalues to the actual ones.
  */
  SingularValueFilter(double sigma_min = 1e-3, double shape_f = 20);

  void setSigmaMin(double sigma_min);
  void setShapeFactor(double shape_f);

  double getSigmaMin() const;
  double getShapeFactor() const;

  double filterEigVal(double sigma) const;

  // filtered pseudo-inverse
  Eigen::MatrixXd pinv(const Eigen::MatrixXd &M) const;
  arma::mat pinv(const arma::mat &M) const;
private:
  double sigma0; ///> minimum allowable eigen value
  double v; ///> shape factor
};

} // namespace robo_

} // namespace as64_

#endif // ROBO_LIB_SINGULAR_VALUE_FILTER_H
