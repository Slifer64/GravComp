#include <robo_lib/singular_value_filter.h>

namespace as64_
{

namespace robo_
{

SingularValueFilter::SingularValueFilter(double sigma_min, double shape_f)
{
  setSigmaMin(sigma_min);
  setShapeFactor(shape_f);
}

void SingularValueFilter::setSigmaMin(double sigma_min)
{
  sigma0 = sigma_min;
}

void SingularValueFilter::setShapeFactor(double shape_f)
{
  v = shape_f;
}

double SingularValueFilter::getSigmaMin() const
{
  return sigma0;
}

double SingularValueFilter::getShapeFactor() const
{
  return v;
}

double SingularValueFilter::filterEigVal(double sigma) const
{
  double filt_sigma = ( std::pow(sigma,3.0) + v*std::pow(sigma,2.0) + 2*sigma + 2*sigma0 ) / (std::pow(sigma,2.0) + v*sigma + 2);

  return filt_sigma;
}

Eigen::MatrixXd SingularValueFilter::pinv(const Eigen::MatrixXd &M) const
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd S = svd.singularValues().asDiagonal();
  Eigen::MatrixXd U = svd.matrixU();
  Eigen::MatrixXd V = svd.matrixV();

  // Eigen::MatrixXd M_reconstr = U*S*V.transpose();

  for (int i=0;i<S.cols(); i++) S(i,i) = 1/filterEigVal(S(i,i));

  return V*S*U.transpose();
}

arma::mat SingularValueFilter::pinv(const arma::mat &M) const
{
  arma::mat inv_M(M.n_cols, M.n_rows);
  Eigen::Map<Eigen::MatrixXd> inv_M_wrapper(inv_M.memptr(), inv_M.n_rows, inv_M.n_cols);
  Eigen::Map<const Eigen::MatrixXd> M_wrapper(M.memptr(), M.n_rows, M.n_cols);
  inv_M_wrapper = this->pinv(M_wrapper);

  return inv_M;
}

} // namespace math_

} // namespace as64_
