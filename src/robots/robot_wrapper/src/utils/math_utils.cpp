#include <robot_wrapper/utils/math_utils.h>

namespace rw_
{

Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm)
{
  Eigen::Quaternion<double> temp_quat(rotm);
  Eigen::Vector4d quat;
  quat << temp_quat.w(), temp_quat.x(), temp_quat.y(), temp_quat.z();

  quat = quat * (2*(quat(0)>=0)-1); // to avoid discontinuities

  return quat;
}

arma::vec rotm2quat(const arma::mat &rotm)
{
  arma::vec quat(4);

  Eigen::Map<const Eigen::Matrix3d> rotm_wrapper(rotm.memptr());
  Eigen::Map<Eigen::Vector4d> quat_wrapper(quat.memptr());
  quat_wrapper = rotm2quat(rotm_wrapper);

  return quat;
}

arma::mat get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime)
{
  arma::mat retTemp = arma::zeros<arma::mat>(p0.n_rows, 3);

  if (t < 0)
  {
    // before start
    retTemp.col(0) = p0;
  }
  else if (t > totalTime)
  {
    // after the end
    retTemp.col(0) = pT;
  }
  else
  {
    // somewhere betweeen ...
    // position
    retTemp.col(0) = p0 +
                     (pT - p0) * (10 * std::pow(t / totalTime, 3) -
                                  15 * std::pow(t / totalTime, 4) +
                                  6 * std::pow(t / totalTime, 5));
    // vecolity
    retTemp.col(1) = (pT - p0) * (30 * std::pow(t, 2) / std::pow(totalTime, 3) -
                                  60 * std::pow(t, 3) / pow(totalTime, 4) +
                                  30 * std::pow(t, 4) / std::pow(totalTime, 5));
    // acceleration
    retTemp.col(2) = (pT - p0) * (60 * t / std::pow(totalTime, 3) -
                                  180 * std::pow(t, 2) / std::pow(totalTime, 4) +
                                  120 * std::pow(t, 3) /std::pow(totalTime, 5));
  }

  // return vector
  return retTemp;
}

arma::vec quatProd(const arma::vec &quat1, const arma::vec &quat2)
{
  arma::vec quat12(4);

  double n1 = quat1(0);
  arma::vec e1 = quat1.subvec(1,3);

  double n2 = quat2(0);
  arma::vec e2 = quat2.subvec(1,3);

  quat12(0) = n1*n2 - arma::dot(e1,e2);
  quat12.subvec(1,3) = n1*e2 + n2*e1 + arma::cross(e1,e2);

  return quat12;
}

arma::vec quatExp(const arma::vec &v_rot, double zero_tol)
{
  arma::vec quat(4);
  double norm_v_rot = arma::norm(v_rot);
  double theta = norm_v_rot;

  if (norm_v_rot > zero_tol)
  {
    quat(0) = std::cos(theta/2);
    quat.subvec(1,3) = std::sin(theta/2)*v_rot/norm_v_rot;
  }
  else{
    quat << 1 << 0 << 0 << 0;
  }

  return quat;
}

arma::vec quatLog(const arma::vec &quat, double zero_tol)
{
  arma::vec e = quat.subvec(1,3);
  double n = quat(0);

  if (n > 1) n = 1;
  if (n < -1) n = -1;

  arma::vec omega(3);
  double e_norm = arma::norm(e);

  if (e_norm > zero_tol) omega = 2*std::atan2(e_norm,n)*e/e_norm;
  else omega = arma::vec().zeros(3);

  return omega;
}

arma::vec quatInv(const arma::vec &quat)
{
  arma::vec quatI(4);

  quatI(0) = quat(0);
  quatI.subvec(1,3) = - quat.subvec(1,3);

  return quatI;
}

}
