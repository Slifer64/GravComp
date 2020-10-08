#include <ur_robot/utils.h>
#include <cmath>
#include <vector>

namespace ur_
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

arma::mat quat2rotm(const arma::vec &quat)
{
  double qw=quat(0), qx=quat(1), qy=quat(2), qz=quat(3);

  arma::mat rotm;
  rotm = {{1 - 2*qy*qy - 2*qz*qz,      2*qx*qy - 2*qz*qw,      2*qx*qz + 2*qy*qw},
          {    2*qx*qy + 2*qz*qw,  1 - 2*qx*qx - 2*qz*qz,      2*qy*qz - 2*qx*qw},
          {    2*qx*qz - 2*qy*qw,      2*qy*qz + 2*qx*qw,  1 - 2*qx*qx - 2*qy*qy}};

  return rotm;
}

arma::vec quatLog(const arma::vec &quat, double zero_tol)
{
  arma::vec v = quat.subvec(1,3);
  double u = quat(0);

  arma::vec omega(3);
  double v_norm = arma::norm(v);

  if (v_norm > zero_tol) omega = 2*std::atan2(v_norm,u)*v/v_norm;
  else omega = arma::vec().zeros(3);

  return omega;
}

std::vector<arma::vec> get5thOrder(double t, const arma::vec p0, const arma::vec pT, double totalTime)
{
  std::vector<arma::vec> retTemp(3);

  int N_dim = p0.n_rows;

  if (t < 0)
  {
    // before start
    retTemp[0] = p0;
    retTemp[1] = arma::vec().zeros(N_dim);
    retTemp[2] = arma::vec().zeros(N_dim);
  }
  else if (t > totalTime)
  {
    // after the end
    retTemp[0] = pT;
    retTemp[1] = arma::vec().zeros(N_dim);
    retTemp[2] = arma::vec().zeros(N_dim);
  }
  else
  {
    // somewhere betweeen ...
    // position
    retTemp[0] = p0 +
                     (pT - p0) * (10 * std::pow(t / totalTime, 3) -
                     15 * std::pow(t / totalTime, 4) +
                     6 * std::pow(t / totalTime, 5));
    // vecolity
    retTemp[1] = (pT - p0) * (30 * std::pow(t, 2) / std::pow(totalTime, 3) -
                     60 * std::pow(t, 3) / std::pow(totalTime, 4) +
                     30 * std::pow(t, 4) / std::pow(totalTime, 5));
    // acceleration
    retTemp[2] = (pT - p0) * (60 * t / std::pow(totalTime, 3) -
                     180 * std::pow(t, 2) / std::pow(totalTime, 4) +
                     120 * std::pow(t, 3) / std::pow(totalTime, 5));
  }

  // return vector
  return retTemp;
}

void print_err_msg(const std::string &msg)
{
  std::cerr << "\033[1m\033[31m" << "[lwr4p+ ERROR]: " << msg << "\033[0m";
}

void print_info_msg(const std::string &msg)
{
  std::cout << "\033[1m\033[34m" << "[lwr4p+ INFO]: " << msg << "\033[0m";
}

void print_warn_msg(const std::string &msg)
{
  std::cout << "\033[1m\033[33m" << "[lwr4p+ WARNING]: " << msg << "\033[0m";
}

}; // namespace ur_

