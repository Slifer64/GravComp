#include <grav_comp/robot/robot.h>
#include <ros/package.h>
#include <io_lib/parser.h>

Robot::Robot(const ToolEstimator *tool_est):tool_estimator(tool_est)
{
  mode_name.resize(6);
  mode_name[0] = "JOINT_POS_CONTROL";
  mode_name[1] = "JOINT_TORQUE_CONTROL";
  mode_name[2] = "CART_VEL_CTRL";
  mode_name[3] = "FREEDRIVE";
  mode_name[4] = "IDLE";
  mode_name[5] = "STOPPED";
}

Robot::~Robot()
{

}

Robot::Mode Robot::getMode() const
{
  return mode.get();
}

std::string Robot::getModeName() const
{
  return mode_name[getMode()];
}

Eigen::Vector4d Robot::rotm2quat(Eigen::Matrix3d rotm) const
{
    Eigen::Quaternion<double> temp_quat(rotm);
    Eigen::Vector4d quat;
    quat << temp_quat.w(), temp_quat.x(), temp_quat.y(), temp_quat.z();

    quat = quat * (2*(quat(0)>=0)-1); // to avoid discontinuities

    return quat;
}

arma::vec Robot::rotm2quat(const arma::mat &rotm) const
{
  arma::vec quat(4);

  Eigen::Map<const Eigen::Matrix3d> rotm_wrapper(rotm.memptr());
  Eigen::Map<Eigen::Vector4d> quat_wrapper(quat.memptr());
  quat_wrapper = rotm2quat(rotm_wrapper);

  return quat;
}

arma::mat Robot::get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime)
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
                     (pT - p0) * (10 * pow(t / totalTime, 3) -
                     15 * pow(t / totalTime, 4) +
                     6 * pow(t / totalTime, 5));
    // vecolity
    retTemp.col(1) = (pT - p0) * (30 * pow(t, 2) / pow(totalTime, 3) -
                     60 * pow(t, 3) / pow(totalTime, 4) +
                     30 * pow(t, 4) / pow(totalTime, 5));
    // acceleration
    retTemp.col(2) = (pT - p0) * (60 * t / pow(totalTime, 3) -
                     180 * pow(t, 2) / pow(totalTime, 4) +
                     120 * pow(t, 3) / pow(totalTime, 5));
  }

  // return vector
  return retTemp;
}
