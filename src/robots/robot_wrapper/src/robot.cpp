#include <robot_wrapper/robot.h>

namespace rw_
{

#define Robot_fun_ std::string("[Robot::") + __func__ + "]: "

std::vector<std::string> Robot::mode_name =
{
  "JOINT_POS_CONTROL",
  "JOINT_TORQUE_CONTROL",
  "CART_VEL_CTRL",
  "FREEDRIVE",
  "ADMITTANCE",
  "IDLE",
  "STOPPED",
  "PROTECTIVE_STOP"
};

Robot::Robot()
{
  vel_CLIK = 2;
  rotVel_CLIK = 0.8;

  R_et = arma::mat().eye(3,3);
  p_et = arma::vec().zeros(3);

  external_stop_ = false;

  global_time_sec = 0;

  // ============================================================

  adm_ctrl.reset(new AdmittanceCtrl(this));
}

Robot::~Robot()
{

}

void Robot::setEeToolTrans(const arma::vec &p)
{
  this->p_et = p;
}

void Robot::setEeToolRot(const arma::mat &R)
{
  arma::mat I = arma::eye(3,3);
  arma::mat I_hat = R*R.t();
  arma::mat I2_hat = R.t()*R;

  double eps = 1e-6;
  for (int i=0; i<3 ;i++)
  {
    for (int j=0; j<3; j++)
    {
      if (std::fabs(I(i,j) - I_hat(i,j)) > eps || std::fabs(I(i,j) - I2_hat(i,j)) > eps)
        throw std::runtime_error(Robot_fun_ + "Ee-tool rotm is not orthonormal\n");
    }
  }
  if (std::fabs(arma::det(R) -1) > eps) throw std::runtime_error(Robot_fun_ + "Ee-tool rotm has det(.) != 1\n");

  this->R_et = R;
  this->q_et = rotm2quat(R);
}

void Robot::setExternalStop(bool set)
{
  cart_vel_cmd.set(arma::vec().zeros(6));
  jtorque_cmd.set(arma::vec().zeros(getNumOfJoints()));
  adm_ctrl->stop();

  setRobotIdle();
  mode.set(rw_::Mode::IDLE);
  cmd_mode.set(rw_::Mode::IDLE);
  
  external_stop_ = set;
}

void Robot::setVelCLICK(double vel_click, double rotVel_click)
{
  vel_CLIK = vel_click;
  rotVel_CLIK = rotVel_click;
}

void Robot::setTaskVelocity(const arma::vec &vel, const arma::vec &pos, const arma::vec &quat)
{
  arma::vec robot_pos = this->getTaskPosition();
  arma::vec robot_quat = this->getTaskOrientation();
  if (arma::dot(robot_quat,quat)<0) robot_quat = -robot_quat;

  arma::vec vel_click = vel;
  vel_click.subvec(0,2) += vel_CLIK*(pos - robot_pos);
  vel_click.subvec(3,5) += rotVel_CLIK*quatLog(quatProd(quat, quatInv(robot_quat)));

  this->setTaskVelocity(vel_click);
}

void Robot::setJLAV(double gain, double jlim_safety_margin)
{
  jlim_safety_margin *= 3.14159/180;
  arma::vec q_min = this->getJointPosLowLim() + jlim_safety_margin;
  arma::vec q_max = this->getJointPosUpperLim() - jlim_safety_margin;
  jlav.reset(new robo_::PPCJointLimAvoid(q_min, q_max));
  jlav->setGains(gain);

  use_jlav = true;
}

Mode Robot::getMode() const
{
  return mode.get();
}

std::string Robot::getModeName() const
{
  return Robot::getModeName(getMode());
}

std::string Robot::getModeName(Mode mode)
{
  return Robot::mode_name[mode];
}

} // namespace rw_
