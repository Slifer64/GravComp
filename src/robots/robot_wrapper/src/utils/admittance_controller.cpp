#include <robot_wrapper/utils/admittance_controller.h>

#include <robot_wrapper/robot.h>
#include <robot_wrapper/utils/xml_parser.h>

#include <armadillo>

#include <ros/package.h>

namespace rw_
{

#define AdmCtrl_fun_ std::string("[AdmittanceCtrl::") + __func__ + "]: "

AdmittanceCtrl::AdmittanceCtrl(rw_::Robot *robot)
{
  robot_ = robot;
}

void AdmittanceCtrl::init()
{
  p = robot_->getTaskPosition();
  p_dot = p_ddot = arma::vec().zeros(3);

  Q = robot_->getTaskOrientation();
  omega = omega_dot = arma::vec().zeros(3);

  Fext_prev = arma::vec().zeros(6);

  std::string params_file = ros::package::getPath("robot_wrapper") + "/config/adm_params.yaml";
  loadParams(params_file);
}

void AdmittanceCtrl::stop()
{
  p_ddot = p_dot = omega_dot = omega = arma::vec().zeros(3);
  ret_vel = arma::vec().zeros(6);
}

void AdmittanceCtrl::loadParams(const std::string &filename)
{
  rw_::XmlParser parser(filename);

  if (!parser.getParam("Mp",Mp)) throw std::runtime_error(AdmCtrl_fun_ + "Failed to load param \"Mp\"...");
  if (!parser.getParam("Dp",Dp)) throw std::runtime_error(AdmCtrl_fun_ + "Failed to load param \"Dp\"...");
  if (!parser.getParam("Mo",Mo)) throw std::runtime_error(AdmCtrl_fun_ + "Failed to load param \"Mo\"...");
  if (!parser.getParam("Do",Do)) throw std::runtime_error(AdmCtrl_fun_ + "Failed to load param \"Do\"...");
  if (!parser.getParam("a_f",a_f)) throw std::runtime_error(AdmCtrl_fun_ + "Failed to load param \"a_f\"...");
}

void AdmittanceCtrl::update()
{
  arma::vec Fext = robot_->getTaskWrench();
  Fext = (1-a_f)*Fext_prev + a_f*Fext;
  Fext_prev = Fext;

  //std::cerr << "||F_ext|| = " << arma::norm(Fext) << "\n";

  p_ddot = (-Dp%p_dot + Fext.subvec(0,2)) / Mp;
  omega_dot = (-Do%omega + Fext.subvec(3,5)) / Mo;

  this->ret_vel = arma::join_vert(p_dot, omega);

  double dt = robot_->getCtrlCycle();
  p = p + p_dot*dt;
  p_dot = p_dot + p_ddot*dt;
  Q = quatProd(quatExp(omega*dt), Q);
  omega = omega + omega_dot*dt;

  // we could also use KLICK, but in admittance control its not really necessary
}


arma::vec AdmittanceCtrl::getVelocity() const
{
  return ret_vel;
}


} // namespace rw_
