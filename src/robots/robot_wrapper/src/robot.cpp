#include <robot_wrapper/robot.h>

#include <ros/package.h>

namespace rw_
{

#define Robot_fun_ std::string("[Ur_Robot::") + __func__ + "]: "

Robot::Robot()
{
  vel_CLIK = 2;
  rotVel_CLIK = 0.8;

  mode_name.resize(6);
  mode_name[0] = "JOINT_POS_CONTROL";
  mode_name[1] = "JOINT_TORQUE_CONTROL";
  mode_name[2] = "CART_VEL_CTRL";
  mode_name[3] = "FREEDRIVE";
  mode_name[4] = "IDLE";
  mode_name[5] = "STOPPED";

  R_et = arma::mat().eye(3,3);

  tool_estimator.reset(new robo_::ToolEstimator);

  use_svf = false;
  use_jlav = false;
  emergency_stop = false;

  Fext_dead_zone = arma::vec().zeros(6);
  af = 0;

  global_time_sec = 0;
}

Robot::~Robot()
{

}

void Robot::setVelCLICK(double vel_click, double rotVel_click)
{
  vel_CLIK = vel_click;
  rotVel_CLIK = rotVel_click;
}

arma::vec Robot::getCompTaskWrench() const
{
  arma::mat R = this->getTaskRotMat();
  arma::vec tool_wrench = tool_estimator->getToolWrench(R);
  tool_wrench.subvec(0,2) = R*tool_wrench.subvec(0,2);
  tool_wrench.subvec(3,5) = R*tool_wrench.subvec(3,5);

  return applyFextDeadZone( getTaskWrench() - tool_wrench );
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

arma::vec Robot::applyFextDeadZone(const arma::vec &F_ext) const
{
  arma::vec sign_Fext = arma::sign(F_ext);
  arma::vec Fext2 = F_ext - sign_Fext%Fext_dead_zone();
  return 0.5*(arma::sign(Fext2)+sign_Fext)%arma::abs(Fext2);
}

void Robot::setToolEstimator(const std::string &tool_massCoM_file)
{
  tool_estimator.reset(new robo_::ToolEstimator);
  tool_estimator->initFromFile(tool_massCoM_file);
}

void Robot::setToolEstimator(const robo_::ToolEstimator &tool_est_)
{
  this->tool_estimator->setMass(tool_est_.getMass());
  this->tool_estimator->setCoM(tool_est_.getCoM());
}

void Robot::setSVFilt(double sigma_min, double shape_f)
{
  svf.reset(new robo_::SingularValueFilter(sigma_min, shape_f));

  use_svf = true;
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

void Robot::useAtiSensor()
{
  std::string ati_ip;
  if (!ros::NodeHandle("~").getParam("ati_ip", ati_ip)) throw std::runtime_error(Robot_fun_ + "Failed to read param \"ati_ip\"...");

  ftsensor.reset(new ati::FTSensor);
  std::cerr << "=======> Initializing F/T sensor at ip: " << ati_ip << "\n";
  ftsensor->init(ati_ip.c_str());
  ftsensor->setTimeout(1.0);
  // ftsensor->setBias();
  std::cerr << "=======> F/T sensor initialized successfully!\n";

  get_wrench_fun = std::bind(&Robot::getTaskWrenchFromAti, this);
}

arma::vec Robot::getTaskWrenchFromAti() const
{
  if (!ftsensor) throw std::runtime_error(Robot_fun_ + "Ati sensor is not initialized...\n");

  static double measurements[6];
  uint32_t rdt(0),ft(0);
  (const_cast<ati::FTSensor *>(ftsensor.get()))->getMeasurements(measurements,rdt,ft);
  //ftsensor->getMeasurements(measurements,rdt,ft);

  arma::vec Fext(6);
  Fext(0) = measurements[0];
  Fext(1) = measurements[1];
  Fext(2) = measurements[2];
  Fext(3) = measurements[3];
  Fext(4) = measurements[4];
  Fext(5) = measurements[5];

  arma::mat R = this->getTaskRotMat();
  Fext.subvec(0,2) = R*Fext.subvec(0,2);
  Fext.subvec(3,5) = R*Fext.subvec(3,5);
  return Fext;
}

void Robot::publishJointStates(const std::string &publish_jstates_topic, double pub_rate)
{
  jState_pub.setPublishTopic(publish_jstates_topic);
  jState_pub.addFun(&Robot::addJointState, this);
  jState_pub.setPublishCycle(pub_rate);
  jState_pub.start(); // launches joint states publisher thread
}

Mode Robot::getMode() const
{
  return mode.get();
}

std::string Robot::getModeName() const
{
  return mode_name[getMode()];
}

} // namespace rw_
