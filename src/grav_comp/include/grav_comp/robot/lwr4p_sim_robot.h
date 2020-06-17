#ifndef $_PROJECT_384$_LWR4P_SIM_ROBOT_H
#define $_PROJECT_384$_LWR4P_SIM_ROBOT_H

#include <grav_comp/robot/robot.h>
#include <lwr4p/lwr4p_sim_robot.h>
#include <robo_lib/joint_state_publisher.h>

class LWR4p_Sim_Robot: public Robot
{
public:
  LWR4p_Sim_Robot();
  ~LWR4p_Sim_Robot();

  void commandThread();

  void setWrenchBias() { }

  int getNumOfJoints() const
  { return N_JOINTS; }

  arma::vec getJointPosLowLim() const { return jpos_low_lim; }
  arma::vec getJointPosUpperLim() const { return jpos_upper_lim; }

  std::string getErrMsg() const
  { return err_msg; }

  arma::vec getTaskPosition() const
  { return robot->getTaskPosition(); }

  arma::mat getTaskRotMat() const
  { return robot->getTaskOrientation() * R_et; }

  arma::vec getTaskOrientation() const
  { return rotm2quat(this->getTaskRotMat()); }

  arma::vec getTaskForce() const
  { return (robot->getExternalWrench()).subvec(0,2); }

  arma::vec getTaskTorque() const
  { return (robot->getExternalWrench()).subvec(3,5); }

  arma::vec getTaskWrench() const
  { return robot->getExternalWrench(); }

  arma::vec getCompTaskWrench() const
  {
    arma::vec wrench(6);
    Eigen::Map<Eigen::Matrix<double,6,1>> wrench_map(wrench.memptr());
    arma::vec quat(4);

    wrench = this->getTaskWrench();
    quat = this->getTaskOrientation();
    Eigen::Vector6d tool_wrench = tool_estimator->getToolWrench(Eigen::Quaterniond(quat(0),quat(1),quat(2),quat(3)));
    wrench_map -= tool_wrench;

    return wrench;
  }

  arma::vec getEstimatedTaskWrench() const { return robot->getExternalWrench(); }

  arma::vec getJointsPosition() const
  { return robot->getJointPosition(); }

  arma::mat getJacobian() const
  { return robot->getRobotJacobian(); }

  void update()
  { if (isOk()) KRC_tick.wait(); }

  arma::vec getJointsLowerLimits() const
  { return jpos_low_lim; }

  arma::vec getJointsUpperLimits() const
  { return jpos_upper_lim; }

  void stop();

  void setMode(const Robot::Mode &mode);

  double getCtrlCycle() const
  { return robot->getControlCycle(); }

  bool isOk() const
  { return robot->isOk(); }

  void setJointsPosition(const arma::vec &jpos) { jpos_cmd.set(jpos); }
  void setJointsTorque(const arma::vec &jtorq) { jtorque_cmd.set(jtorq); }
  void setTaskVelocity(const arma::vec &vel) { cart_vel_cmd.set(vel); }
  bool setJointsTrajectory(const arma::vec &qT, double duration);

  std::vector<std::string> getJointNames() const
  { return jnames; }

private:

  arma::mat get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime) const;

  std::string err_msg;

  int N_JOINTS;

  MtxVar<Mode> cmd_mode;

  Semaphore mode_change;

  arma::vec jpos_low_lim;
  arma::vec jpos_upper_lim;
  std::vector<std::string> jnames;

  std::shared_ptr<lwr4p::LWR4pSimRobot> robot;
  as64_::robo_::JointStatePublisher jState_pub;


};

#endif // $_PROJECT_384$_LWR4p_Sim_Robot_H
