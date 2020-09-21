#ifndef $_PROJECT_384$_LWR4P_ROBOT_H
#define $_PROJECT_384$_LWR4P_ROBOT_H

#include <grav_comp/robot/robot.h>
#include <lwr4p/robot.h>
#include <lwr4p/sim_robot.h>
#include <robo_lib/joint_state_publisher.h>

using namespace as64_;

class LWR4p_Robot: public Robot
{
public:
  LWR4p_Robot(bool use_sim);
  ~LWR4p_Robot();

  void commandThread() override;

  void setWrenchBias() override { if (ftsensor) ftsensor->setBias(); }

  int getNumOfJoints() const override
  { return robot->getNumJoints(); }

  std::string getErrMsg() const override
  { return err_msg; }

  void publishJointStates(const std::string &publish_jstates_topic) override;

  void useAtiSensor() override;

  arma::vec getTaskPosition() const override
  { return robot->getTaskPosition(); }

  arma::mat getTaskRotMat() const override
  { return robot->getTaskRotm() * R_et; }

  arma::vec getTaskOrientation() const override
  { return rotm2quat(this->getTaskRotMat()); }

  arma::vec getTaskForce() const override
  { return getTaskWrench().subvec(0,2); }

  arma::vec getTaskTorque() const override
  { return getTaskWrench().subvec(3,5); }

  arma::vec getTaskWrench() const override
  { return applyFextDeadZone(get_wrench_fun()); }

  arma::vec getJointsPosition() const override
  { return robot->getJointsPosition(); }

  arma::mat getJacobian() const override
  { return robot->getJacobian(); }

  void update() override
  { KRC_tick.wait(); }

  void setEmergencyStop(bool set) override
  {
    emergency_stop = set;
    if (set) setMode(Robot::IDLE);
  }

  arma::vec getJointPosLowLim() const override
  { return arma::vec(robot->robot_urdf->getJointsPosLowLim())*180/3.14159; }

  arma::vec getJointPosUpperLim() const override
  { return arma::vec(robot->robot_urdf->getJointsPosUpperLim())*180/3.14159; }

  void stop() override;

  void setMode(const Robot::Mode &mode) override;

  double getCtrlCycle() const override
  { return Ts; }

  bool isOk() const override
  { return robot->isOk(); }

  void setJointsPosition(const arma::vec &jpos) override { jpos_cmd.set(jpos); }
  void setJointsTorque(const arma::vec &jtorq) override { jtorque_cmd.set(jtorq); }
  void setTaskVelocity(const arma::vec &vel) override { cart_vel_cmd.set(vel); }
  bool setJointsTrajectory(const arma::vec &qT, double duration) override;

  std::vector<std::string> getJointNames() const
  { return robot->robot_urdf->getJointsName(); }

private:

  arma::vec getTaskWrenchFromAti() const;
  arma::vec getTaskWrenchFromRobot() const;

  std::shared_ptr<lwr4p_::RobotArm> robot;
  robo_::JointStatePublisher jState_pub;
  std::shared_ptr<ati::FTSensor> ftsensor;
  std::function<arma::vec()> get_wrench_fun;

  std::string err_msg;
  int N_JOINTS;
  thr_::MtxVar<Mode> cmd_mode;
  thr_::Semaphore mode_change;

  double Ts; // robot control cycle
};

#endif // $_PROJECT_384$_LWR4P_ROBOT_H
