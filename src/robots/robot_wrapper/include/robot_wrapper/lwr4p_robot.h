#ifndef AS64_ROBOT_WRAPPER_LWR4P_ROBOT_H
#define AS64_ROBOT_WRAPPER_LWR4P_ROBOT_H

#include <robot_wrapper/robot.h>
#include <lwr4p/robot.h>
#include <lwr4p/sim_robot.h>

using namespace as64_;

namespace rw_
{

class LWR4p_Robot: public Robot
{
public:
  LWR4p_Robot(bool use_sim);
  ~LWR4p_Robot();

  void commandThread() override;

  arma::vec getTaskPosition() const override
  { return robot->getTaskPosition(); }

  arma::mat getTaskRotMat() const override
  { return robot->getTaskRotm(); }

  arma::vec getTaskOrientation() const override
  { return rotm2quat(this->getTaskRotMat()); }

  arma::vec getJointsPosition() const override
  { return robot->getJointsPosition(); }

  arma::mat getJacobian() const override
  { return robot->getJacobian(); }

  void update() override
  { KRC_tick.wait(); }

  arma::vec getJointPosLowLim() const override
  { return arma::vec(robot->robot_urdf->getJointsPosLowLim())*180/3.14159; }

  arma::vec getJointPosUpperLim() const override
  { return arma::vec(robot->robot_urdf->getJointsPosUpperLim())*180/3.14159; }

  void stop() override;

  void setMode(const Mode &mode) override;

  bool isOk() const override
  { return robot->isOk() && !external_stop_; }

  void setJointsPosition(const arma::vec &jpos) override { jpos_cmd.set(jpos); }
  void setJointsTorque(const arma::vec &jtorq) override { jtorque_cmd.set(jtorq); }
  void setTaskVelocity(const arma::vec &vel) override { cart_vel_cmd.set(vel); }
  bool setJointsTrajectory(const arma::vec &qT, double duration) override;

  std::vector<std::string> getJointNames() const
  { return robot->robot_urdf->getJointsName(); }

  void biasRobotSensor() override
  { PRINT_WARNING_MSG("[LWR4p_Robot::biasRobotSensor]: ** NULL **\n"); }

private:

  void setRobotIdle() override { robot->setMode(lwr4p_::Mode::JOINT_POS_CONTROL); }

  arma::vec getTaskWrenchFromRobot() const override
  { return -(robot->getExternalWrench()); }

  std::shared_ptr<lwr4p_::RobotArm> robot;

  thr_::Semaphore mode_change;

  bool run_;
};

} // namespace rw_

#endif // AS64_ROBOT_WRAPPER_LWR4P_ROBOT_H
