/**
 * Copyright (C) 2016 AUTH-ARL
 */

#ifndef LWR_ABSTRACT_ROBOT_H
#define LWR_ABSTRACT_ROBOT_H

#include <ros/ros.h>

#include <memory>
#include <vector>
#include <string>
#include <map>

#include <armadillo>

#include <lwr4p/utils.h>

namespace lwr4p
{

enum Mode {UNDEFINED = -1000, /**< For internal use */
           STOPPED = -1, /**< When the robot is stopped and does not accept commands */
           JOINT_POS_CTRL  = 0, /**< For sending joint position commands */
           JOINT_VEL_CTRL  = 1, /**< For sending joint velocity commands */
           JOINT_TORQUE_CTRL    = 2, /**< For sending torque commands */
           IMPEDANCE_CONTROL = 3, /**< For operating in Impedance control */
          };

class Robot
{
public:
  Robot();
  ~Robot();

  virtual void stop() = 0;
  virtual bool isOk() const = 0;
  virtual void setMode(lwr4p::Mode mode) = 0;
  virtual void waitNextCycle() = 0;
  virtual double getControlCycle() const = 0;

  Mode getMode() const { return mode; }
  std::string getModeName() const { return mode_name.find(mode)->second; }
  std::string getModeName(lwr4p::Mode mode) const { return mode_name.find(mode)->second; }

  virtual arma::vec getJointPosition() const = 0;

  virtual arma::vec getJointTorque() const = 0;

  virtual arma::vec getJointExternalTorque() const = 0;

  virtual arma::mat getEEJacobian() const = 0;

  virtual arma::mat getRobotJacobian() const = 0;

  virtual arma::mat getTaskPose() const = 0;

  virtual arma::vec getTaskPosition() const = 0;

  virtual arma::mat getTaskOrientation() const = 0;

  virtual arma::vec getExternalWrench() const = 0;

  virtual void setJointPosition(const arma::vec &jpos) = 0;

  virtual void setJointVelocity(const arma::vec &jvel) = 0;

  virtual void setJointTorque(const arma::vec &jtorq) = 0;

  virtual void setTaskPose(const arma::mat &task_pose) = 0;

  virtual void setWrench(const arma::vec &wrench) = 0;

  virtual void setJointTrajectory(const arma::vec &jtarget, double duration) = 0;

  virtual void setCartStiffness(const arma::vec &cart_stiff) = 0;

  virtual void setCartDamping(const arma::vec &cart_damp) = 0;

protected:

  Mode mode;
  std::map<lwr4p::Mode, std::string> mode_name;
  double cycle;
  const int N_JOINTS;

  static arma::mat get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime);

};


}  // namespace lwr4p

#endif  // LWR_ABSTRACT_ROBOT_H
