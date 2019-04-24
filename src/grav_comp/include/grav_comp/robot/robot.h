#ifndef GRAVITY_COMPENSATION_ROBOT_H
#define GRAVITY_COMPENSATION_ROBOT_H

#include <cstdlib>
#include <exception>
#include <vector>
#include <cstring>
#include <thread>
#include <armadillo>
#include <Eigen/Dense>

#include <grav_comp/utils.h>

#include <grav_comp/tool_estimator.h>

class Robot
{
public:
  /** The control modes that can be applied to the robot. */
  enum Mode
  {
    JOINT_POS_CONTROL, // joint torque control
    JOINT_TORQUE_CONTROL, // joint torque control
    CART_VEL_CTRL, // Cartesian velocity control
    FREEDRIVE, // freedrive mode (or gravity compensation)
    IDLE, // robot is idle and doesn't move
    STOPPED, // the robot stops
  };

  Robot(const ToolEstimator *tool_est);
  ~Robot();

  Robot::Mode getMode() const;
  std::string getModeName() const;

  virtual std::string getErrMsg() const = 0;

  virtual int getNumOfJoints() const = 0;

  virtual arma::vec getTaskPosition() const = 0;
  virtual arma::vec getTaskOrientation() const = 0;
  virtual arma::vec getTaskForce() const = 0;
  virtual arma::vec getTaskTorque() const = 0;
  virtual arma::vec getTaskWrench() const = 0;

  virtual arma::vec getJointsPosition() const = 0;
  virtual arma::mat getJacobian() const = 0;

  /** Updates the robot state (position, forces, velocities etc.) by reading them
   *  from the actual hardware. Must be called once in each control cycle.
   */
  virtual void update() = 0;

  virtual void stop() = 0;

  virtual void setMode(const Robot::Mode &mode) = 0;

  virtual double getCtrlCycle() const = 0;
  virtual bool isOk() const = 0;

  virtual void commandThread() = 0;

  virtual void setJointsPosition(const arma::vec &jpos) = 0;
  virtual void setJointsTorque(const arma::vec &jtorq) = 0;
  virtual void setTaskVelocity(const arma::vec &vel) = 0;

  virtual bool setJointsTrajectory(const arma::vec &qT, double duration) = 0;

  virtual arma::vec getJointsLowerLimits() const = 0;
  virtual arma::vec getJointsUpperLimits() const = 0;

  virtual std::vector<std::string> getJointNames() const = 0;

  virtual void setExternalStop(bool set) = 0;

  virtual arma::vec getCompTaskWrench() const = 0;

  static arma::mat get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime);
  
protected:

  Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm) const;
  arma::vec rotm2quat(const arma::mat &rotm) const ;

  MtxVar<Mode> mode; // current mode
  std::vector<std::string> mode_name; ///< robot's control mode name

  MtxVar<arma::vec> jpos_cmd;
  MtxVar<arma::vec> cart_vel_cmd;
  MtxVar<arma::vec> jtorque_cmd;

  Semaphore KRC_tick;

  const ToolEstimator *tool_estimator;
};

#endif // GRAVITY_COMPENSATION_ROBOT_H