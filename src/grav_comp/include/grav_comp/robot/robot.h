#ifndef $_PROJECT_384$_ROBOT_H
#define $_PROJECT_384$_ROBOT_H

#include <cstdlib>
#include <exception>
#include <vector>
#include <cstring>
#include <thread>
#include <armadillo>
#include <Eigen/Dense>

#include <grav_comp/utils.h>

#include <robo_lib/ppc_joint_limit_avoid.h>
#include <robo_lib/singular_value_filter.h>
#include <robo_lib/tool_estimator.h>

using namespace as64_;

class Robot
{
public:
  /** The control modes that can be applied to the robot. */
  enum Mode
  {
    JOINT_POS_CONTROL, // joint position control
    JOINT_TORQUE_CONTROL, // joint velocity control
    CART_VEL_CTRL, // Cartesian velocity control
    FREEDRIVE, // freedrive mode (or gravity compensation)
    IDLE, // robot is idle and doesn't move
    STOPPED, // the robot stops
  };

  Robot();
  ~Robot();

  Robot::Mode getMode() const;
  std::string getModeName() const;

  void setToolEstimator(const std::string &tool_massCoM_file);
  void setSVFilt(double sigma_min = 0.1, double shape_f = 19.9);
  void setJLAV(double gain = 1e-4, double jlim_safety_margin = 3);

  void setEmergencyStop(bool set) { emergency_stop = set; }
  bool emergencyStop() const { return emergency_stop; }

  virtual std::string getErrMsg() const = 0;

  virtual int getNumOfJoints() const = 0;
  virtual arma::vec getJointPosLowLim() const = 0;
  virtual arma::vec getJointPosUpperLim() const = 0;

  virtual arma::vec getTaskPosition() const = 0;
  virtual arma::mat getTaskRotMat() const = 0;
  virtual arma::vec getTaskOrientation() const = 0;
  virtual arma::vec getTaskForce() const = 0;
  virtual arma::vec getTaskTorque() const = 0;
  virtual arma::vec getTaskWrench() const = 0;

  virtual arma::vec getEstimatedTaskWrench() const = 0;

  virtual arma::vec getJointsPosition() const = 0;
  virtual arma::mat getJacobian() const = 0;

  virtual void setWrenchBias() = 0;

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

  virtual arma::vec getCompTaskWrench() const = 0;

  static arma::mat get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime);
  static arma::vec quatProd(const arma::vec &quat1, const arma::vec &quat2);
  static arma::vec quatExp(const arma::vec &v_rot, double zero_tol=1e-16);
  static arma::vec quatLog(const arma::vec &quat, double zero_tol=1e-16);
  static arma::vec quatInv(const arma::vec &quat);

  void setEeToolRot(const arma::mat &R) { this->R_et = R; }

  std::shared_ptr<robo_::ToolEstimator> tool_estimator;

protected:

  Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm) const;
  arma::vec rotm2quat(const arma::mat &rotm) const ;

  MtxVar<Mode> mode; // current mode
  std::vector<std::string> mode_name; ///< robot's control mode name

  MtxVar<arma::vec> jpos_cmd;
  MtxVar<arma::vec> cart_vel_cmd;
  MtxVar<arma::vec> jtorque_cmd;

  arma::mat R_et;
  std::shared_ptr<robo_::SingularValueFilter> svf;
  std::shared_ptr<robo_::PPCJointLimAvoid> jlav;

  Semaphore KRC_tick;

  bool use_svf;
  bool use_jlav;
  bool emergency_stop;

};

#endif // $_PROJECT_384$_ROBOT_H
