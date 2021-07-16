#ifndef AS64_ROBOT_WRAPPER_ROBOT_H
#define AS64_ROBOT_WRAPPER_ROBOT_H

#include <cstdlib>
#include <exception>
#include <vector>
#include <cstring>
#include <thread>
#include <armadillo>
#include <Eigen/Dense>

#include <sensor_msgs/JointState.h>

#include <robo_lib/ppc_joint_limit_avoid.h>
#include <robo_lib/singular_value_filter.h>
#include <robo_lib/joint_state_publisher.h>
#include <thread_lib/thread_lib.h>

#include <robot_wrapper/utils/print_utils.h>
#include <robot_wrapper/utils/math_utils.h>
#include <robot_wrapper/utils/admittance_controller.h>

#include <robot_wrapper/utils/wrench_interface.h>
#include <robot_wrapper/utils/inv_kinematics_interface.h>
#include <robot_wrapper/utils/publish_state_interface.h>

#include <robot_wrapper/utils/safety_limits.h>

using namespace as64_;

namespace rw_
{

class AdmittanceCtrl; // forward declaration

enum Mode
{
  JOINT_POS_CONTROL = 0, // joint position control
  JOINT_TORQUE_CONTROL, // joint torque control
  CART_VEL_CTRL, // Cartesian velocity control
  FREEDRIVE, // freedrive mode (or gravity compensation)
  ADMITTANCE, // admittance control
  IDLE, // robot is idle and doesn't move
  STOPPED, // the robot stops
  PROTECTIVE_STOP,
};



class Robot : public WrenchInterface, public InvKinematicsInterface, public PublishStateInterface
{
public:
  /** The control modes that can be applied to the robot. */
  Robot();
  ~Robot();

  Mode getMode() const;
  std::string getModeName() const;
  static std::string getModeName(Mode mode);

  void setJLAV(double gain = 1e-4, double jlim_safety_margin = 3);

  double getGlobalTime() const { return global_time_sec; }

  void setExternalStop(bool set);

  std::string getErrMsg() const { return err_msg; }

  int getNumOfJoints() const { return N_JOINTS; }

  double getCtrlCycle() const { return Ts; }

  virtual arma::vec getJointPosLowLim() const = 0;
  virtual arma::vec getJointPosUpperLim() const = 0;

  virtual arma::vec getTaskPosition() const = 0;
  virtual arma::mat getTaskRotMat() const = 0;
  virtual arma::vec getTaskOrientation() const = 0;

  virtual arma::vec getJointsPosition() const = 0;

  /** Updates the robot state (position, forces, velocities etc.) by reading them
   *  from the actual hardware. Must be called once in each control cycle.
   */
  virtual void update() = 0;

  virtual void stop() = 0;

  virtual void setMode(const Mode &mode) = 0;

  virtual bool isOk() const = 0;

  virtual void commandThread() = 0;

  virtual void setJointsPosition(const arma::vec &jpos) = 0;
  virtual void setJointsTorque(const arma::vec &jtorq) = 0;
  virtual void setTaskVelocity(const arma::vec &vel) = 0;

  // For setting task velocity with CLICK
  void setTaskVelocity(const arma::vec &vel, const arma::vec &pos, const arma::vec &quat);

  virtual bool setJointsTrajectory(const arma::vec &qT, double duration) = 0;

  virtual std::vector<std::string> getJointNames() const = 0;

  void setVelCLICK(double vel_click, double rotVel_click);

protected:

  virtual void setRobotIdle() = 0;

  std::string err_msg;
  void setErrMsg(const std::string &msg) { err_msg = msg; }

  thr_::MtxVar<Mode> mode; // current mode
  static std::vector<std::string> mode_name; ///< robot's control mode name

  thr_::MtxVar<arma::vec> jpos_cmd;
  thr_::MtxVar<arma::vec> cart_vel_cmd;
  thr_::MtxVar<arma::vec> jtorque_cmd;

  std::shared_ptr<robo_::PPCJointLimAvoid> jlav;

  std::unique_ptr<AdmittanceCtrl> adm_ctrl;

  thr_::Semaphore KRC_tick;

  int N_JOINTS;
  double Ts; // robot control cycle

  bool use_jlav;

  thr_::MtxVar<Mode> cmd_mode;

  double vel_CLIK;
  double rotVel_CLIK;

  double global_time_sec;

  bool external_stop_;

  SafetyLimits safety_;

};

} // namespace rw_

#endif // AS64_ROBOT_WRAPPER_ROBOT_H
