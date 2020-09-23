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

#include <ati_sensor/ft_sensor.h>

#include <robo_lib/tool_estimator.h>
#include <robo_lib/ppc_joint_limit_avoid.h>
#include <robo_lib/singular_value_filter.h>
#include <robo_lib/joint_state_publisher.h>
#include <thread_lib/thread_lib.h>

#include <robot_wrapper/utils/print_utils.h>
#include <robot_wrapper/utils/math_utils.h>

using namespace as64_;

namespace rw_
{

enum Mode
{
  JOINT_POS_CONTROL = 0, // joint position control
  JOINT_TORQUE_CONTROL, // joint velocity control
  CART_VEL_CTRL, // Cartesian velocity control
  FREEDRIVE, // freedrive mode (or gravity compensation)
  IDLE, // robot is idle and doesn't move
  STOPPED, // the robot stops
  PROTECTIVE_STOP,
};

class Robot
{
public:
  /** The control modes that can be applied to the robot. */
  Robot();
  ~Robot();

  Mode getMode() const;
  std::string getModeName() const;

  void setToolEstimator(const std::string &tool_massCoM_file);
  void setToolEstimator(const robo_::ToolEstimator &tool_est_);

  void setSVFilt(double sigma_min = 0.1, double shape_f = 19.9);
  void setJLAV(double gain = 1e-4, double jlim_safety_margin = 3);

  void useAtiSensor();
  void setWrenchBias() { if (ftsensor) ftsensor->setBias(); }

  double getGlobalTime() const { return global_time_sec; }

  bool emergencyStop() const { return emergency_stop; }

  virtual void setEmergencyStop(bool set) = 0;

  virtual std::string getErrMsg() const = 0;

  int getNumOfJoints() const { return N_JOINTS; }

  double getCtrlCycle() const { return Ts; }

  virtual arma::vec getJointPosLowLim() const = 0;
  virtual arma::vec getJointPosUpperLim() const = 0;

  virtual arma::vec getTaskPosition() const = 0;
  virtual arma::mat getTaskRotMat() const = 0;
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

  arma::vec getCompTaskWrench() const;

  void setEeToolRot(const arma::mat &R) { this->R_et = R; }

  void setWrenchDeadZone(const arma::vec &Fext_dead_zone) { this->Fext_dead_zone.set(Fext_dead_zone); }
  void setWrenchFilter(double af) { this->af.set(af); }

  void setVelCLICK(double vel_click, double rotVel_click);

  void publishJointStates(const std::string &publish_jstates_topic, double pub_rate=0.033);

protected:

  virtual void addJointState(sensor_msgs::JointState &joint_state_msg) = 0;

  std::function<arma::vec()> get_wrench_fun;
  arma::vec getTaskWrenchFromAti() const;
  virtual arma::vec getTaskWrenchFromRobot() const = 0;

  arma::vec applyFextDeadZone(const arma::vec &F_ext) const;

  thr_::MtxVar<arma::vec> Fext_dead_zone;
  thr_::MtxVar<double> af;

  thr_::MtxVar<Mode> mode; // current mode
  std::vector<std::string> mode_name; ///< robot's control mode name

  thr_::MtxVar<arma::vec> jpos_cmd;
  thr_::MtxVar<arma::vec> cart_vel_cmd;
  thr_::MtxVar<arma::vec> jtorque_cmd;

  arma::mat R_et;
  std::shared_ptr<robo_::ToolEstimator> tool_estimator;
  std::shared_ptr<robo_::SingularValueFilter> svf;
  std::shared_ptr<robo_::PPCJointLimAvoid> jlav;
  std::shared_ptr<ati::FTSensor> ftsensor;

  thr_::Semaphore KRC_tick;

  int N_JOINTS;
  double Ts; // robot control cycle

  bool use_svf;
  bool use_jlav;
  bool emergency_stop;

  robo_::JointStatePublisher jState_pub;

  double vel_CLIK;
  double rotVel_CLIK;

  double global_time_sec;

};

} // namespace rw_

#endif // AS64_ROBOT_WRAPPER_ROBOT_H
