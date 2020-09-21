#ifndef UR_ROBOT_ARM_H
#define UR_ROBOT_ARM_H

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <memory>
#include <functional>
#include <map>
#include <string>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>

#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <armadillo>
#include <ur_robot/utils.h>
#include <ur_robot/robot_urdf.h>
#include <ur_modern_driver/utils.h>

namespace as64_
{

namespace ur_
{

class RobotArm
{
public:
  RobotArm(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link);
  RobotArm(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link);
  ~RobotArm();

  virtual bool isOk() const = 0;
  virtual void enable() = 0;
  virtual void setFreedriveMode() = 0;
  virtual void setNormalMode() = 0;
  virtual void update() = 0;
  virtual void setJointsPosition(const arma::vec &j_pos) = 0;
  virtual void setJointsVelocity(const arma::vec &j_vel) = 0;
  virtual void setTaskVelocity(const arma::vec &task_vel) = 0;
  virtual void setTaskPose(const arma::mat &task_pose) = 0;

  virtual arma::vec getJointsPosition() const = 0;
  virtual arma::vec getJointsVelocity() const = 0;
  virtual arma::mat getTaskPose() const = 0;
  virtual arma::vec getTaskPosition() const = 0;
  virtual arma::mat getTaskRotm() const = 0;
  virtual arma::vec getTaskQuat() const = 0;
  virtual arma::vec getTaskVelocity() const = 0;
  virtual arma::mat getJacobian() const = 0;
  virtual arma::mat getEEJacobian() const = 0;
  virtual arma::vec getTcpWrench() const = 0;

  double getCtrlCycle() const;
  int getNumJoints() const;
  std::string getErrMsg() const;
  ur_::Mode getMode() const;
  bool setJointsTrajectory(const arma::vec &j_targ, double duration);
  bool setTaskTrajectory(const arma::mat &target_pose, double duration);

  void setJointLimitCheck(bool check);
  void setSingularityCheck(bool check);
  void setSingularityThreshold(double thres);

  void readWrenchFromTopic(const std::string &topic);
  void setGetExternalWrenchFun(arma::vec (*getWrenchFun)(void));
  void setGetExternalWrenchFun(std::function<arma::vec()> get_wrench_fun);
  template<class T>
  void setGetExternalWrenchFun(arma::vec (T::*getWrenchFun)(void), T *obj_ptr)
  {
    get_wrench_fun_ptr = std::bind(getWrenchFun, obj_ptr);
  }

  void addJointState(sensor_msgs::JointState &joint_state_msg);

  arma::vec getLowerJointLimits() const { return arma::vec(robot_urdf->getJointsPosLowLim()); }
  arma::vec getUpperJointLimits() const { return arma::vec(robot_urdf->getJointsPosUpperLim()); }

  std::shared_ptr<RobotUrdf> robot_urdf;

protected:

  virtual void protectiveStop() = 0;

  void init();

  Mode mode;
  std::map<ur_::Mode, std::string> mode_name;

  int N_JOINTS;

  double SINGULARITY_THRES;

  mutable std::mutex robot_state_mtx;

  urdf::Model urdf_model;

  ros::NodeHandle node;

  std::string base_link_name;
  std::string tool_link_name;

  double ctrl_cycle;

  arma::vec Fext;

  bool check_limits;
  bool check_singularity;

  std::shared_ptr<WrenchReader> wrench_reader;

  std::string err_msg;
  bool checkLimits(const arma::vec &j_pos);
  bool checkJointPosLimits(const arma::vec &j_pos);
  bool checkJointVelLimits(const arma::vec &dj_pos);
  bool checkSingularity(const arma::vec &j_pos);

  std::string getModeName(Mode mode) const;

  std::function<arma::vec(void)> get_wrench_fun_ptr;
};

}; // namespace ur_

}; // namespace as64_

#endif // UR_ROBOT_ARM_H
