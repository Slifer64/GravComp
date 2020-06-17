#ifndef LWR_LWRp_SIM_ROBOT_H
#define LWR_LWRp_SIM_ROBOT_H

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
#include <sensor_msgs/JointState.h>

#include <armadillo>

#include <lwr4p/robot.h>

namespace lwr4p
{

class LWR4pSimRobot : public Robot
{
public:
  LWR4pSimRobot(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link, double ctrl_cycle);
  LWR4pSimRobot(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link, double ctrl_cycle);

  ~LWR4pSimRobot();
  void stop();
  bool isOk() const;
  void setMode(lwr4p::Mode mode);
  void waitNextCycle();
  double getControlCycle() const;

  arma::vec getJointPosition() const;

  arma::vec getJointTorque() const;

  arma::vec getJointExternalTorque() const;

  arma::mat getEEJacobian() const;

  arma::mat getRobotJacobian() const;

  arma::mat getTaskPose() const;

  arma::vec getTaskPosition() const;

  arma::mat getTaskOrientation() const;

  arma::vec getExternalWrench() const;

  void setJointPosition(const arma::vec &jpos);

  void setJointVelocity(const arma::vec &jvel);

  void setJointTorque(const arma::vec &jtorq);

  void setTaskPose(const arma::mat &task_pose);

  void setWrench(const arma::vec &wrench);

  void setJointTrajectory(const arma::vec &jtarget, double duration);

  void setCartStiffness(const arma::vec &cart_stiff);

  void setCartDamping(const arma::vec &cart_damp);

  void addJointState(sensor_msgs::JointState &joint_state_msg);

private:
  void init();

  void setSingularityThreshold(double thres);
  void setJointLimitCheck(bool check);
  void setSingularityCheck(bool check);

  void stopController();

  std::string err_msg;
  bool checkJointPosLimits(const arma::vec &j_pos);
  bool checkJointVelLimits(const arma::vec &dj_pos);
  bool checkSingularity();

  void setJointPositionHelper(const arma::vec &j_pos);
  void setJointVelocityHelper(const arma::vec &j_vel);
  void setTaskVelocityHelper(const arma::vec &task_vel);

  arma::mat getRobotJacobian(const arma::vec &j_pos) const;
  arma::mat getTaskPose(const arma::vec &j_pos) const;
  arma::vec getJointPosition(const arma::mat &pose, const arma::vec &q0, bool *found_solution) const;

  int N_JOINTS;

  double SINGULARITY_THRES;

  std::mutex robot_state_mtx;

  urdf::Model urdf_model;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver;//Inverse velocity solver
  std::shared_ptr<KDL::ChainIkSolverPos_NR> ik_solver;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver;
  KDL::Chain chain;

  ros::NodeHandle node;

  std::vector<std::string> joint_names;
  std::vector<double> joint_pos_lower_lim;
  std::vector<double> joint_pos_upper_lim;
  std::vector<double> joint_vel_lim;
  std::vector<double> effort_lim;

  std::string base_link_name;
  std::string tool_link_name;

  double ctrl_cycle;
  arma::vec prev_joint_pos;
  arma::vec joint_pos;
  arma::vec Fext;

  lwr4p::Timer timer;
  unsigned long update_time;

  bool check_limits;
  bool check_singularity;

};


}  // namespace lwr4p

#endif  // LWR_LWRp_SIM_ROBOT_H
