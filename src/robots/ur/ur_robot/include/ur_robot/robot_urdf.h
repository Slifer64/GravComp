#ifndef UR_ROBOT_URDF_ARM_H
#define UR_ROBOT_URDF_ARM_H

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

namespace ur_
{

class RobotUrdf
{
public:
  RobotUrdf(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link);
  RobotUrdf(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link);
  ~RobotUrdf();

  int getNumJoints() const;

  // return using the urdf model and kdl forward-inverse kinematics solvers
  arma::vec getJointsPosition(const arma::mat &pose, const arma::vec &q0, bool *found_solution=NULL) const;
  arma::mat getTaskPose(const arma::vec &j_pos) const;
  arma::mat getTaskPosition(const arma::vec &j_pos) const;
  arma::mat getTaskRotm(const arma::vec &j_pos) const;
  arma::mat getTaskQuat(const arma::vec &j_pos) const;
  arma::mat getJacobian(const arma::vec &j_pos) const;
  arma::mat getEEJacobian(const arma::vec &j_pos) const;

  std::string getJointName(int i) const { return joint_names[i]; }
  double getJointPosLowLim(int i) const { return joint_pos_lower_lim[i]; }
  double getJointPosUpperLim(int i) const { return joint_pos_upper_lim[i]; }
  double getJointVelLim(int i) const { return joint_vel_lim[i]; }
  double getJointEffortLim(int i) const { return effort_lim[i]; }

  std::vector<std::string> getJointsName() const { return joint_names; }
  std::vector<double> getJointsPosLowLim() const { return joint_pos_lower_lim; }
  std::vector<double> getJointsPosUpperLim() const { return joint_pos_upper_lim; }
  std::vector<double> getJointsVelLim() const { return joint_vel_lim; }
  std::vector<double> getJointsEffortLim() const { return effort_lim; }

private:

  void init();

  std::vector<std::string> joint_names;
  std::vector<double> joint_pos_lower_lim;
  std::vector<double> joint_pos_upper_lim;
  std::vector<double> joint_vel_lim;
  std::vector<double> effort_lim;

  urdf::Model urdf_model;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver;//Inverse velocity solver
  std::shared_ptr<KDL::ChainIkSolverPos_NR> ik_solver;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver;
  KDL::Chain chain;

  std::string base_link_name;
  std::string tool_link_name;
};

}; // namespace ur_

#endif // UR_ROBOT_URDF_ARM_H
