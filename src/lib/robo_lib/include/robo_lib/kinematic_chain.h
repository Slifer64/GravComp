#ifndef ROBO_LIB_KINEMATIC_CHAIN_H
#define ROBO_LIB_KINEMATIC_CHAIN_H

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <memory>
#include <string>

#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <armadillo>

namespace as64_
{

namespace robo_
{

class KinematicChain
{
public:
  KinematicChain(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link);
  KinematicChain(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link);
  ~KinematicChain();

  int getNumJoints() const;

  // return using the urdf model and kdl forward-inverse kinematics solvers
  arma::vec getJointsPosition(const arma::mat &pose, const arma::vec &q0, bool *found_solution=NULL) const;
  arma::mat getTaskPose(const arma::vec &j_pos) const;
  arma::mat getJacobian(const arma::vec j_pos) const;

  std::vector<std::string> joint_names;
  std::vector<double> joint_pos_lower_lim;
  std::vector<double> joint_pos_upper_lim;
  std::vector<double> joint_vel_lim;
  std::vector<double> effort_lim;

private:

  void init();

  int N_JOINTS;

  urdf::Model urdf_model;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver;//Inverse velocity solver
  std::shared_ptr<KDL::ChainIkSolverPos_NR> ik_solver;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver;
  KDL::Chain chain;

  std::string base_link_name;
  std::string tool_link_name;
};

}; // namespace robo_

}; // namespace as64_

#endif // ROBO_LIB_KINEMATIC_CHAIN_H
