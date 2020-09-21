#include <robo_lib/kinematic_chain.h>

#include <stdexcept>
#include <sstream>
#include <stack>

#include <kdl_parser/kdl_parser.hpp>

namespace as64_
{

namespace robo_
{

KinematicChain::KinematicChain(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link)
{
  this->urdf_model = urdf_model;
  this->base_link_name = base_link;
  this->tool_link_name = tool_link;

  init();
}

KinematicChain::KinematicChain(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link)
{
  if (!urdf_model.initParam(robot_desc_param.c_str()))
  {
    throw std::ios_base::failure("Couldn't load urdf model from \"" + robot_desc_param + "\"...\n");
  }

  this->base_link_name = base_link;
  this->tool_link_name = tool_link;

  init();
}

KinematicChain::~KinematicChain() {}

void KinematicChain::init()
{
  // find base_link and tool_link
  bool found_base_link = false;
  bool found_tool_link = false;
  boost::shared_ptr<const urdf::Link> base_link;
  boost::shared_ptr<const urdf::Link> tool_link;
  std::stack<boost::shared_ptr<const urdf::Link>> link_stack;
  link_stack.push(urdf_model.getRoot());
  while (!link_stack.empty())
  {
    auto link = link_stack.top();
    link_stack.pop();

    if (base_link_name.compare(link->name) == 0)
    {
      base_link = link;
      found_base_link = true;
    }

    if (tool_link_name.compare(link->name) == 0)
    {
      tool_link = link;
      found_tool_link = true;
    }

    for (int i=0;i<link->child_links.size();i++) link_stack.push(link->child_links[i]);
    // for (int i=0;i<link->child_joints.size();i++) _joints.push_back(link->child_joints[i]);
  }

  if (!found_base_link)
    throw std::runtime_error("Couldn't find specified base link \"" + base_link_name + "\" in the robot urdf model...\n");

  if (!found_tool_link)
    throw std::runtime_error("Couldn't find specified tool link \"" + tool_link_name + "\" in the robot urdf model...\n");

  // find all links in the chain from tool_link to base_link
  std::vector<boost::shared_ptr<const urdf::Link>> chain_links;
  auto link = tool_link;
  while (link->name.compare(base_link->name))
  {
    chain_links.push_back(link);
    link = link->getParent();
  }
  chain_links.push_back(base_link);

  // parse all joints for each link starting from base_link
  for (int i=chain_links.size()-1; i>0; i--)
  {
    link = chain_links[i];
    auto next_link = chain_links[i-1];

    for (int i=0;i<link->child_joints.size();i++)
    {
      auto joint = link->child_joints[i];
      auto jtype = joint->type;

      if (jtype==urdf::Joint::FIXED || jtype==urdf::Joint::FLOATING) continue;

      if (joint->mimic) continue;

      if (joint->child_link_name.compare(next_link->name)) continue;

      joint_names.push_back(joint->name);

      if (jtype==urdf::Joint::CONTINUOUS)
      {
        joint_pos_lower_lim.push_back(-M_PI);
        joint_pos_upper_lim.push_back(M_PI);
      }
      else
      {
        joint_pos_lower_lim.push_back(joint->limits->lower);
        joint_pos_upper_lim.push_back(joint->limits->upper);
      }

      effort_lim.push_back(joint->limits->effort);
      joint_vel_lim.push_back(joint->limits->velocity);
    }
  }

  // create KDL::Chain and forward/inverse kinematics and Jacobian solvers
  KDL::Tree tree;
  kdl_parser::treeFromUrdfModel(urdf_model, tree);

  if (!tree.getChain(base_link_name, tool_link_name, chain))
  {
    throw std::runtime_error("Failed to create kdl chain from " + base_link_name + " to " + tool_link_name + " ...\n");
  }
  else
  {
    fk_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
    jac_solver.reset(new KDL::ChainJntToJacSolver(chain));
    ik_vel_solver.reset(new KDL::ChainIkSolverVel_pinv(chain));
    ik_solver.reset(new KDL::ChainIkSolverPos_NR(chain,*fk_solver,*ik_vel_solver,200,1e-6));
  }

}

int KinematicChain::getNumJoints() const
{
  return N_JOINTS;
}

arma::vec KinematicChain::getJointsPosition(const arma::mat &pose, const arma::vec &q0, bool *found_solution) const
{
  KDL::JntArray jnt(N_JOINTS);
  KDL::JntArray jnt0(N_JOINTS);

  for (int i=0;i<N_JOINTS;i++) jnt0(i) = q0(i);

  KDL::Frame kdl_pose;
  for (int i=0;i<3;i++)
  {
    kdl_pose.p[i] = pose(i,3);
    for (int j=0;j<3;j++) kdl_pose.M(i,j) = pose(i,j);
  }

  int ret = ik_solver->CartToJnt(jnt0,kdl_pose,jnt);

  if (found_solution) *found_solution = ret >= 0;

  arma::vec q = arma::vec().zeros(N_JOINTS);

  if (ret>=0)
  {
    for (int i=0;i<N_JOINTS;i++) q(i) = jnt(i);
  }

  return q;
}

arma::mat KinematicChain::getTaskPose(const arma::vec &j_pos) const
{
  arma::mat task_pose(4,4);

  KDL::JntArray jnt(N_JOINTS);
  for (int i=0;i<N_JOINTS;i++) jnt(i) = j_pos(i);

  KDL::Frame fk;
  fk_solver->JntToCart(jnt, fk);
  for (int i=0;i<3;i++)
  {
    for (int j=0;j<4;j++) task_pose(i,j) = fk(i,j);
  }
  task_pose.row(3) = arma::rowvec({0,0,0,1});

  return task_pose;
}

arma::mat KinematicChain::getJacobian(const arma::vec j_pos) const
{
  KDL::JntArray jnt(N_JOINTS);
  for (int i=0;i<N_JOINTS;i++) jnt(i) = j_pos(i);

  KDL::Jacobian J(N_JOINTS);
  jac_solver->JntToJac(jnt, J);
  arma::mat Jac(6, N_JOINTS);
  for (int i=0;i<Jac.n_rows;i++)
  {
    for (int j=0;j<Jac.n_cols;j++) Jac(i,j) = J(i,j);
  }

  return Jac;
}

}; // namespace robo_

}; // namespace as64_
