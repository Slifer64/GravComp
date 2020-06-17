#include <lwr4p/lwr4p_sim_robot.h>
#include <iostream>
#include <string>
#include <stdexcept>
#include <sstream>
#include <chrono>
#include <map>
#include <stack>

#include <kdl_parser/kdl_parser.hpp>
#include <ros/package.h>

namespace lwr4p
{

  LWR4pSimRobot::LWR4pSimRobot(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link, double ctrl_cycle)
  {
    this->urdf_model = urdf_model;
    this->base_link_name = base_link;
    this->tool_link_name = tool_link;
    this->ctrl_cycle = ctrl_cycle;
    this->check_limits = true;
    this->check_singularity = false;

    init();
  }

  LWR4pSimRobot::LWR4pSimRobot(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link, double ctrl_cycle)
  {
    if (!urdf_model.initParam(robot_desc_param.c_str()))
    {
      throw std::ios_base::failure("Couldn't load urdf model from \"" + robot_desc_param + "\"...\n");
    }

    this->base_link_name = base_link;
    this->tool_link_name = tool_link;
    this->ctrl_cycle = ctrl_cycle;
    this->check_limits = false;
    this->check_singularity = false;

    init();
  }

  LWR4pSimRobot::~LWR4pSimRobot()
  {

  }

  void LWR4pSimRobot::init()
  {
    update_time = (unsigned long)(getControlCycle()*1e9);
    setSingularityThreshold(0.05);

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
      throw std::runtime_error("[LWR4pSimRobot::init]: Couldn't find specified base link \"" + base_link_name + "\" in the robot urdf model...\n");

    if (!found_tool_link)
      throw std::runtime_error("[LWR4pSimRobot::init]: Couldn't find specified tool link \"" + tool_link_name + "\" in the robot urdf model...\n");

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
      throw std::runtime_error("[LWR4pSimRobot::init]: Failed to create kdl chain from " + base_link_name + " to " + tool_link_name + " ...\n");
    }
    else
    {
      fk_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
      jac_solver.reset(new KDL::ChainJntToJacSolver(chain));
      ik_vel_solver.reset(new KDL::ChainIkSolverVel_pinv(chain));
      ik_solver.reset(new KDL::ChainIkSolverPos_NR(chain,*fk_solver,*ik_vel_solver,200,1e-6));
    }

    // preallocate space for all states
    N_JOINTS = joint_names.size();
    joint_pos.zeros(N_JOINTS);
    prev_joint_pos.zeros(N_JOINTS);
    Fext.zeros(6);

    this->mode = lwr4p::Mode::UNDEFINED;  // Dummy initialization before stopping controller
    setMode(lwr4p::Mode::JOINT_POS_CTRL);
  }

  void LWR4pSimRobot::setMode(lwr4p::Mode mode)
  {
    if (getMode() == mode) return;

    stopController();

    switch (mode)
    {
      case lwr4p::Mode::STOPPED:
        // saveLastJointPosition();
        break;
      case lwr4p::Mode::JOINT_POS_CTRL:
        // startJointPositionController();
        // saveLastJointPosition();
        break;
      case lwr4p::Mode::JOINT_VEL_CTRL:
        // if (this->mode != lwr4p::Mode::JOINT_POS_CTRL) startJointPositionController();
        this->mode = lwr4p::Mode::JOINT_VEL_CTRL;
        // saveLastJointPosition();
        break;
      case lwr4p::Mode::JOINT_TORQUE_CTRL:
        // startJointTorqueController();
        // saveLastJointPosition();
        break;
      case lwr4p::Mode::IMPEDANCE_CONTROL:
        // startCartImpController();
        // saveLastJointPosition();
        break;
      default:
        throw std::runtime_error("[LWR4pSimRobot::setMode]: Invalid mode " + getModeName(mode) + "...");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // simulate delay in mode change
    this->mode = mode;

    timer.start();
  }

  void LWR4pSimRobot::stopController()
  {
    this->mode = lwr4p::Mode::STOPPED;
    prev_joint_pos = joint_pos;
  }

  void LWR4pSimRobot::setCartStiffness(const arma::vec &input)
  {
    throw std::runtime_error("[LWR4pSimRobot::setCartStiffness]: Not implemented!\n");

    if (getMode() != lwr4p::Mode::IMPEDANCE_CONTROL)
    {
      PRINT_WARNING_MSG("[LWR4pSimRobot::setCartStiffness]: Cannot set Cartesian stiffness. Current mode is \"" + getModeName(getMode()) + "\"\n");
      return;
    }
  }

  void LWR4pSimRobot::setCartDamping(const arma::vec &input)
  {
    throw std::runtime_error("[LWR4pSimRobot::setCartDamping]: Not implemented!\n");

    if (getMode() != lwr4p::Mode::IMPEDANCE_CONTROL)
    {
      PRINT_WARNING_MSG("[LWR4pSimRobot::setCartDamping]: Cannot set Cartesian damping. Current mode is \"" + getModeName(getMode()) + "\"\n");
      return;
    }
  }

  void LWR4pSimRobot::waitNextCycle()
  {
    // ros::spinOnce();

    unsigned long elaps_time = timer.elapsedNanoSec();
    if (elaps_time < update_time)
    {
      std::this_thread::sleep_for(std::chrono::nanoseconds(update_time-elaps_time));
    }
    timer.start(); // restart time cycle
  }

  arma::vec LWR4pSimRobot::getJointExternalTorque() const
  {
    return arma::vec().zeros(7);
  }

  void LWR4pSimRobot::setJointPosition(const arma::vec &jpos)
  {
    if (getMode() != lwr4p::Mode::JOINT_POS_CTRL)
    {
      PRINT_WARNING_MSG("[LWR4pSimRobot::setJointPosition]: Cannot set joints position. Current mode is \"" + getModeName(getMode()) + "\"\n");
      return;
    }
    setJointPositionHelper(jpos);
  }

  void LWR4pSimRobot::setJointVelocity(const arma::vec &jvel)
  {
    if (getMode() != lwr4p::Mode::JOINT_VEL_CTRL)
    {
      PRINT_WARNING_MSG("[LWR4pSimRobot::setJointVelocity]: Cannot set joints velocity. Current mode is \"" + getModeName(getMode()) + "\"\n");
      return;
    }
    setJointVelocityHelper(jvel);
  }

  void LWR4pSimRobot::setJointTorque(const arma::vec &jtorq)
  {
    throw std::runtime_error("[LWR4pSimRobot::setJointTorque]: Not implemented!\n");

    if (getMode() != lwr4p::Mode::JOINT_TORQUE_CTRL)
    {
      PRINT_WARNING_MSG("[LWR4pSimRobot::setJointTorque]: Cannot set joint torques. Current mode is \"" + getModeName(getMode()) + "\"\n");
      return;
    }
  }

  void LWR4pSimRobot::setTaskPose(const arma::mat &task_pose)
  {
    throw std::runtime_error("[LWR4pSimRobot::setTaskPose]: Not implemented!\n");

    if (getMode() != lwr4p::Mode::IMPEDANCE_CONTROL)
    {
      PRINT_WARNING_MSG("[LWR4pSimRobot::setTaskPose]: Cannot set joint torques. Current mode is \"" + getModeName(getMode()) + "\"\n");
      return;
    }
  }

  void LWR4pSimRobot::setWrench(const arma::vec &wrench)
  {
    throw std::runtime_error("[LWR4pSimRobot::setWrench]: Not implemented!\n");

    if (getMode() != lwr4p::Mode::IMPEDANCE_CONTROL)
    {
      PRINT_WARNING_MSG("[LWR4pSimRobot::setWrench]: Cannot set joint torques. Current mode is \"" + getModeName(getMode()) + "\"\n");
      return;
    }
  }
  
  void LWR4pSimRobot::stop()
  {
    setMode(lwr4p::Mode::STOPPED);
  }

  bool LWR4pSimRobot::isOk() const
  {
    return getMode()!=lwr4p::Mode::STOPPED;
  }

  void LWR4pSimRobot::setJointLimitCheck(bool check)
  {
    check_limits = check;
  }

  void LWR4pSimRobot::setSingularityCheck(bool check)
  {
    check_singularity = check;
  }

  void LWR4pSimRobot::setSingularityThreshold(double thres)
  {
    SINGULARITY_THRES = thres;
  }

  void LWR4pSimRobot::addJointState(sensor_msgs::JointState &joint_state_msg)
  {
    std::unique_lock<std::mutex> lck(robot_state_mtx);

    arma::vec j_pos = getJointPosition();
    arma::vec j_vel = arma::vec().zeros(N_JOINTS);

    for (int i=0;i<N_JOINTS;i++)
    {
      joint_state_msg.name.push_back(joint_names[i]);
      joint_state_msg.position.push_back(j_pos(i));
      joint_state_msg.velocity.push_back(j_vel(i));
      joint_state_msg.effort.push_back(0.0);
    }
  }

  double LWR4pSimRobot::getControlCycle() const
  {
    return ctrl_cycle;
  }

  void LWR4pSimRobot::setJointTrajectory(const arma::vec &j_targ, double duration)
  {
    // keep last known robot mode
    lwr4p::Mode prev_mode = getMode();

    // initialize position
    arma::vec q0 = getJointPosition();
    arma::vec qref = q0;

    // initalize time
    double t = 0.0;
    double Ts = getControlCycle();

    int iters = 0;

    // start conttroller
    setMode(lwr4p::Mode::JOINT_POS_CTRL);
    while (isOk() && t<duration)
    {
      waitNextCycle();
      t += Ts;
      qref = get5thOrder(t, q0, j_targ, duration)[0];
      setJointPosition(qref);
    }

    bool reached_target = t>=duration;

    if (isOk() && getMode()==lwr4p::Mode::JOINT_POS_CTRL) setMode(prev_mode);
  }

  void LWR4pSimRobot::setJointPositionHelper(const arma::vec &j_pos)
  {
    arma::vec current_j_pos = joint_pos; // getJointPosition();
    arma::vec dj_pos = (j_pos - current_j_pos) / ctrl_cycle;

    if (check_limits)
    {
      if (!checkJointPosLimits(j_pos)) return;
      if (!checkJointVelLimits(dj_pos)) return;
    }

    if (check_singularity)
    {
      if (!checkSingularity()) return;
    }

    std::unique_lock<std::mutex> lck(robot_state_mtx);

    prev_joint_pos = current_j_pos;
    joint_pos = j_pos;
  }

  void LWR4pSimRobot::setJointVelocityHelper(const arma::vec &j_vel)
  {
    setJointPositionHelper(getJointPosition() + j_vel*ctrl_cycle);
  }

  void LWR4pSimRobot::setTaskVelocityHelper(const arma::vec &task_vel)
  {
    setJointVelocityHelper(arma::solve(getRobotJacobian(),task_vel));
  }

  arma::vec LWR4pSimRobot::getJointPosition() const
  {
    // std::unique_lock<std::mutex> lck(robot_state_mtx);
    return joint_pos;
  }

  arma::vec LWR4pSimRobot::getJointPosition(const arma::mat &pose, const arma::vec &q0, bool *found_solution) const
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

  arma::mat LWR4pSimRobot::getTaskPose() const
  {
    // std::unique_lock<std::mutex> lck(robot_state_mtx);
    return getTaskPose(getJointPosition());
  }

  arma::mat LWR4pSimRobot::getTaskPose(const arma::vec &j_pos) const
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

  arma::vec LWR4pSimRobot::getTaskPosition() const
  {
    // std::unique_lock<std::mutex> lck(robot_state_mtx);
    return getTaskPose().submat(0,3,2,3);
  }

  arma::mat LWR4pSimRobot::getTaskOrientation() const
  {
    // std::unique_lock<std::mutex> lck(robot_state_mtx);
    return getTaskPose().submat(0,0,2,2);
  }

  arma::mat LWR4pSimRobot::getRobotJacobian() const
  {
    // std::unique_lock<std::mutex> lck(robot_state_mtx);
    return getRobotJacobian(getJointPosition());
  }

  arma::mat LWR4pSimRobot::getRobotJacobian(const arma::vec &j_pos) const
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

  arma::mat LWR4pSimRobot::getEEJacobian() const
  {
    // std::unique_lock<std::mutex> lck(robot_state_mtx);
    arma::mat R = getTaskPose().submat(0,0,2,2);
    arma::mat Jrobot = getRobotJacobian();
    arma::mat Jee(6, N_JOINTS);
    Jee.submat(0,0,2,N_JOINTS-1) = R * Jrobot.submat(0,0,2,N_JOINTS-1);
    Jee.submat(3,0,5,N_JOINTS-1) = R * Jrobot.submat(3,0,5,N_JOINTS-1);

    return Jee;
  }

  arma::vec LWR4pSimRobot::getJointTorque() const
  {
    // std::unique_lock<std::mutex> lck(robot_state_mtx);
    return getRobotJacobian().t() * getExternalWrench();
  }

  arma::vec LWR4pSimRobot::getExternalWrench() const
  {
    // std::unique_lock<std::mutex> lck(robot_state_mtx);
    return arma::vec().zeros(6);
  }

  bool LWR4pSimRobot::checkJointPosLimits(const arma::vec &j_pos)
  {
    for (int i=0;i<N_JOINTS;i++)
    {
      if (j_pos(i)>joint_pos_upper_lim[i] || j_pos(i)<joint_pos_lower_lim[i])
      {
        std::ostringstream out;
        out << joint_names[i] << ": position limit reached: " << j_pos(i) << " rad";
        err_msg = out.str();
        // print_err_msg(err_msg);
        setMode(lwr4p::Mode::STOPPED);
        return false;
      }
    }

    return true;
  }

  bool LWR4pSimRobot::checkJointVelLimits(const arma::vec &dj_pos)
  {
    for (int i=0;i<N_JOINTS;i++)
    {
      if (std::fabs(dj_pos(i))>joint_vel_lim[i])
      {
        std::ostringstream out;
        out << joint_names[i] << ": velocity limit reached: " << dj_pos(i) << " rad/s";
        err_msg = out.str();
        // print_err_msg(err_msg);
        setMode(lwr4p::Mode::STOPPED);
        return false;
      }
    }

    return true;
  }

  bool LWR4pSimRobot::checkSingularity()
  {
    bool singularity_reached = false;

    arma::vec eigval; // = arma::eig_gen(Jrobot);
    arma::svd(eigval, getRobotJacobian());

    if (arma::min(arma::abs(eigval)) < SINGULARITY_THRES) singularity_reached = true;

    if (singularity_reached)
    {
      err_msg = "Singularity reached!";
      // print_err_msg(err_msg);
      setMode(lwr4p::Mode::STOPPED);
      return false;
    }

    return true;
  }


  // ===========================================================================

  arma::vec lwr4_friction(const arma::vec &dq)
  {
    arma::vec Fr(7);
    Fr(0) = 0.183379012548875*std::atan(100.0*dq(0));
    Fr(1) = 0.173198345026047*std::atan(100.0*dq(1));
    Fr(2) = 0.27004148667018*std::atan(100.0*dq(2));
    Fr(3) = 0.203723283726169*std::atan(100.0*dq(3));
    Fr(4) = 0.128735665726732*std::atan(100.0*dq(4));
    Fr(5) = 0.0210453117134514*std::atan(100.0*dq(5));
    Fr(6) = 0.0826258245130735*std::atan(100.0*dq(6));
    return Fr;
  }

  arma::vec lwr4_gravity(const arma::vec &q)
  {
    arma::vec sinq = arma::sin(q);
    arma::vec cosq = arma::cos(q);

    arma::vec G(7);

    G(0) = 0;
    G(1) = 981*(-(-0.0390675737668488*sinq(4)*sinq(5) + 0.0327196625084711*cosq(4))*sinq(2) + ((0.0390675737668488*cosq(5) + 1.3011637173221)*sinq(3) + (-0.0327196625084711*sinq(4) - 0.0390675737668488*sinq(5)*cosq(4) - 0.0220471479660755)*cosq(3) + 0.0283274807515703)*cosq(2))*cosq(1)/100 - 981*((0.0390675737668488*cosq(5) + 1.3011637173221)*cosq(3) - (-0.0327196625084711*sinq(4) - 0.0390675737668488*sinq(5)*cosq(4) - 0.0220471479660755)*sinq(3) + 3.33001322499426)*sinq(1)/100;
    G(2) = -981*(-0.0390675737668488*sinq(4)*sinq(5) + 0.0327196625084711*cosq(4))*sinq(1)*cosq(2)/100 - 981*((0.0390675737668488*cosq(5) + 1.3011637173221)*sinq(3) + (-0.0327196625084711*sinq(4) - 0.0390675737668488*sinq(5)*cosq(4) - 0.0220471479660755)*cosq(3) + 0.0283274807515703)*sinq(1)*sinq(2)/100;
    G(3) = -981*((0.0390675737668488*cosq(5) + 1.3011637173221)*sinq(3) + (-0.0327196625084711*sinq(4) - 0.0390675737668488*sinq(5)*cosq(4) - 0.0220471479660755)*cosq(3))*cosq(1)/100 + 981*((0.0390675737668488*cosq(5) + 1.3011637173221)*cosq(3) - (-0.0327196625084711*sinq(4) - 0.0390675737668488*sinq(5)*cosq(4) - 0.0220471479660755)*sinq(3))*sinq(1)*cosq(2)/100;
    G(4) = 981*(0.0390675737668488*sinq(4)*sinq(5) - 0.0327196625084711*cosq(4))*sinq(1)*cosq(2)*cosq(3)/100 - 981*(-0.0390675737668488*sinq(1)*sinq(2)*sinq(5) - 0.0327196625084711*sinq(3)*cosq(1))*cosq(4)/100 + 0.320979889208102*sinq(1)*sinq(2)*sinq(4) - 0.383252898652787*sinq(3)*sinq(4)*sinq(5)*cosq(1);
    G(5) = -981*(0.0390675737668488*sinq(3)*sinq(5) + 0.0390675737668488*cosq(3)*cosq(4)*cosq(5))*sinq(1)*cosq(2)/100 + 981*(0.0390675737668488*sinq(3)*cosq(4)*cosq(5) - 0.0390675737668488*sinq(5)*cosq(3))*cosq(1)/100 + 0.383252898652787*sinq(1)*sinq(2)*sinq(4)*cosq(5);
    G(6) = 0;
    return G;
  }

  arma::vec lwr4_fkine(const arma::vec &q)
  {
    arma::vec s = arma::sin(q);
    arma::vec c = arma::cos(q);
    arma::mat fk(4,4);

    fk(0,0)   = c(3)*s(0)*s(2)*s(4)*s(6)-c(0)*s(1)*s(3)*s(4)*s(6)-c(2)*c(4)*s(0)*s(6)-c(6)*s(0)*s(2)*s(3)*s(5)-c(0)*c(1)*c(4)*s(2)*s(6)-c(0)*c(3)*c(6)*s(1)*s(5)-c(2)*c(5)*c(6)*s(0)*s(4)-c(0)*c(1)*c(2)*c(3)*s(4)*s(6)+c(0)*c(1)*c(2)*c(6)*s(3)*s(5)-c(0)*c(1)*c(5)*c(6)*s(2)*s(4)+c(0)*c(4)*c(5)*c(6)*s(1)*s(3)-c(3)*c(4)*c(5)*c(6)*s(0)*s(2)+c(0)*c(1)*c(2)*c(3)*c(4)*c(5)*c(6);
    fk(0,1)   = c(0)*c(3)*s(1)*s(5)*s(6)-c(0)*c(6)*s(1)*s(3)*s(4)-c(2)*c(4)*c(6)*s(0)+c(3)*c(6)*s(0)*s(2)*s(4)+c(2)*c(5)*s(0)*s(4)*s(6)+s(0)*s(2)*s(3)*s(5)*s(6)-c(0)*c(1)*c(4)*c(6)*s(2)-c(0)*c(1)*c(2)*c(3)*c(6)*s(4)-c(0)*c(1)*c(2)*s(3)*s(5)*s(6)+c(0)*c(1)*c(5)*s(2)*s(4)*s(6)-c(0)*c(4)*c(5)*s(1)*s(3)*s(6)+c(3)*c(4)*c(5)*s(0)*s(2)*s(6)-c(0)*c(1)*c(2)*c(3)*c(4)*c(5)*s(6);
    fk(0,2)   = c(2)*s(0)*s(4)*s(5)-c(5)*s(0)*s(2)*s(3)-c(0)*c(3)*c(5)*s(1)+c(0)*c(1)*s(2)*s(4)*s(5)-c(0)*c(4)*s(1)*s(3)*s(5)+c(3)*c(4)*s(0)*s(2)*s(5)+c(0)*c(1)*c(2)*c(5)*s(3)-c(0)*c(1)*c(2)*c(3)*c(4)*s(5);
    fk(0,3)   = 0.39*c(0)*c(1)*c(2)*s(3)-0.39*c(0)*c(3)*s(1)-0.39*s(0)*s(2)*s(3)-0.4*c(0)*s(1)-0.088*c(0)*c(3)*c(5)*s(1)-0.088*c(5)*s(0)*s(2)*s(3)+0.088*c(2)*s(0)*s(4)*s(5)+0.088*c(0)*c(1)*s(2)*s(4)*s(5)-0.088*c(0)*c(4)*s(1)*s(3)*s(5)+0.088*c(3)*c(4)*s(0)*s(2)*s(5)+0.088*c(0)*c(1)*c(2)*c(5)*s(3)-0.088*c(0)*c(1)*c(2)*c(3)*c(4)*s(5);
    fk(1,0)   = c(0)*c(2)*c(4)*s(6)-c(1)*c(4)*s(0)*s(2)*s(6)-c(0)*c(3)*s(2)*s(4)*s(6)-c(3)*c(6)*s(0)*s(1)*s(5)+c(0)*c(6)*s(2)*s(3)*s(5)-s(0)*s(1)*s(3)*s(4)*s(6)+c(0)*c(2)*c(5)*c(6)*s(4)+c(0)*c(3)*c(4)*c(5)*c(6)*s(2)-c(1)*c(2)*c(3)*s(0)*s(4)*s(6)+c(1)*c(2)*c(6)*s(0)*s(3)*s(5)-c(1)*c(5)*c(6)*s(0)*s(2)*s(4)+c(4)*c(5)*c(6)*s(0)*s(1)*s(3)+c(1)*c(2)*c(3)*c(4)*c(5)*c(6)*s(0);
    fk(1,1)   = c(0)*c(2)*c(4)*c(6)-c(6)*s(0)*s(1)*s(3)*s(4)+c(3)*s(0)*s(1)*s(5)*s(6)-c(0)*s(2)*s(3)*s(5)*s(6)-c(1)*c(4)*c(6)*s(0)*s(2)-c(0)*c(3)*c(6)*s(2)*s(4)-c(0)*c(2)*c(5)*s(4)*s(6)-c(1)*c(2)*c(3)*c(6)*s(0)*s(4)-c(0)*c(3)*c(4)*c(5)*s(2)*s(6)-c(1)*c(2)*s(0)*s(3)*s(5)*s(6)+c(1)*c(5)*s(0)*s(2)*s(4)*s(6)-c(4)*c(5)*s(0)*s(1)*s(3)*s(6)-c(1)*c(2)*c(3)*c(4)*c(5)*s(0)*s(6);
    fk(1,2)   = c(0)*c(5)*s(2)*s(3)-c(3)*c(5)*s(0)*s(1)-c(0)*c(2)*s(4)*s(5)+c(1)*s(0)*s(2)*s(4)*s(5)-c(4)*s(0)*s(1)*s(3)*s(5)+c(1)*c(2)*c(5)*s(0)*s(3)-c(0)*c(3)*c(4)*s(2)*s(5)-c(1)*c(2)*c(3)*c(4)*s(0)*s(5);
    fk(1,3)   = 0.39*c(0)*s(2)*s(3)-0.39*c(3)*s(0)*s(1)-0.4*s(0)*s(1)+0.39*c(1)*c(2)*s(0)*s(3)-0.088*c(3)*c(5)*s(0)*s(1)+0.088*c(0)*c(5)*s(2)*s(3)-0.088*c(0)*c(2)*s(4)*s(5)+0.088*c(1)*s(0)*s(2)*s(4)*s(5)-0.088*c(4)*s(0)*s(1)*s(3)*s(5)+0.088*c(1)*c(2)*c(5)*s(0)*s(3)-0.088*c(0)*c(3)*c(4)*s(2)*s(5)-0.088*c(1)*c(2)*c(3)*c(4)*s(0)*s(5);
    fk(2,0)   = c(1)*c(3)*c(6)*s(5)-c(4)*s(1)*s(2)*s(6)+c(1)*s(3)*s(4)*s(6)-c(2)*c(3)*s(1)*s(4)*s(6)+c(2)*c(6)*s(1)*s(3)*s(5)-c(5)*c(6)*s(1)*s(2)*s(4)-c(1)*c(4)*c(5)*c(6)*s(3)+c(2)*c(3)*c(4)*c(5)*c(6)*s(1);
    fk(2,1)   = c(1)*c(6)*s(3)*s(4)-c(4)*c(6)*s(1)*s(2)-c(1)*c(3)*s(5)*s(6)-c(2)*s(1)*s(3)*s(5)*s(6)+c(5)*s(1)*s(2)*s(4)*s(6)-c(2)*c(3)*c(6)*s(1)*s(4)+c(1)*c(4)*c(5)*s(3)*s(6)-c(2)*c(3)*c(4)*c(5)*s(1)*s(6);
    fk(2,2)   = c(1)*c(3)*c(5)+c(2)*c(5)*s(1)*s(3)+c(1)*c(4)*s(3)*s(5)+s(1)*s(2)*s(4)*s(5)-c(2)*c(3)*c(4)*s(1)*s(5);
    fk(2,3)   = 0.4*c(1)+0.39*c(1)*c(3)+0.088*c(1)*c(3)*c(5)+0.39*c(2)*s(1)*s(3)+0.088*c(2)*c(5)*s(1)*s(3)+0.088*c(1)*c(4)*s(3)*s(5)+0.088*s(1)*s(2)*s(4)*s(5)+-0.088*c(2)*c(3)*c(4)*s(1)*s(5)+0.31;
    fk(3,0)   = 0;
    fk(3,1)   = 0;
    fk(3,2)   = 0;
    fk(3,3)   = 1;

    return fk;
  }

}  // namespace lwr
