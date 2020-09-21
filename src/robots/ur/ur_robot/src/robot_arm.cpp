#include <ur_robot/robot_arm.h>

#include <stdexcept>
#include <sstream>
#include <chrono>
#include <map>
#include <stack>

#include <ros/package.h>
#include <kdl_parser/kdl_parser.hpp>

namespace as64_
{

namespace ur_
{

RobotArm::RobotArm(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link)
{
  this->urdf_model = urdf_model;
  this->base_link_name = base_link;
  this->tool_link_name = tool_link;
  this->ctrl_cycle = ctrl_cycle;
  this->check_limits = false;
  this->check_singularity = false;

  init();
}

RobotArm::RobotArm(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link)
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

RobotArm::~RobotArm() {}

void RobotArm::init()
{
  robot_urdf.reset(new RobotUrdf(urdf_model, base_link_name, tool_link_name));

  mode_name[ur_::NORMAL] = "NORMAL";
  mode_name[ur_::FREEDRIVE] = "FREEDRIVE";
  mode_name[ur_::STOPPED] = "STOPPED";
  mode_name[ur_::PROTECTIVE_STOP] = "PROTECTIVE_STOP";

  mode = ur_::Mode::NORMAL;

  setSingularityThreshold(0.05);

  // preallocate space for all states
  N_JOINTS = robot_urdf->getNumJoints();
  Fext.zeros(6);
}

void RobotArm::setJointLimitCheck(bool check)
{
  check_limits = check;
}

void RobotArm::setSingularityCheck(bool check)
{
  check_singularity = check;
}

void RobotArm::setSingularityThreshold(double thres)
{
  SINGULARITY_THRES = thres;
}

void RobotArm::setGetExternalWrenchFun(arma::vec (*getWrenchFun)(void))
{
  get_wrench_fun_ptr = std::bind(getWrenchFun);
}

void RobotArm::setGetExternalWrenchFun(std::function<arma::vec()> get_wrench_fun)
{
  get_wrench_fun_ptr = get_wrench_fun;
}

std::string RobotArm::getErrMsg() const
{
  return err_msg;
}

void RobotArm::addJointState(sensor_msgs::JointState &joint_state_msg)
{
  arma::vec j_pos = getJointsPosition();

  for (int i=0;i<N_JOINTS;i++)
  {
    joint_state_msg.name.push_back(robot_urdf->getJointName(i));
    joint_state_msg.position.push_back(j_pos(i));
    // joint_state_msg.velocity.push_back(j_vel(i));
    // joint_state_msg.effort.push_back(0.0);
  }
}

double RobotArm::getCtrlCycle() const
{
  return ctrl_cycle;
}

ur_::Mode RobotArm::getMode() const
{
  return mode;
}

std::string RobotArm::getModeName(Mode mode) const
{
  return (mode_name.find(mode))->second;
}

int RobotArm::getNumJoints() const
{
  return N_JOINTS;
}

bool RobotArm::setJointsTrajectory(const arma::vec &j_targ, double duration)
{
  // initialize position
  arma::vec q0 = getJointsPosition();
  arma::vec qref = q0;

  // initalize time
  double t = 0.0;
  double Ts = getCtrlCycle();

  int iters = 0;

  // start conttroller
  while (isOk() && t<duration)
  {
    update();
    t += Ts;
    qref = get5thOrder(t, q0, j_targ, duration)[0];
    setJointsPosition(qref);
  }
  bool reached_target = t>=duration;

  return reached_target;
}

bool RobotArm::setTaskTrajectory(const arma::mat &target_pose, double duration)
{
  return false;
}

bool RobotArm::checkLimits(const arma::vec &j_pos)
{
  arma::vec current_j_pos = getJointsPosition();
  arma::vec dj_pos = (j_pos - current_j_pos) / ctrl_cycle;

  if (check_limits)
  {
    if (!checkJointPosLimits(j_pos)) return false;
    if (!checkJointVelLimits(dj_pos)) return false;
  }

  if (check_singularity)
  {
    if (!checkSingularity(j_pos)) return false;
  }

  return true;
}

bool RobotArm::checkJointPosLimits(const arma::vec &j_pos)
{
  for (int i=0;i<N_JOINTS;i++)
  {

    if (j_pos(i)>robot_urdf->getJointPosUpperLim(i) || j_pos(i)<robot_urdf->getJointPosLowLim(i))
    {
      std::ostringstream out;
      out << robot_urdf->getJointName(i) << ": position limit reached: " << j_pos(i) << " rad";
      err_msg = out.str();
      print_err_msg(err_msg);
      return false;
    }
  }

  return true;
}

bool RobotArm::checkJointVelLimits(const arma::vec &dj_pos)
{
  for (int i=0;i<N_JOINTS;i++)
  {
    if (std::fabs(dj_pos(i))>robot_urdf->getJointVelLim(i))
    {
      std::ostringstream out;
      out << robot_urdf->getJointName(i) << ": velocity limit reached: " << dj_pos(i) << " rad/s";
      err_msg = out.str();
      print_err_msg(err_msg);
      return false;
    }
  }

  return true;
}

bool RobotArm::checkSingularity(const arma::vec &j_pos)
{
  bool singularity_reached = false;

  arma::vec eigval; // = arma::eig_gen(Jrobot);
  arma::svd(eigval, robot_urdf->getJacobian(j_pos));

  if (arma::min(arma::abs(eigval)) < SINGULARITY_THRES) singularity_reached = true;

  if (singularity_reached)
  {
    err_msg = "Singularity reached!";
    // print_err_msg(err_msg);
    return false;
  }

  return true;
}

void RobotArm::readWrenchFromTopic(const std::string &topic)
{
  wrench_reader.reset(new WrenchReader(topic));
  setGetExternalWrenchFun(&WrenchReader::getWrench, wrench_reader.get());
}



}; // namespace ur_

}; // namespace as64_
