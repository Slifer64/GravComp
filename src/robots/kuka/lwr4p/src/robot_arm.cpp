#include <lwr4p/robot_arm.h>

#include <stdexcept>
#include <sstream>
#include <chrono>
#include <map>
#include <stack>

#include <ros/package.h>
#include <kdl_parser/kdl_parser.hpp>

namespace as64_
{

namespace lwr4p_
{

RobotArm::RobotArm()
{
  std::string robot_description_name;
  if (!node.getParam("/lwr4p_robot/robot_description_name",robot_description_name))
  {
    throw std::runtime_error("[RobotArm Error]: Failed to load parameter \"/lwr4p_robot/robot_description_name\" ...\n");
  }

  if (!node.getParam("/lwr4p_robot/base_frame",base_link_name))
  {
    throw std::runtime_error("[RobotArm Error]: Failed to load parameter \"/lwr4p_robot/base_frame\" ...\n");
  }

  if (!node.getParam("/lwr4p_robot/tool_frame",tool_link_name))
  {
    throw std::runtime_error("[RobotArm Error]: Failed to load parameter \"/lwr4p_robot/tool_frame\" ...\n");
  }

  if (!node.getParam("/lwr4p_robot/ctrl_cycle",ctrl_cycle))
  {
    ctrl_cycle = 0.01;
  }

  if (!node.getParam("/lwr4p_robot/check_limits",check_limits))
  {
    check_limits = false;
  }

  if (!node.getParam("/lwr4p_robot/check_singularity",check_singularity))
  {
    check_singularity = false;
  }

  //std::string urdf_file_path = ros::package::getPath("lwr4p") + "/urdf/lwr4p_robot.urdf";
  if (!urdf_model.initParam(robot_description_name.c_str()))
  // if (!urdf_model.initFile(urdf_file_path.c_str()))
  {
    throw std::ios_base::failure("[RobotArm Error]: Couldn't load urdf model from \"" + robot_description_name + "\"...\n");
  }

  init();
}

RobotArm::RobotArm(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link, double ctrl_cycle)
{
  this->urdf_model = urdf_model;
  this->base_link_name = base_link;
  this->tool_link_name = tool_link;
  this->ctrl_cycle = ctrl_cycle;
  this->check_limits = false;
  this->check_singularity = false;

  init();
}

RobotArm::RobotArm(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link, double ctrl_cycle)
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

  mode_name[lwr4p_::IDLE] = "IDLE";
  mode_name[lwr4p_::FREEDRIVE] = "FREEDRIVE";
  mode_name[lwr4p_::JOINT_POS_CONTROL] = "JOINT_POS_CONTROL";
  mode_name[lwr4p_::JOINT_VEL_CONTROL] = "JOINT_VEL_CONTROL";
  mode_name[lwr4p_::JOINT_TORQUE_CONTROL] = "JOINT_TORQUE_CONTROL";
  mode_name[lwr4p_::CART_VEL_CONTROL] = "CART_VEL_CONTROL";
  mode_name[lwr4p_::CART_IMPEDANCE_CONTROL] = "CART_IMPEDANCE_CONTROL";
  mode_name[lwr4p_::PROTECTIVE_STOP] = "PROTECTIVE_STOP";

  mode = lwr4p_::Mode::IDLE;

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

lwr4p_::Mode RobotArm::getMode() const
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
  // keep last known robot mode
  lwr4p_::Mode prev_mode = getMode();

  // initialize position
  arma::vec q0 = getJointsPosition();
  arma::vec qref = q0;

  // initalize time
  double t = 0.0;
  double Ts = getCtrlCycle();

  int iters = 0;

  // start conttroller
  setMode(lwr4p_::Mode::JOINT_POS_CONTROL);
  while (isOk() && getMode()!=lwr4p_::Mode::IDLE && t<duration)
  {
    update();
    t += Ts;
    qref = get5thOrder(t, q0, j_targ, duration)[0];
    setJointsPosition(qref);
  }

  bool reached_target = t>=duration;

  if (isOk() && getMode()==lwr4p_::Mode::JOINT_POS_CONTROL) setMode(prev_mode);

  return reached_target;
}

arma::vec posError(const arma::vec &p1, const arma::vec &p2)
{
  return p1-p2;
}

arma::vec orientError(const arma::mat &R1, const arma::mat &R2)
{
  return (lwr4p_::rotm2quat(R1).subvec(1,3) - lwr4p_::rotm2quat(R2).subvec(1,3));
}

bool RobotArm::setTaskTrajectory(const arma::mat &target_pose, double duration)
{
  bool found_solution = false;
  // arma::vec j_pos0 = joint_pos;
  arma::vec j_pos0 = arma::vec().zeros(N_JOINTS);

  // find an initial joint configuration that yiels a pose close to target_pose
  for (int i=N_JOINTS-1; i>-1; i--)
  {
    int n_div = 8;
    arma::vec jp = arma::linspace<arma::vec>(robot_urdf->getJointPosLowLim(i), robot_urdf->getJointPosUpperLim(i), n_div);
    jp = jp.subvec(1,n_div-2);
    arma::vec j_pos = j_pos0;
    for (int j=0; j<jp.size() ; j++)
    {
      j_pos[i] = jp[j];
      arma::mat pose0 = robot_urdf->getTaskPose(j_pos0);
      arma::mat pose = robot_urdf->getTaskPose(j_pos);
      double e0 = arma::norm(lwr4p_::posError(pose0.submat(0,3,2,3),target_pose.submat(0,3,2,3))) + arma::norm(lwr4p_::orientError(pose0.submat(0,0,2,2),target_pose.submat(0,0,2,2)));
      double e = arma::norm(lwr4p_::posError(pose.submat(0,3,2,3),target_pose.submat(0,3,2,3))) + arma::norm(lwr4p_::orientError(pose.submat(0,0,2,2),target_pose.submat(0,0,2,2)));
      if (e < e0) j_pos0 = j_pos;
    }
  }

  arma::vec j_target = robot_urdf->getJointsPosition(target_pose, j_pos0, &found_solution);

  if (found_solution)
  {
    // check if the found solution respects the joint limits
    for (int i=0; i<j_target.size(); i++)
    {
      if (j_target(i)>robot_urdf->getJointPosUpperLim(i) || j_target(i)<robot_urdf->getJointPosLowLim(i)) return false;
    }
    return setJointsTrajectory(j_target, duration);
  }


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

  if (getMode()!=lwr4p_::Mode::CART_VEL_CONTROL &&
      getMode()!=lwr4p_::Mode::CART_VEL_CONTROL) return true;

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



}; // namespace lwr4p_

}; // namespace as64_
