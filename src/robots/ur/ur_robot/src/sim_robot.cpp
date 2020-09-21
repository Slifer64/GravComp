#include <ur_robot/sim_robot.h>

namespace as64_
{

namespace ur_
{

#define UR_SimRobot_fun_ std::string("[ur_::SimRobot::") + __func__ + "]: "

SimRobot::SimRobot(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link):
RobotArm(urdf_model, base_link, tool_link)
{
  initSimRobot();
}

SimRobot::SimRobot(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link):
RobotArm(robot_desc_param, base_link, tool_link)
{
  initSimRobot();
}

void SimRobot::initSimRobot()
{
  is_freedrive_on = false;

  std::function<arma::vec()> zero_wrench_fun = [](){ return arma::vec().zeros(6); };
  setGetExternalWrenchFun(zero_wrench_fun);
  // jState_pub = node.advertise<sensor_msgs::JointState>(pub_state_topic, 1);
  jState_sub = node.subscribe("/joint_states", 1, &SimRobot::jStateSubCallback, this);
  update_time = (unsigned long)(getCtrlCycle()*1e9);
  timer.start();
}

void SimRobot::update()
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);

  ros::spinOnce();

  unsigned long elaps_time = timer.elapsedNanoSec();
  if (elaps_time < update_time)
  {
    std::this_thread::sleep_for(std::chrono::nanoseconds(update_time-elaps_time));
  }
  timer.start(); // restart time cycle
}

bool SimRobot::isOk() const
{
  return getMode()!=ur_::Mode::PROTECTIVE_STOP;
}

void SimRobot::enable()
{
  mode = ur_::Mode::NORMAL;
  is_freedrive_on = false;
  update();
}

void SimRobot::setFreedriveMode()
{
  if (getMode() != ur_::Mode::PROTECTIVE_STOP) is_freedrive_on = true;
}

void SimRobot::setNormalMode()
{
  if (getMode() != ur_::Mode::PROTECTIVE_STOP) is_freedrive_on = false;
}

// =================  GET functions  ========================

arma::vec SimRobot::getJointsPosition() const
{
  std::unique_lock<std::mutex> lck(robot_state_mtx);
  return joint_pos;
}

arma::vec SimRobot::getJointsVelocity() const
{
  throw std::runtime_error(UR_SimRobot_fun_ + "Function is not implemented...\n");
}

arma::mat SimRobot::getTaskPose() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return robot_urdf->getTaskPose(getJointsPosition());
}

arma::vec SimRobot::getTaskPosition() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return getTaskPose().submat(0,3,2,3);
}

arma::mat SimRobot::getTaskRotm() const
{
  return getTaskPose().submat(0,0,2,2);
}

arma::vec SimRobot::getTaskQuat() const
{
  return rotm2quat(getTaskRotm());
}

arma::vec SimRobot::getTaskVelocity() const
{
  throw std::runtime_error(UR_SimRobot_fun_ + "Function is not implemented...\n");
}

arma::mat SimRobot::getJacobian() const
{
  return robot_urdf->getJacobian(getJointsPosition());
}

arma::mat SimRobot::getEEJacobian() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  arma::mat R = getTaskPose().submat(0,0,2,2);
  arma::mat Jrobot = getJacobian();
  arma::mat Jee(6, N_JOINTS);
  Jee.submat(0,0,2,N_JOINTS-1) = R * Jrobot.submat(0,0,2,N_JOINTS-1);
  Jee.submat(3,0,5,N_JOINTS-1) = R * Jrobot.submat(3,0,5,N_JOINTS-1);

  return Jee;
}


arma::vec SimRobot::getTcpWrench() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  if (!get_wrench_fun_ptr) throw std::runtime_error(UR_SimRobot_fun_ + "Function is not initialized...\n");
  return get_wrench_fun_ptr();
}

// =================  SET functions  ========================

void SimRobot::setJointsPosition(const arma::vec &j_pos)
{
  err_msg = ""; // clear the error msg
  setJointsPositionHelper(j_pos);
}

void SimRobot::setJointsVelocity(const arma::vec &j_vel)
{
  arma::vec j_pos = getJointsPosition() + j_vel*getCtrlCycle();
  setJointsPositionHelper(j_pos);
}

void SimRobot::setTaskVelocity(const arma::vec &task_vel)
{
  arma::vec j_vel = arma::solve(getJacobian(),task_vel);
  arma::vec j_pos = getJointsPosition() + j_vel*getCtrlCycle();
  setJointsPositionHelper(j_pos);
}

void SimRobot::setTaskPose(const arma::mat &task_pose)
{
  throw std::runtime_error("[ERROR]: SimRobot::setTaskPose is not supported.");
}

void SimRobot::setJointsPositionHelper(const arma::vec &j_pos)
  {
    if (checkLimits(j_pos))
    {
      std::unique_lock<std::mutex> lck(robot_state_mtx);
      joint_pos = j_pos;
    }
    else protectiveStop();
  }

// =====================================================

void SimRobot::protectiveStop()
{
  is_freedrive_on = false;
  mode = ur_::Mode::PROTECTIVE_STOP;
  print_warn_msg("Mode changed to \"" + getModeName(getMode()) + "\"\n");
}

void SimRobot::jStateSubCallback(const sensor_msgs::JointState::ConstPtr& j_state)
{
  if (!is_freedrive_on) return;

  std::map<std::string, double> j_map;

  for (int i=0;i<j_state->name.size();i++)
    j_map.insert( std::pair<std::string,double>(j_state->name[i], j_state->position[i]) );

  std::unique_lock<std::mutex> lck(robot_state_mtx);
  for (int i=0;i<robot_urdf->getNumJoints();i++)
    joint_pos[i] = j_map[robot_urdf->getJointName(i)];
  // joint_vel = j_state->velocity;
  // joint_torques = j_state->effort;
}

void SimRobot::initJointsPosition(const arma::vec j_pos0)
{
  joint_pos = j_pos0;
}

}; // namespace ur_

}; // namespace as64_
