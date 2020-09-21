#include <lwr4p/sim_robot.h>

namespace as64_
{

namespace lwr4p_
{

#define LWR4p_SimRobot_fun_ std::string("[lwr4p_::SimRobot::") + __func__ + "]: "

SimRobot::SimRobot()
{
  initSimRobot();
}

SimRobot::SimRobot(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link,
                   double ctrl_cycle): RobotArm(urdf_model, base_link, tool_link, ctrl_cycle)
{
  initSimRobot();
}

SimRobot::SimRobot(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link,
                   double ctrl_cycle): RobotArm(robot_desc_param, base_link, tool_link, ctrl_cycle)
{
  initSimRobot();
}

void SimRobot::initSimRobot()
{
  std::function<arma::vec()> zero_wrench_fun = [](){ return arma::vec().zeros(6); };
  setGetExternalWrenchFun(zero_wrench_fun);
  // jState_pub = node.advertise<sensor_msgs::JointState>(pub_state_topic, 1);
  jState_sub = node.subscribe("/joint_states", 1, &SimRobot::jStateSubCallback, this);
  update_time = (unsigned long)(getCtrlCycle()*1e9);
  timer.start();
}

void SimRobot::setMode(const lwr4p_::Mode &m)
{
  if (getMode() == m) return;

  if (m == PROTECTIVE_STOP)
  {
    protectiveStop();
    return;
  }

  stop();
  if (!isOk()) return;

  switch (m) {
    case lwr4p_::Mode::IDLE:
    case lwr4p_::Mode::FREEDRIVE:
    case lwr4p_::Mode::JOINT_POS_CONTROL:
    case lwr4p_::Mode::JOINT_VEL_CONTROL:
    case lwr4p_::Mode::CART_VEL_CONTROL:
      break;
    case lwr4p_::Mode::JOINT_TORQUE_CONTROL:
    case lwr4p_::Mode::CART_IMPEDANCE_CONTROL:
      throw std::runtime_error("[lwr4+::SimRobot::setMode]: \"CART_IMPEDANCE_CONTROL\" is not supported.");
  }

  mode = m;
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
  return getMode()!=lwr4p_::Mode::PROTECTIVE_STOP;
}

void SimRobot::enable()
{
  mode = lwr4p_::Mode::IDLE;
  update();
}

// =================  GET functions  ========================

arma::vec SimRobot::getJointsPosition() const
{
  std::unique_lock<std::mutex> lck(robot_state_mtx);
  return joint_pos;
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

arma::vec SimRobot::getJointsTorque() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  throw std::runtime_error("[ERROR]: SimRobot::getJointsTorque is not supported.");
}

arma::vec SimRobot::getExternalWrench() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  if (!get_wrench_fun_ptr) throw std::runtime_error(LWR4p_SimRobot_fun_ + "Function is not initialized...\n");
  return get_wrench_fun_ptr();
}

arma::vec SimRobot::getJointExternalTorque() const
{
  return getJacobian().t() * getExternalWrench();
}

// =================  SET functions  ========================

void SimRobot::setJointsPosition(const arma::vec &j_pos)
{
  if (getMode() != lwr4p_::Mode::JOINT_POS_CONTROL)
  {
    print_warn_msg("[lwr4+::SimRobot::setJointsPosition]: Cannot set joints position. Current mode is \"" + getModeName(getMode()) + "\"\n");
    return;
  }

  err_msg = ""; // clear the error msg
  setJointsPositionHelper(j_pos);
}

void SimRobot::setJointsVelocity(const arma::vec &j_vel)
{

  if (getMode() != lwr4p_::Mode::JOINT_VEL_CONTROL)
  {
    print_warn_msg("[lwr4+::SimRobot::setJointsVelocity]: Cannot set joints velocity. Current mode is \"" + getModeName(getMode()) + "\"\n");
    return;
  }

  err_msg = ""; // clear the error msg

  arma::vec j_pos = getJointsPosition() + j_vel*getCtrlCycle();
  setJointsPositionHelper(j_pos);

}

void SimRobot::setTaskVelocity(const arma::vec &task_vel)
{
  err_msg = ""; // clear the error msg
  if (getMode() != lwr4p_::Mode::CART_VEL_CONTROL)
  {
    print_warn_msg("[lwr4+::SimRobot::setTaskVelocity]: Cannot set task velocity. Current mode is \"" + getModeName(getMode()) + "\"\n");
    return;
  }

  arma::vec j_vel = arma::solve(getJacobian(),task_vel);
  arma::vec j_pos = getJointsPosition() + j_vel*getCtrlCycle();
  setJointsPositionHelper(j_pos);
}

void SimRobot::setJointsTorque(const arma::vec &j_torques)
{
  throw std::runtime_error("[ERROR]: SimRobot::setJointsTorque is not supported.");
}

void SimRobot::setTaskPose(const arma::mat &task_pose)
{
  throw std::runtime_error("[ERROR]: SimRobot::setTaskPose is not supported.");
}

void SimRobot::setWrench(const arma::vec &wrench)
{
  throw std::runtime_error("[ERROR]: SimRobot::setWrench is not supported.");
}

void SimRobot::setCartStiffness(const arma::vec &cart_stiff)
{
  this->cart_stiff = cart_stiff;
}

void SimRobot::setCartDamping(const arma::vec &cart_damp)
{
  this->cart_damp = cart_damp;
}

void SimRobot::setJointsPositionHelper(const arma::vec &j_pos)
  {
    if (checkLimits(j_pos))
    {
      std::unique_lock<std::mutex> lck(robot_state_mtx);
      joint_pos = j_pos;
    }
    else setMode(lwr4p_::Mode::PROTECTIVE_STOP);
  }

// =====================================================

void SimRobot::stop()
{
  if (getMode() == lwr4p_::IDLE) return;

  update();
  arma::vec q_current = getJointsPosition();
  setJointsPositionHelper(q_current);
  if (isOk()) mode = lwr4p_::IDLE;
}

void SimRobot::protectiveStop()
{
  if (getMode() == lwr4p_::PROTECTIVE_STOP) return;

  mode = lwr4p_::Mode::PROTECTIVE_STOP;
  print_warn_msg("Mode changed to \"" + getModeName(getMode()) + "\"\n");
}

void SimRobot::jStateSubCallback(const sensor_msgs::JointState::ConstPtr& j_state)
{
  if (getMode() != lwr4p_::Mode::FREEDRIVE) return;

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

}; // namespace lwr4p_

}; // namespace as64_
