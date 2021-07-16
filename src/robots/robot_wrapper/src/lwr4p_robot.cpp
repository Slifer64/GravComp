#include <robot_wrapper/lwr4p_robot.h>
#include <robot_wrapper/utils/xml_parser.h>

#include <ros/package.h>
#include <pthread.h>

namespace rw_
{

#define LWR4p_Robot_fun_ std::string("[LWR4p_Robot::") + __func__ + "]: "

LWR4p_Robot::LWR4p_Robot(bool use_sim)
{
  ros::NodeHandle nh("~");
  std::string robot_desc;
  std::string base_link;
  std::string tool_link;
  double ctrl_cycle;
  if (!nh.getParam("robot_description_name",robot_desc)) throw std::ios_base::failure("Failed to read parameter \"robot_description_name\".");
  if (!nh.getParam("base_link",base_link)) throw std::ios_base::failure("Failed to read parameter \"base_link\".");
  if (!nh.getParam("tool_link",tool_link)) throw std::ios_base::failure("Failed to read parameter \"tool_link\".");
  if (!nh.getParam("ctrl_cycle",ctrl_cycle)) throw std::ios_base::failure("Failed to read parameter \"ctrl_cycle\".");

  // Initialize generic robot with the kuka-lwr model
  std::cerr << "=======> Creating robot...\n";
  if (use_sim)
  {
    robot.reset(new lwr4p_::SimRobot(robot_desc, base_link, tool_link, ctrl_cycle));
    bool limits_check;
    if (!nh.getParam("limits_check",limits_check)) limits_check = false;
    robot->setJointLimitCheck(limits_check);
    robot->setSingularityCheck(true);
    robot->setSingularityThreshold(0.01);
    std::vector<double> q_start;
    if (nh.getParam("q_start",q_start)) dynamic_cast<lwr4p_::SimRobot *>(robot.get())->initJointsPosition(arma::vec(q_start));
  }
  else robot.reset(new lwr4p_::Robot(robot_desc, base_link, tool_link, ctrl_cycle));
  std::cerr << "=======> Robot created successfully!\n";

  initWrenchInterface(std::bind(&LWR4p_Robot::getTaskRotMat,this));
  initInvKinematicsInterface();
  initPublishStateInterface(robot_desc, getJointNames(), std::bind(&LWR4p_Robot::getJointsPosition, this));

  std::string params_filename = ros::package::getPath("robot_wrapper") + "/config/params.yaml";
  rw_::XmlParser parser(params_filename);

  std::string load_fail_msg = LWR4p_Robot_fun_ + "Failed to load param ";

  // =======  check whether to use joint limit avoidance  =======
  use_jlav = false;
  if (parser.getParam("use_jlav", use_jlav) && use_jlav)
  {
    double jlav_gain, jlim_safety_margin;
    if (!parser.getParam("jlav_gain", jlav_gain)) throw std::runtime_error(load_fail_msg + "\"jlav_gain\"...");
    if (!parser.getParam("jlim_safety_margin", jlim_safety_margin)) throw std::runtime_error(load_fail_msg + "\"jlim_safety_margin\"...");
    this->setJLAV(jlav_gain, jlim_safety_margin);
  }



  Ts = robot->getCtrlCycle();
  if (Ts == 0) throw std::runtime_error(LWR4p_Robot_fun_ + "control cycle is 0...");

  N_JOINTS = robot->getNumJoints();

  mode.set(rw_::STOPPED);
  cmd_mode.set(rw_::IDLE);
  jpos_cmd.set(robot->getJointsPosition());

  run_ = true;
  std::thread robot_ctrl_thread = std::thread(&LWR4p_Robot::commandThread,this);
  int err_code = thr_::setThreadPriority(robot_ctrl_thread, SCHED_FIFO, 99);
  if (err_code) PRINT_WARNING_MSG(LWR4p_Robot_fun_ + "Failed to set thread priority! Reason:\n" + thr_::setThreadPriorErrMsg(err_code) + "\n", std::cerr);
  else PRINT_INFO_MSG(LWR4p_Robot_fun_ + "Set thread priority successfully!\n", std::cerr);
  robot_ctrl_thread.detach();

  PRINT_INFO_MSG("OK r13\n");

  mode_change.wait(); // wait for mode to be set

  PRINT_INFO_MSG("OK r14\n");
}

LWR4p_Robot::~LWR4p_Robot()
{
  run_ = false;
}

void LWR4p_Robot::setMode(const Mode &mode)
{
  if (mode == cmd_mode.get()) return;

  cmd_mode.set(mode);
  while (getMode()!=mode && isOk()) // to avoid spurious wake ups
  {
    mode_change.wait(); // wait to get notification from commandThread
  }
}

void LWR4p_Robot::commandThread()
{
  unsigned long long count = 0;

  arma::mat J;
  arma::vec dq;

  arma::wall_clock timer;

  while (run_)
  {
    if (!isOk())
    {
      // notify all to avoid deadlocks
      //mode_change.notify();
      KRC_tick.notify();
      continue;
    }

    timer.tic();

    Mode new_mode = cmd_mode.read();
    // check if we have to switch mode
    if (new_mode != mode.read())
    {
      switch (new_mode)
      {
        case rw_::Mode::JOINT_POS_CONTROL:
          robot->setMode(lwr4p_::Mode::JOINT_POS_CONTROL);
          jpos_cmd.set(robot->getJointsPosition());
          break;
        case rw_::Mode::JOINT_TORQUE_CONTROL:
          robot->setMode(lwr4p_::Mode::JOINT_TORQUE_CONTROL);
          jtorque_cmd.set(arma::vec().zeros(N_JOINTS));
          break;
        case rw_::Mode::FREEDRIVE:
          robot->setMode(lwr4p_::Mode::FREEDRIVE);
          jtorque_cmd.set(arma::vec().zeros(N_JOINTS));
          break;
        case rw_::Mode::ADMITTANCE:
          robot->setMode(lwr4p_::Mode::JOINT_VEL_CONTROL);
          adm_ctrl->init();
          break;
        case rw_::Mode::CART_VEL_CTRL:
          robot->setMode(lwr4p_::Mode::JOINT_VEL_CONTROL);
          cart_vel_cmd.set(arma::vec().zeros(6));
          break;
        case rw_::Mode::IDLE:
          robot->setMode(lwr4p_::Mode::JOINT_POS_CONTROL);
          jpos_cmd.set(robot->getJointsPosition());
          break;
        case rw_::Mode::STOPPED:
          robot->setMode(lwr4p_::Mode::IDLE);
          // robot->setExternalStop(true);
          mode.set(new_mode);
          mode_change.notify(); // unblock in case wait was called from another thread
          KRC_tick.notify(); // unblock in case wait was called from another thread
          return;
        case rw_::Mode::PROTECTIVE_STOP:
          mode.set(new_mode);
          continue;
          // if (main_ctrl) emit main_ctrl->gui->emergencyStopSignal();
          break;
      }
      mode.set(new_mode);
      mode_change.notify();

      continue;
    }

    // send command according to current mode
    switch (mode.read())
    {
      case JOINT_POS_CONTROL:
        robot->setJointsPosition(jpos_cmd.get());
        break;
      case JOINT_TORQUE_CONTROL:
        robot->setJointsTorque(jtorque_cmd.get());
        break;
      case rw_::Mode::ADMITTANCE:
        adm_ctrl->update();
        dq = getInvJacobian()*adm_ctrl->getVelocity();
        if (this->use_jlav) dq += jlav->getControlSignal(getJointsPosition());
        robot->setJointsVelocity(dq);
        break;
      case CART_VEL_CTRL:
        dq = getInvJacobian()*cart_vel_cmd();
        if (this->use_jlav) dq += jlav->getControlSignal(getJointsPosition());
        robot->setJointsVelocity(dq);
        break;
      case rw_::Mode::FREEDRIVE:
        // robot->setJointTorque(-robot->getRobotJacobian().t() * tool_estimator->getToolWrench(this->getTaskOrientation()));
        // robot->setJointsTorque(arma::vec().zeros(7));
        break;
      case rw_::Mode::IDLE:
        robot->setJointsPosition(jpos_cmd.get());
        break;
      case rw_::Mode::STOPPED:
      case rw_::Mode::PROTECTIVE_STOP:
        break;
    }

    // sync with KRC
    if (robot->isOk()) robot->update();
    KRC_tick.notify();

    double elaps_time = timer.toc();
    // if (elaps_time > 1.5*getCtrlCycle())
    // {
    //   std::ostringstream oss;
    //   oss << elaps_time*1000;
    //   PRINT_WARNING_MSG(LWR4p_Robot_fun_ + "*** WARNING *** Elaps time = " + oss.str() + " ms\n");
    // }
  }

  // mode_change.notify(); // unblock in case wait was called from another thread
  KRC_tick.notify(); // unblock in case wait was called from another thread
}

bool LWR4p_Robot::setJointsTrajectory(const arma::vec &qT, double duration)
{
  // keep last known robot mode
  rw_::Mode prev_mode = this->getMode();
  // start controller
  this->setMode(rw_::JOINT_POS_CONTROL);
  // std::cerr << "[LWR4p_Robot::setJointsTrajectory]: Mode changed to \"IDLE\"!\n";

  // waits for the next tick
  update();

  arma::vec q0 = robot->getJointsPosition();
  arma::vec qref = q0;
  // std::cerr << "q0 = " << q0.t()*180/3.14159 << "\n";
  // std::cerr << "duration = " << duration << " sec\n";

  // robot->setMode(lwr4p::Mode::POSITION_CONTROL);
  // initalize time
  double t = 0.0;
  // the main while
  while (t < duration)
  {
    if (!isOk())
    {
      err_msg = "An error occured on the robot!";
      return false;
    }

    // compute time now
    t += getCtrlCycle();
    // update trajectory
    qref = get5thOrder(t, q0, qT, duration).col(0);

    // set joint positions
    jpos_cmd.set(qref);
    //setJointPosition(qref);

    // waits for the next tick
    update();
  }
  // reset last known robot mode
  this->setMode(prev_mode);

  // std::cerr << "[LWR4p_Robot::setJointsTrajectory]: Mode restored to previous mode!\n";

  return true;
}

void LWR4p_Robot::stop()
{
  setMode(STOPPED);
}

} // namespace rw_
