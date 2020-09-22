#include <grav_comp/robot/ur_robot.h>
#include <grav_comp/utils.h>

#define Ur_Robot_fun_ std::string("[Ur_Robot::") + __func__ + "]: "

Ur_Robot::Ur_Robot(bool use_sim)
{
  ros::NodeHandle nh("~");
  std::string robot_desc;
  std::string base_link;
  std::string tool_link;
  std::string robot_ip;
  int reverse_port;
  if (!nh.getParam("robot_description_name",robot_desc)) throw std::ios_base::failure(Ur_Robot_fun_ + "Failed to read parameter \"robot_description_name\".");
  if (!nh.getParam("base_link",base_link)) throw std::ios_base::failure(Ur_Robot_fun_ + "Failed to read parameter \"base_link\".");
  if (!nh.getParam("tool_link",tool_link)) throw std::ios_base::failure(Ur_Robot_fun_ + "Failed to read parameter \"tool_link\".");

  if (use_sim) robot_ip = "localhost";
  else if (!nh.getParam("robot_ip",robot_ip)) throw std::ios_base::failure(Ur_Robot_fun_ + "Failed to read parameter \"robot_ip\".");

  if (!nh.getParam("reverse_port",reverse_port))
  {
    reverse_port = 5001;
    PRINT_WARNING_MSG(std::string(Ur_Robot_fun_ + "Failed to read parameter \"reverse_port\".\n") + "Setting default: reverse_port = 5001\n");
  }

  std::vector<double> temp;
  if (nh.getParam("Fext_dead_zone",temp)) Fext_dead_zone = temp;

  // Initialize generic robot with the kuka-lwr model
  std::cerr << "=======> Creating robot...\n";
//  if (use_sim)
//  {
//    robot.reset(new lwr4p_::SimRobot(robot_desc, base_link, tool_link, ctrl_cycle));
//    bool limits_check;
//    if (!nh.getParam("limits_check",limits_check)) limits_check = false;
//    robot->setJointLimitCheck(limits_check);
//    robot->setSingularityCheck(true);
//    robot->setSingularityThreshold(0.01);
//    std::vector<double> q_start;
//    if (nh.getParam("q_start",q_start)) dynamic_cast<lwr4p_::SimRobot *>(robot.get())->initJointsPosition(arma::vec(q_start));
//  }
//  else
  robot.reset(new ur_::Robot(robot_desc, base_link, tool_link, robot_ip, reverse_port));

  std::cerr << "=======> ur-robot created successfully!\n";

  N_JOINTS = robot->getNumJoints();

  Ts = robot->getCtrlCycle();

  get_wrench_fun = std::bind(&Ur_Robot::getTaskWrenchFromRobot, this);

  mode.set(Robot::STOPPED);
  cmd_mode.set(Robot::IDLE);
  jpos_cmd.set(robot->getJointsPosition());

  std::thread robot_ctrl_thread = std::thread(&Ur_Robot::commandThread,this);
  int err_code = thr_::setThreadPriority(robot_ctrl_thread, SCHED_FIFO, 99);
  if (err_code) PRINT_WARNING_MSG(Ur_Robot_fun_ + "Failed to set thread priority! Reason:\n" + thr_::setThreadPriorErrMsg(err_code) + "\n", std::cerr);
  else PRINT_INFO_MSG(Ur_Robot_fun_ + "Set thread priority successfully!\n", std::cerr);
  robot_ctrl_thread.detach();

  mode_change.wait(); // wait for mode to be set
}

Ur_Robot::~Ur_Robot()
{}

void Ur_Robot::setMode(const Robot::Mode &mode)
{
  if (mode == cmd_mode.get()) return;

  cmd_mode.set(mode);
  mode_change.wait(); // wait to get notification from commandThread
}

void Ur_Robot::commandThread()
{
  unsigned long long count = 0;

  arma::mat J;
  arma::vec dq;

  arma::wall_clock timer;

  while (isOk())
  {
    timer.tic();

    Mode new_mode = cmd_mode.read();
    // check if we have to switch mode
    if (new_mode != mode.read())
    {
      switch (new_mode)
      {
        case JOINT_POS_CONTROL:
          robot->setNormalMode();
          jpos_cmd.set(robot->getJointsPosition());
          break;
        case JOINT_TORQUE_CONTROL:
          throw std::runtime_error(Ur_Robot_fun_ + "Unsupported mode \"JOINT_TORQUE_CONTROL\" for ur-robot...");
          break;
        case FREEDRIVE:
          robot->setFreedriveMode();
          break;
        case CART_VEL_CTRL:
          robot->setNormalMode();
          cart_vel_cmd.set(arma::vec().zeros(6));
          break;
        case IDLE:
          robot->setNormalMode();
          jpos_cmd.set(robot->getJointsPosition());
          break;
        case STOPPED:
          robot->setNormalMode();
          // robot->setExternalStop(true);
          mode.set(new_mode);
          mode_change.notify(); // unblock in case wait was called from another thread
          KRC_tick.notify(); // unblock in case wait was called from another thread
          return;
        case PROTECTIVE_STOP:
          mode.set(new_mode);
          continue;
          // if (main_ctrl) emit main_ctrl->gui->emergencyStopSignal();
          break;
      }
      mode.set(new_mode);
      mode_change.notify();

//      if (new_mode == PROTECTIVE_STOP) {
//        std::cerr << "mode = " << mode.get() << "\n";
//        exit(-1);
//      }
      continue;
    }

    // send command according to current mode
    switch (mode.read())
    {
      case JOINT_POS_CONTROL:
        robot->setJointsPosition(jpos_cmd.get());
        break;
      case CART_VEL_CTRL:
        robot->setTaskVelocity(cart_vel_cmd.get());
        break;
      case Robot::Mode::FREEDRIVE:
        // robot->setJointTorque(-robot->getRobotJacobian().t() * tool_estimator->getToolWrench(this->getTaskOrientation()));
        // robot->setJointsTorque(arma::vec().zeros(7));
        break;
      case Robot::Mode::IDLE:
        // std::cerr << "*** Send command in IDLE mode ***\n";
        // std::cerr << "Robot mode: " << robot->getModeName() << "\n";
        // robot->setJointsPosition(jpos_cmd.get());
        break;
      case Robot::Mode::STOPPED:
      case Robot::Mode::PROTECTIVE_STOP:
        // std::cerr << "***  MODE: STOPPED ***\n";
        break;
    }

    // sync with KRC
    if (robot->isOk()) robot->update();
    KRC_tick.notify();

    double elaps_time = timer.toc();
    if (elaps_time > 2*getCtrlCycle())
    {
      std::ostringstream oss;
      oss << elaps_time*1000;
      PRINT_WARNING_MSG(Ur_Robot_fun_ + "*** WARNING *** Elaps time = " + oss.str() + " ms\n");
    }
  }

  // mode_change.notify(); // unblock in case wait was called from another thread
  KRC_tick.notify(); // unblock in case wait was called from another thread
}

bool Ur_Robot::setJointsTrajectory(const arma::vec &qT, double duration)
{
  // keep last known robot mode
  Robot::Mode prev_mode = this->getMode();
  // start controller
  this->setMode(Robot::JOINT_POS_CONTROL);
  // std::cerr << "[Ur_Robot::setJointsTrajectory]: Mode changed to \"IDLE\"!\n";

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

    if (emergencyStop())
    {
      err_msg = "Emergency stop triggered!";
      setEmergencyStop(false);
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

  // std::cerr << "[Ur_Robot::setJointsTrajectory]: Mode restored to previous mode!\n";

  return true;
}

void Ur_Robot::stop()
{
  setMode(STOPPED);
}

void Ur_Robot::publishJointStates(const std::string &publish_jstates_topic)
{
  jState_pub.setPublishTopic(publish_jstates_topic);
  jState_pub.addFun(&ur_::RobotArm::addJointState, (ur_::RobotArm *)robot.get());
  jState_pub.setPublishCycle(0.033);
  jState_pub.start(); // launches joint states publisher thread
}

void Ur_Robot::useAtiSensor()
{
  ftsensor.reset(new ati::FTSensor);
  std::string ft_sensor_ip = "192.168.1.1";
  std::cerr << "=======> Initializing F/T sensor at ip: " << ft_sensor_ip << "\n";
  ftsensor->init(ft_sensor_ip.c_str());
  ftsensor->setTimeout(1.0);
  // ftsensor->setBias();
  std::cerr << "=======> F/T sensor initialized successfully!\n";

  get_wrench_fun = std::bind(&Ur_Robot::getTaskWrenchFromAti, this);
}

arma::vec Ur_Robot::getTaskWrenchFromAti() const
{
  if (!ftsensor) throw std::runtime_error(Ur_Robot_fun_ + "Ati sensor is not initialized...\n");

  static double measurements[6];
  uint32_t rdt(0),ft(0);
  (const_cast<ati::FTSensor *>(ftsensor.get()))->getMeasurements(measurements,rdt,ft);
  //ftsensor->getMeasurements(measurements,rdt,ft);

  arma::vec Fext(6);
  Fext(0) = measurements[0];
  Fext(1) = measurements[1];
  Fext(2) = measurements[2];
  Fext(3) = measurements[3];
  Fext(4) = measurements[4];
  Fext(5) = measurements[5];

  arma::mat R = this->getTaskRotMat();
  Fext.subvec(0,2) = R*Fext.subvec(0,2);
  Fext.subvec(3,5) = R*Fext.subvec(3,5);
  return Fext;
}

arma::vec Ur_Robot::getTaskWrenchFromRobot() const
{ return robot->getTcpWrench(); }
