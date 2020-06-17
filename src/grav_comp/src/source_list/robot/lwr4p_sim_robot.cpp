#include <grav_comp/robot/lwr4p_sim_robot.h>

LWR4p_Sim_Robot::LWR4p_Sim_Robot()
{
  N_JOINTS = 7;

  jpos_low_lim = -arma::vec({170, 120, 170, 120, 170, 120, 170});
  jpos_upper_lim = arma::vec({170, 120, 170, 120, 170, 120, 170});

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
  robot.reset(new lwr4p::LWR4pSimRobot(robot_desc, base_link, tool_link, ctrl_cycle));
  std::cerr << "=======> Robot created successfully!\n";

  jState_pub.setPublishCycle(0.0333); // 30 Hz
  std::string publish_states_topic;

  std::cerr << "*************  publish_states_topic = " << publish_states_topic << "\n";
  ros::NodeHandle("~").getParam("publish_states_topic",publish_states_topic);
//  jState_pub.setPublishTopic(publish_states_topic);
   jState_pub.setPublishTopic("/robot_joint_states");
  jState_pub.addFun(&lwr4p::LWR4pSimRobot::addJointState, robot.get());
  jState_pub.start(); // launches joint states publisher thread

  jnames.resize(N_JOINTS);
  jnames[0] = "lwr_joint_1";
  for (int i=1;i<N_JOINTS;i++)
  {
    jnames[i] = jnames[i-1];
    jnames[i].back()++;
  }

  mode.set(Robot::IDLE);
  cmd_mode.set(mode.get());
  jpos_cmd.set(robot->getJointPosition());

  std::thread robot_ctrl_thread = std::thread(&LWR4p_Sim_Robot::commandThread,this);
  int err_code = setThreadPriority(robot_ctrl_thread, SCHED_FIFO, 99);
  if (err_code) PRINT_WARNING_MSG("[LWR4p_Sim_Robot::LWR4p_Sim_Robot]: Failed to set thread priority! Reason:\n" + setThreadPriorErrMsg(err_code) + "\n", std::cerr);
  else PRINT_INFO_MSG("[LWR4p_Sim_Robot::LWR4p_Sim_Robot]: Set thread priority successfully!\n", std::cerr);
  robot_ctrl_thread.detach();
}

LWR4p_Sim_Robot::~LWR4p_Sim_Robot()
{
  jState_pub.stop();
}

void LWR4p_Sim_Robot::setMode(const Robot::Mode &mode)
{
  if (mode == cmd_mode.get()) return;

  cmd_mode.set(mode);
  mode_change.wait(); // wait to get notification from commandThread
}

void LWR4p_Sim_Robot::commandThread()
{
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
          robot->setMode(lwr4p::Mode::JOINT_POS_CTRL);
          jpos_cmd.set(robot->getJointPosition());
          break;
        case JOINT_TORQUE_CONTROL:
          robot->setMode(lwr4p::Mode::JOINT_TORQUE_CTRL);
          jtorque_cmd.set(arma::vec().zeros(N_JOINTS));
          break;
        case FREEDRIVE:
          robot->setMode(lwr4p::Mode::JOINT_TORQUE_CTRL);
          jtorque_cmd.set(arma::vec().zeros(N_JOINTS));
          break;
        case CART_VEL_CTRL:
          robot->setMode(lwr4p::Mode::JOINT_VEL_CTRL);
          cart_vel_cmd.set(arma::vec().zeros(6));
          break;
        case IDLE:
          robot->setMode(lwr4p::Mode::JOINT_POS_CTRL);
          jpos_cmd.set(robot->getJointPosition());
          break;
        case STOPPED:
          robot->setMode(lwr4p::Mode::JOINT_POS_CTRL);
          robot->setMode(lwr4p::Mode::STOPPED);
          // robot->setExternalStop(true);
          mode.set(new_mode);
          mode_change.notify(); // unblock in case wait was called from another thread
          KRC_tick.notify(); // unblock in case wait was called from another thread
          return;
      }
      mode.set(new_mode);
      mode_change.notify();

      continue;
    }

    // send command according to current mode
    switch (mode.read())
    {
      case JOINT_POS_CONTROL:
        robot->setJointPosition(jpos_cmd.get());
        break;
      case JOINT_TORQUE_CONTROL:
        // robot->setJointTorque(jtorque_cmd.get());
        break;
      case CART_VEL_CTRL:
        J = robot->getRobotJacobian();

        if (this->use_svf) dq = svf->pinv(J)*cart_vel_cmd.get();
        else dq = arma::pinv(J)*cart_vel_cmd.get();

        if (this->use_jlav) dq += jlav->getControlSignal(getJointsPosition());

        robot->setJointVelocity(dq);

        break;
      case Robot::Mode::FREEDRIVE:
        // robot->setJointTorque(jtorque_cmd.get());
        break;
      case Robot::Mode::IDLE:
        robot->setJointPosition(jpos_cmd.get());
        break;
    }

    // sync with KRC
    robot->waitNextCycle();
    KRC_tick.notify();

    // double elaps_time = timer.toc();
    // if (elaps_time > 1.5*getCtrlCycle())
    // {
    //   std::ostringstream oss;
    //   oss << elaps_time*1000;
    //   PRINT_WARNING_MSG("[LWR4p_Sim_Robot::commandThread]: *** WARNING *** Elaps time = " + oss.str() + " ms\n");
    // }
  }

  mode_change.notify(); // unblock in case wait was called from another thread
  KRC_tick.notify(); // unblock in case wait was called from another thread
}

arma::mat LWR4p_Sim_Robot::get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime) const
{
  arma::mat retTemp = arma::zeros<arma::mat>(p0.n_rows, 3);

  if (t < 0)
  {
    // before start
    retTemp.col(0) = p0;
  }
  else if (t > totalTime)
  {
    // after the end
    retTemp.col(0) = pT;
  }
  else
  {
    // somewhere betweeen ...
    // position
    retTemp.col(0) = p0 +
                     (pT - p0) * (10 * pow(t / totalTime, 3) -
                     15 * pow(t / totalTime, 4) +
                     6 * pow(t / totalTime, 5));
    // vecolity
    retTemp.col(1) = (pT - p0) * (30 * pow(t, 2) / pow(totalTime, 3) -
                     60 * pow(t, 3) / pow(totalTime, 4) +
                     30 * pow(t, 4) / pow(totalTime, 5));
    // acceleration
    retTemp.col(2) = (pT - p0) * (60 * t / pow(totalTime, 3) -
                     180 * pow(t, 2) / pow(totalTime, 4) +
                     120 * pow(t, 3) / pow(totalTime, 5));
  }

  // return vector
  return retTemp;
}

bool LWR4p_Sim_Robot::setJointsTrajectory(const arma::vec &qT, double duration)
{
  // keep last known robot mode
  Robot::Mode prev_mode = this->getMode();
  // start controller
  this->setMode(Robot::IDLE);
  // std::cerr << "[LWR4p_Sim_Robot::setJointsTrajectory]: Mode changed to \"IDLE\"!\n";

  // waits for the next tick
  update();

  arma::vec q0 = robot->getJointPosition();
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
      std::cerr << "=====> [LWR4p_Sim_Robot::setJointsTrajectory]: EMERGENCY STOP ACTIVATED!\n";
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

  // std::cerr << "[LWR4p_Sim_Robot::setJointsTrajectory]: Mode restored to previous mode!\n";

  return true;
}

void LWR4p_Sim_Robot::stop()
{
  robot->stop();
}
