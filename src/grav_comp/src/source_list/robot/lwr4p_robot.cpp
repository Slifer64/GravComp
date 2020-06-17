#include <grav_comp/robot/lwr4p_robot.h>

#include <ros/package.h>

#include <pthread.h>

#include <grav_comp/utils.h>

LWR4p_Robot::LWR4p_Robot()
{
  is_ok = true;

  N_JOINTS = 7;

  jpos_low_lim = -arma::vec({170, 120, 170, 120, 170, 120, 170});
  jpos_upper_lim = arma::vec({170, 120, 170, 120, 170, 120, 170});

  jnames.resize(N_JOINTS);
  jnames[0] = "lwr_joint_1";
  for (int i=1;i<N_JOINTS;i++)
  {
    jnames[i] = jnames[i-1];
    jnames[i].back()++;
  }

  // Initialize generic robot with the kuka-lwr model
  std::cerr << "=======> Creating robot...\n";
  robot.reset(new lwr4p::LWR4pRobot());
  std::cerr << "=======> Robot created successfully!\n";

  std::string ft_sensor_ip = "192.168.2.1";
  std::cerr << "=======> Initializing F/T sensor at ip: " << ft_sensor_ip << "\n";
  ftsensor.init(ft_sensor_ip.c_str());
  ftsensor.setTimeout(1.0);
  // ftsensor.setBias();
  std::cerr << "=======> F/T sensor initialized successfully!\n";

  mode.set(Robot::IDLE);
  cmd_mode.set(mode.get());
  jpos_cmd.set(robot->getJointPosition());

  std::thread robot_ctrl_thread = std::thread(&LWR4p_Robot::commandThread,this);
  int err_code = setThreadPriority(robot_ctrl_thread, SCHED_FIFO, 99);
  if (err_code) PRINT_WARNING_MSG("[LWR4p_Robot::LWR4p_Robot]: Failed to set thread priority! Reason:\n" + setThreadPriorErrMsg(err_code) + "\n", std::cerr);
  else PRINT_INFO_MSG("[LWR4p_Robot::LWR4p_Robot]: Set thread priority successfully!\n", std::cerr);
  robot_ctrl_thread.detach();
}

LWR4p_Robot::~LWR4p_Robot()
{

}

void LWR4p_Robot::setMode(const Robot::Mode &mode)
{
  if (mode == cmd_mode.get()) return;

  cmd_mode.set(mode);
  mode_change.wait(); // wait to get notification from commandThread
}

void LWR4p_Robot::commandThread()
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
        robot->setJointTorque(jtorque_cmd.get());
        // std::cerr << "jtorque_cmd.get() = " << jtorque_cmd.get().t() << "\n";
        break;
      case CART_VEL_CTRL:
        J = robot->getRobotJacobian();

        if (this->use_svf) dq = svf->pinv(J)*cart_vel_cmd.get();
        else dq = arma::pinv(J)*cart_vel_cmd.get();

        if (this->use_jlav) dq += jlav->getControlSignal(getJointsPosition());

        // std::cerr << "dq = " << dq.t() << "\n";
        robot->setJointVelocity(dq);
        break;
      case Robot::Mode::FREEDRIVE:
        // robot->setJointTorque(-robot->getRobotJacobian().t() * tool_estimator->getToolWrench(this->getTaskOrientation()));
        robot->setJointTorque(arma::vec().zeros(7));
        break;
      case Robot::Mode::IDLE:
        // std::cerr << "*** Send command in IDLE mode ***\n";
        // std::cerr << "Robot mode: " << robot->getModeName() << "\n";
        robot->setJointPosition(jpos_cmd.get());
        break;
      case Robot::Mode::STOPPED:
        // std::cerr << "***  MODE: STOPPED ***\n";
        break;
    }

    // sync with KRC
    robot->waitNextCycle();
    KRC_tick.notify();

    double elaps_time = timer.toc();
    if (elaps_time > 1.5*getCtrlCycle())
    {
      std::ostringstream oss;
      oss << elaps_time*1000;
      PRINT_WARNING_MSG("[LWR4p_Robot::commandThread]: *** WARNING *** Elaps time = " + oss.str() + " ms\n");
    }
  }

  mode_change.notify(); // unblock in case wait was called from another thread
  KRC_tick.notify(); // unblock in case wait was called from another thread
}

bool LWR4p_Robot::setJointsTrajectory(const arma::vec &qT, double duration)
{
  // keep last known robot mode
  Robot::Mode prev_mode = this->getMode();
  // start controller
  this->setMode(Robot::IDLE);
  // std::cerr << "[LWR4p_Robot::setJointsTrajectory]: Mode changed to \"IDLE\"!\n";

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

  // std::cerr << "[LWR4p_Robot::setJointsTrajectory]: Mode restored to previous mode!\n";

  return true;
}

void LWR4p_Robot::stop()
{
  // setMode(Robot::STOPPED);
  robot->stop();
}
