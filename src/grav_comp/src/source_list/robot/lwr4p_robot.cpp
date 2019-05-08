#include <grav_comp/robot/lwr4p_robot.h>

#include <ros/package.h>

#include <pthread.h>

arma::vec quatProd(const arma::vec &quat1, const arma::vec &quat2)
{
  arma::vec quat12(4);

  double n1 = quat1(0);
  arma::vec e1 = quat1.subvec(1,3);

  double n2 = quat2(0);
  arma::vec e2 = quat2.subvec(1,3);

  quat12(0) = n1*n2 - arma::dot(e1,e2);
  quat12.subvec(1,3) = n1*e2 + n2*e1 + arma::cross(e1,e2);

  return quat12;
}

arma::vec quatExp(const arma::vec &v_rot, double zero_tol=1e-16)
{
  arma::vec quat(4);
  double norm_v_rot = arma::norm(v_rot);
  double theta = norm_v_rot;

 if (norm_v_rot > zero_tol)
 {
    quat(0) = std::cos(theta/2);
    quat.subvec(1,3) = std::sin(theta/2)*v_rot/norm_v_rot;
  }
  else{
    quat << 1 << 0 << 0 << 0;
  }

  return quat;
}

arma::vec quatLog(const arma::vec &quat, double zero_tol=1e-16)
{
  arma::vec e = quat.subvec(1,3);
  double n = quat(0);

  if (n > 1) n = 1;
  if (n < -1) n = -1;

  arma::vec omega(3);
  double e_norm = arma::norm(e);

  if (e_norm > zero_tol) omega = 2*std::atan2(e_norm,n)*e/e_norm;
  else omega = arma::vec().zeros(3);

  return omega;
}

arma::vec quatInv(const arma::vec &quat)
{
  arma::vec quatI(4);

  quatI(0) = quat(0);
  quatI.subvec(1,3) = - quat.subvec(1,3);

  return quatI;
}

LWR4p_Robot::LWR4p_Robot(const ToolEstimator *tool_est):Robot(tool_est)
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
  robot.reset(new lwr4p::Robot());
  std::cerr << "=======> Robot created successfully!\n";

  std::string ft_sensor_ip = "192.168.2.1";
  std::cerr << "Initializing F/T sensor at ip: " << ft_sensor_ip << "\n";
  ftsensor.init(ft_sensor_ip.c_str());
  ftsensor.setTimeout(1.0);
  // ftsensor.setBias();

  std::cerr << "Initialed\n";

  mode.set(Robot::STOPPED);
  cmd_mode.set(mode.get());
  jpos_cmd.set(robot->getJointPosition());

  launchRobotRTCtrlThread();
}

LWR4p_Robot::~LWR4p_Robot()
{

}

void LWR4p_Robot::launchRobotRTCtrlThread()
{
  std::thread rt_run_thread = std::thread(&LWR4p_Robot::commandThread,this);
  struct sched_param sch_param;
  sch_param.sched_priority = 99;
  int policy = SCHED_FIFO;
  int ret = pthread_setschedparam(rt_run_thread.native_handle(), policy, &sch_param);

  if (ret)
  {
    switch (ret)
    {
      std::cerr << "[LWR4p_Robot::launchRobotRTCtrlThread]:\n ********* ERROR setting commandThread to run in RT *********\n===> Reason:\n";
      case ESRCH:
        std::cerr << "No thread with the ID thread could be found.\n";
        break;
      case EINVAL:
        std::cerr << "Policy is not a recognized policy, or param does not make sense for the policy.\n";
        break;
      case EPERM:
        std::cerr << "The caller does not have appropriate privileges to set the specified scheduling policy and parameters.\n";
        break;
      case ENOTSUP:
        std::cerr << "Attempt was made to set the policy or scheduling parameters to an unsupported value.\n";
        break;
      default:
        std::cerr << "Unknown error coce: \"" << ret << "\"\n";
    }
  }
  rt_run_thread.detach();
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

  while (isOk())
  {
    Mode new_mode = cmd_mode.read();
    // check if we have to switch mode
    if (new_mode != mode.read())
    {
      switch (new_mode)
      {
        case JOINT_POS_CONTROL:
          robot->setMode(lwr4p::Mode::POSITION_CONTROL);
          jpos_cmd.set(robot->getJointPosition());
          break;
        case JOINT_TORQUE_CONTROL:
          robot->setMode(lwr4p::Mode::TORQUE_CONTROL);
          jtorque_cmd.set(arma::vec().zeros(N_JOINTS));
          break;
        case FREEDRIVE:
          robot->setMode(lwr4p::Mode::TORQUE_CONTROL);
          jtorque_cmd.set(arma::vec().zeros(N_JOINTS));
          break;
        case CART_VEL_CTRL:
          robot->setMode(lwr4p::Mode::VELOCITY_CONTROL);
          cart_vel_cmd.set(arma::vec().zeros(6));
          break;
        case IDLE:
          std::cerr << "===>> Setting MODE to: IDLE\n";
          robot->setMode(lwr4p::Mode::POSITION_CONTROL);
          jpos_cmd.set(robot->getJointPosition());
          break;
        case STOPPED:
          robot->setMode(lwr4p::Mode::POSITION_CONTROL);
          robot->setMode(lwr4p::Mode::STOPPED);
          // robot->setExternalStop(true);
          mode.set(new_mode);
          mode_change.notify(); // unblock in case wait was called from another thread
          KRC_tick.notify(); // unblock in case wait was called from another thread
          return;
      }
      mode.set(new_mode);
      mode_change.notify();
    }

    // send command according to current mode
    switch (mode.read())
    {
      case JOINT_POS_CONTROL:
        std::cerr << "***  MODE: JOINT_POS_CONTROL ***\n";
        robot->setJointPosition(jpos_cmd.get());
        break;
      case JOINT_TORQUE_CONTROL:
        robot->setJointTorque(jtorque_cmd.get());
        // std::cerr << "jtorque_cmd.get() = " << jtorque_cmd.get().t() << "\n";
        break;
      case CART_VEL_CTRL:
        J = robot->getRobotJacobian();
        dq = arma::pinv(J)*cart_vel_cmd.get();
        // std::cerr << "dq = " << dq.t() << "\n";
        robot->setJointVelocity(dq);
        break;
      case Robot::Mode::FREEDRIVE:
        robot->setJointTorque(jtorque_cmd.get());
        break;
      case Robot::Mode::IDLE:
        std::cerr << "***  MODE: IDLE ***\n";
        robot->setJointPosition(jpos_cmd.get());
        break;
      case Robot::Mode::STOPPED:
        std::cerr << "***  MODE: STOPPED ***\n";
        break;
    }

    std::cerr << "count = " << count++ << "\n";

    // sync with KRC
    robot->waitNextCycle();
    KRC_tick.notify();
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
