#include <robot_wrapper/ur_robot.h>
// #include <project_name_/utils.h>

#include <ros/package.h>
#include <robot_wrapper/utils/xml_parser.h>

#include <robo_lib/kinematic_chain.h>

namespace rw_
{

#define Ur_Robot_fun_ std::string("[Ur_Robot::") + __func__ + "]: "

Ur_Robot::Ur_Robot(bool use_sim)
{
  ros::NodeHandle nh("~");
  std::string robot_desc;
  std::string base_link;
  std::string tool_link;
  std::string ftsensor_link;
  std::string robot_ip;
  std::string host_ip;
  int reverse_port;
  if (!nh.getParam("robot_description_name",robot_desc)) throw std::ios_base::failure(Ur_Robot_fun_ + "Failed to read parameter \"robot_description_name\".");
  if (!nh.getParam("base_link",base_link)) throw std::ios_base::failure(Ur_Robot_fun_ + "Failed to read parameter \"base_link\".");
  if (!nh.getParam("tool_link",tool_link)) throw std::ios_base::failure(Ur_Robot_fun_ + "Failed to read parameter \"tool_link\".");
  if (!nh.getParam("ftsensor_link",ftsensor_link)) ftsensor_link = tool_link;

  if (use_sim) robot_ip = "127.0.0.1";
  else if (!nh.getParam("robot_ip",robot_ip)) throw std::ios_base::failure(Ur_Robot_fun_ + "Failed to read parameter \"robot_ip\".");

  if (use_sim) host_ip = "127.0.0.1";
  else if (!nh.getParam("host_ip",host_ip)) throw std::ios_base::failure(Ur_Robot_fun_ + "Failed to read parameter \"host_ip\".");

  if (!nh.getParam("reverse_port",reverse_port))
  {
    reverse_port = 8080;
    PRINT_WARNING_MSG(std::string(Ur_Robot_fun_ + "Failed to read parameter \"reverse_port\".\n") + "Setting default: reverse_port = 8080\n");
  }

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

  // robot = malloc(sizeof(ur_::Robot));
  //
  // (ur_::Robot *)robot->

  robot.reset(new ur_::Robot(robot_desc, base_link, tool_link, host_ip, robot_ip, reverse_port));

  std::cerr << "=======> ur-robot created successfully!\n";

  robo_::KinematicChain base_sensor_chain = robo_::KinematicChain(robot_desc, base_link, ftsensor_link);
  arma::mat T_base_sensor = base_sensor_chain.getTaskPose( arma::vec().zeros( base_sensor_chain.getNumJoints() ) );


  robo_::KinematicChain base_tool_chain = robo_::KinematicChain(robot_desc, base_link, tool_link);
  arma::mat T_base_tool = base_tool_chain.getTaskPose( arma::vec().zeros( base_tool_chain.getNumJoints() ) );

  arma::mat T_sensor_tool = arma::inv(T_base_sensor) * T_base_tool;
  arma::mat R_sensor_tool = T_sensor_tool.submat(0,0,2,2);
  arma::vec p_sensor_tool = T_sensor_tool.submat(0,3,2,3);

  std::cerr << "================================================\n";
  std::cerr << "R_sensor_tool = \n" << R_sensor_tool << "\n";
  std::cerr << "p_sensor_tool = \n" << p_sensor_tool.t() << "\n";
  std::cerr << "================================================\n";


  initWrenchInterface(std::bind(&Ur_Robot::getTaskRotMat,this));
  initInvKinematicsInterface();

  // std::vector<std::string> joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  // std::string prefix = "ur5_";
  // for (int i=0; i<joint_names.size(); i++) joint_names[i] = prefix + joint_names[i];

  initPublishStateInterface(robot_desc, getJointNames(), std::bind(&Ur_Robot::getJointsPosition, this));

  std::string params_filename = ros::package::getPath("robot_wrapper") + "/config/params.yaml";
  rw_::XmlParser parser(params_filename);

  std::string load_fail_msg = Ur_Robot_fun_ + "Failed to load param ";

  // =======  load the relative rotation between the robot end-effector and the tool  =======
  arma::mat R_et;
  if (parser.getParam("ee_tool_rot", R_et)) this->setEeToolRot(R_et);

  // =======  check whether to use joint limit avoidance  =======
  use_jlav = false;
  if (parser.getParam("use_jlav", use_jlav) && use_jlav)
  {
    double jlav_gain, jlim_safety_margin;
    if (!parser.getParam("jlav_gain", jlav_gain)) throw std::runtime_error(load_fail_msg + "\"jlav_gain\"...");
    if (!parser.getParam("jlim_safety_margin", jlim_safety_margin)) throw std::runtime_error(load_fail_msg + "\"jlim_safety_margin\"...");
    this->setJLAV(jlav_gain, jlim_safety_margin);
  }

  N_JOINTS = robot->getNumJoints();

  Ts = robot->getCtrlCycle();

  mode.set(rw_::STOPPED);
  cmd_mode.set(rw_::IDLE);
  jpos_cmd.set(robot->getJointsPosition());

  run_ = true;
  std::thread robot_ctrl_thread = std::thread(&Ur_Robot::commandThread,this);
  int err_code = thr_::setThreadPriority(robot_ctrl_thread, SCHED_FIFO, 99);
  if (err_code) PRINT_WARNING_MSG(Ur_Robot_fun_ + "Failed to set thread priority! Reason:\n" + thr_::setThreadPriorErrMsg(err_code) + "\n", std::cerr);
  else PRINT_INFO_MSG(Ur_Robot_fun_ + "Set thread priority successfully!\n", std::cerr);
  robot_ctrl_thread.detach();

  mode_change.wait(); // wait for mode to be set
}

Ur_Robot::~Ur_Robot()
{
  run_ = false;
}

void Ur_Robot::setMode(const Mode &mode)
{
  if (mode == cmd_mode.get()) return;

  cmd_mode.set(mode);
  while (getMode()!=mode && isOk()) // to avoid spurious wake ups
  {
    mode_change.wait(); // wait to get notification from commandThread
  }
}

void Ur_Robot::commandThread()
{
  unsigned long long count = 0;

  arma::mat J;
  arma::vec q_dot;
  arma::vec adm_vel;

  arma::wall_clock timer;

  std::string limit_msg;

  while (run_)
  {
    if (!isOk())
    {
      // notify all to avoid deadlocks
      KRC_tick.notify();
      //mode_change.notify();
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
          robot->setNormalMode();
          jpos_cmd.set(robot->getJointsPosition());
          break;
        case rw_::Mode::JOINT_TORQUE_CONTROL:
          throw std::runtime_error(Ur_Robot_fun_ + "Unsupported mode \"JOINT_TORQUE_CONTROL\" for ur-robot...");
          break;
        case rw_::Mode::FREEDRIVE:
          robot->setFreedriveMode();
          break;
        case rw_::Mode::ADMITTANCE:
          robot->setNormalMode();
          adm_ctrl->init();
          break;
        case rw_::Mode::CART_VEL_CTRL:
          robot->setNormalMode();
          cart_vel_cmd.set(arma::vec().zeros(6));
          break;
        case rw_::Mode::IDLE:
          robot->setNormalMode();
          jpos_cmd.set(robot->getJointsPosition());
          break;
        case rw_::Mode::STOPPED:
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
      continue;
    }

    // send command according to current mode
    switch (mode.read())
    {
      case rw_::Mode::JOINT_POS_CONTROL:
        robot->setJointsPosition(jpos_cmd.get());
        break;
      case CART_VEL_CTRL:
        if (!safety_.assertToolVelLim(cart_vel_cmd.get(), &limit_msg))
        {
          //setErrMsg("Tool velocity limit exceeded: " + std::to_string(arma::norm(cart_vel_cmd.get())) );
          setErrMsg(limit_msg);
          setExternalStop(true);
        }
        else robot->setTaskVelocity(cart_vel_cmd.get());
        break;
      case rw_::Mode::FREEDRIVE:
        // robot->setJointTorque(-robot->getRobotJacobian().t() * tool_estimator->getToolWrench(this->getTaskOrientation()));
        // robot->setJointsTorque(arma::vec().zeros(7));
        break;
      case rw_::Mode::ADMITTANCE:
        adm_ctrl->update();
        adm_vel = adm_ctrl->getVelocity();
        if (!safety_.assertToolVelLim(adm_vel))
        {
          setErrMsg("Tool velocity limit exceeded!");
          setExternalStop(true);
        }
        else
        {
          if (use_svf)
          {
            q_dot = getInvJacobian()*adm_vel;
            if (this->use_jlav) q_dot += jlav->getControlSignal(getJointsPosition());
            if (!safety_.assertJointVelLim(q_dot))
            {
              setErrMsg("Joint Velocity limit exceeded!");
              setExternalStop(true);
            }
            else robot->setJointsVelocity(q_dot);
          }
          else robot->setTaskVelocity(adm_vel);
        }
        break;
      case rw_::Mode::IDLE:
        // std::cerr << "*** Send command in IDLE mode ***\n";
        // std::cerr << "Robot mode: " << robot->getModeName() << "\n";
        // robot->setJointsPosition(jpos_cmd.get());
        break;
      case rw_::Mode::STOPPED:
      case rw_::Mode::PROTECTIVE_STOP:
        // std::cerr << "***  MODE: STOPPED ***\n";
        break;
    }

    // sync with KRC
    if (robot->isOk()) robot->update();
    KRC_tick.notify();

    double elaps_time = timer.toc();
    // if (elaps_time > 2*getCtrlCycle())
    // {
    //   std::ostringstream oss;
    //   oss << elaps_time*1000;
    //   PRINT_WARNING_MSG(Ur_Robot_fun_ + "*** WARNING *** Elaps time = " + oss.str() + " ms\n");
    // }
  }

  // mode_change.notify(); // unblock in case wait was called from another thread
  KRC_tick.notify(); // unblock in case wait was called from another thread
}

bool Ur_Robot::setJointsTrajectory(const arma::vec &qT, double duration)
{
  // keep last known robot mode
  rw_::Mode prev_mode = this->getMode();
  // start controller
  this->setMode(rw_::JOINT_POS_CONTROL);
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

} // namespace rw_
