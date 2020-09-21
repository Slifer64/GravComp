#include <ur_robot/robot.h>
#include <iostream>
#include <string>
#include <ros/package.h>

namespace as64_
{

namespace ur_
{

Robot::Robot(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link,
             const std::string &robot_ip, int reverse_port):
    RobotArm(urdf_model, base_link, tool_link)
{
  initRobot(robot_ip, reverse_port);
}

Robot::Robot(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link,
             const std::string &robot_ip, int reverse_port):
    RobotArm(robot_desc_param, base_link, tool_link)
{
  initRobot(robot_ip, reverse_port);
}

void Robot::initRobot(const std::string &robot_ip, int reverse_port)
{
  ros::NodeHandle nh("~");
  if (!nh.getParam("servo_a",servo_a)) servo_a = 10;
  if (!nh.getParam("servo_v",servo_v)) servo_v = 20;
  if (!nh.getParam("servo_T",servo_T)) servo_T = 0.01;
  if (!nh.getParam("servo_lookahead_time",servo_lookahead_time)) servo_lookahead_time = 0.02;
  if (!nh.getParam("servo_gain",servo_gain)) servo_gain = 800;

  this->robot_ip = robot_ip;
  this->reverse_port = reverse_port;

  runUrDriver();

  mode = ur_::Mode::NORMAL;

  std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // needed to let UR initialize
  // ros::Duration(2.0).sleep(); // needed to let UR initialize

  ctrl_cycle = ur_driver->getServojTime();

  update();

  last_joint_pos = getJointsPosition();
}

Robot::~Robot()
{
  shutdown_sem.notify();
  if (ur_driver_thr.joinable()) ur_driver_thr.join();
  delete ur_driver;
}

void Robot::enable()
{
  mode = ur_::Mode::NORMAL;
}

void Robot::update()
{
  ur_driver->update_sem.wait();
}

void Robot::setFreedriveMode()
{
  if (this->getMode() != ur_::Mode::PROTECTIVE_STOP) command_mode("freedrive_mode()\n");
}

void Robot::setNormalMode()
{
  if (this->getMode() != ur_::Mode::PROTECTIVE_STOP) command_mode("end_freedrive_mode()\n");
}

void Robot::protectiveStop()
{
  if (getMode() == ur_::PROTECTIVE_STOP) return;
  last_joint_pos = getJointsPosition();
  mode = ur_::Mode::PROTECTIVE_STOP;
  print_warn_msg("[Robot::protectiveStop]: Mode changed to \"" + getModeName(getMode()) + "\"\n");
}

void Robot::command_mode(const std::string &mode) const
{
  std::string cmd;
  cmd = "def command_mode():\n\n\t" + mode + "\n\twhile (True):\n\t\tsync()\n\tend\nend\n";
  ur_driver->setUrScriptCmd(cmd);
}

void Robot::runUrDriver()
{
  Semaphore start_ur_driver_sem;

  ur_driver_thr = std::thread( [this, &start_ur_driver_sem]()
  {
    this->ur_driver = new UrDriver(this->robot_ip, this->reverse_port, 0.002);
    if (!this->ur_driver->start()) throw std::runtime_error("[ur_::Robot::runUrDriver]: Failed to start the UrDriver...\n");
    start_ur_driver_sem.notify();
    this->shutdown_sem.wait();
  });
  start_ur_driver_sem.wait();
}

bool Robot::isOk() const
{
  if (ur_driver->isEmergencyStopped())
  {
    *(const_cast<std::string *>(&err_msg)) = "EMERGENCY STOP!\n";
    return false;
  }

  if (ur_driver->isProtectiveStopped())
  {
    *(const_cast<std::string *>(&err_msg)) = "PROTECTIVE STOP!\n";
    return false;
  }

  *(const_cast<std::string *>(&err_msg)) = "";

  return true;
}

void Robot::servoj(const arma::vec &q, double a, double v, double t, double lookahead_time, double gain)
{
  std::ostringstream out;
  out << "servoj(" << print_vector(q) << "," << a << "," << v << "," << t << "," << lookahead_time << gain << ")\n";
  ur_driver->setUrScriptCmd(out.str());
}

void Robot::movej(const arma::vec &q, double a, double v, double t, double r)
{
  std::ostringstream out;
  out << "movej(" << print_vector(q) << "," << a << "," << v << "," << t << "," << r << ")\n";
  ur_driver->setUrScriptCmd(out.str());
}

void Robot::movel(const arma::vec &p, double a, double v, double t, double r)
{
  std::ostringstream out;
  out << "movel(p" << print_vector(p) << "," << a << "," << v << "," << t << "," << r << ")\n";
  ur_driver->setUrScriptCmd(out.str());
}

void Robot::speedj(arma::vec dq, double a, double t)
{
  std::ostringstream out;
  out << "speedj(" << print_vector(dq) << "," << a;
  if (t > 0.0) out << "," << t;
  out << ")\n";
  ur_driver->setUrScriptCmd(out.str());
}

void Robot::speedl(arma::vec dp, double a, double t)
{
  std::ostringstream out;
  out << "speedl(" << print_vector(dp) << "," << a;
  if (t > 0.0) out << "," << t;
  out << ")\n";
  ur_driver->setUrScriptCmd(out.str());
}

void Robot::stopj(double a)
{
  std::ostringstream out;
  out << "stopj(" << a << ")\n";
  ur_driver->setUrScriptCmd(out.str());
}

void Robot::stopl(double a)
{
  std::ostringstream out;
  out << "stopl(" << a << ")\n";
  ur_driver->setUrScriptCmd(out.str());
}

void Robot::sleep(double t)
{
  std::ostringstream out;
  out << "sleep(" << t << ")\n";
  ur_driver->setUrScriptCmd(out.str());
}

void Robot::powerdown()
{
  ur_driver->setUrScriptCmd("powerdown()\n");
}

}  // namespace ur_

} // namespace as64_
