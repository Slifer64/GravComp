#include <grav_comp/grav_comp.h>

#include <QApplication>

#include <exception>
#include <fstream>
#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <ros/package.h>

#include <grav_comp/robot/lwr4p_robot.h>
#include <grav_comp/robot/lwr4p_sim_robot.h>

#include <io_lib/io_utils.h>

using namespace as64_;

GravComp::GravComp()
{
  is_CoM_calculated = false;

  ros::NodeHandle nh("~");
  if (!nh.getParam("robot_type", robot_type)) throw std::ios_base::failure("[GravComp::GravComp]: Failed to read \"robot_type\" param.");

  if (robot_type.compare("lwr4p")==0) robot.reset(new LWR4p_Robot(&tool_estimator));
  else if (robot_type.compare("lwr4p_sim")==0) robot.reset(new LWR4p_Sim_Robot(&tool_estimator));
  else throw std::runtime_error("Unsupported robot type \"" + robot_type + "\".");
}

GravComp::~GravComp()
{
  if (robot->isOk()) robot->stop();

  std::cerr << "[GravComp::~GravComp]: Waiting to be notified...\n";
  finish_sem.wait(); // wait for gui to finish
  std::cerr << "[GravComp::~GravComp]: Got notification!\n";
}

void GravComp::launch()
{
  int argc = 0;
  char **argv = 0;
  QApplication app(argc, argv);
  this->gui = new MainWindow(this->robot.get(), this);
  this->gui->show();
  this->start_sem.notify();
  app.exec();
  std::cerr << "[GravComp::launch]: Notifying!\n";
  this->finish_sem.notify();
  delete (this->gui); // must be destructed in this thread!
}

void GravComp::checkRobot()
{
  while (gui->isRunning())
  {
    if (!robot->isOk())
    {
      gui->robotNotOkSignal("An error occured on the robot!");
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

void GravComp::setMode(Robot::Mode mode)
{
  this->robot->setMode(mode);
}

ExecResultMsg GravComp::recPredefPoses()
{
  ExecResultMsg msg;

  // =====  read poses  =====
  poses.clear();
  ros::NodeHandle nh("~");
  std::vector<double> pose;
  int k = 0;
  while (true)
  {
    std::string pose_name = "pose" + (QString::number(++k)).toStdString();
    if (!nh.getParam(pose_name, pose)) break;
    poses.push_back(arma::vec(pose));
  }

  if (poses.size() == 0)
  {
    msg.setType(ExecResultMsg::WARNING);
    msg.setMsg("No predefined poses where specified...");
    return msg;
  }

  // =====  Move to each pose and record wrench-quat  =====
  Robot::Mode prev_mode = robot->getMode(); // store current robot mode
  robot->setMode(Robot::JOINT_POS_CONTROL);
  for (k=0; k<poses.size(); k++)
  {
    robot->update(); // waits for the next tick
    arma::vec q0 = robot->getJointsPosition();
    arma::vec qref = q0;
    arma::vec qT = poses[k];
    double duration = std::max(arma::max(arma::abs(qT-q0))*8.0/3.14159, 2.5);
    double t = 0.0;
    while (t < duration)
    {
      if (!gui->recPredefPoses()) goto rec_poses_interrupt;

      // if (!isOk())
      t += robot->getCtrlCycle();
      qref = Robot::get5thOrder(t, q0, qT, duration).col(0);
      robot->setJointsPosition(qref);
      robot->update(); // waits for the next tick
    }
    // pose reached ...
    recordCurrentWrenchQuat(); // record wrench quat
  }
  robot->setMode(prev_mode); // restore previous robot mode

  msg.setType(ExecResultMsg::INFO);
  msg.setMsg("Finished recording!");
  return msg;

  rec_poses_interrupt:
  msg.setType(ExecResultMsg::WARNING);
  msg.setMsg("Recording was interrupted!");
  return msg;
}

ExecResultMsg GravComp::recordCurrentWrenchQuat()
{
  ExecResultMsg msg;

  try
  {
    arma::vec wrench_temp = arma::vec().zeros(6);
    for (int i=0; i<20; i++)
    {
      wrench_temp += robot->getTaskWrench();
    }
    wrench_temp = wrench_temp/20; // to alleviate the noise
    Eigen::Map<Eigen::Vector6d> wrench(wrench_temp.memptr());
    Wrench_data.push_back(wrench);

    arma::vec quat = robot->getTaskOrientation();
    Quat_data.push_back(Eigen::Quaterniond(quat(0),quat(1),quat(2),quat(3)));

    std::cerr << "Wrench_data.size() = " << Wrench_data.size() << "\n";
    std::cerr << "Quat_data.size() = " << Quat_data.size() << "\n";

    char msg2[300];
    sprintf(msg2,"\nforce   : %10.3f %8.3f %8.3f\
    \ntorque: %10.3f %8.3f %8.3f\n\
    \nquaternion: %6.2f %7.2f %7.2f %7.2f\n",
    wrench(0),wrench(1),wrench(2),wrench(3),wrench(4),wrench(5),
    quat(0),quat(1),quat(2),quat(3));

    msg.setType(ExecResultMsg::INFO);
    msg.setMsg(std::string("Recorded current wrench and orientation!\n")+msg2);
    return msg;
  }
  catch(std::exception &e)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg(std::string("Error recording current wrench and orientation:\n") + e.what());
    return msg;
  }
}

ExecResultMsg GravComp::clearWrenchQuatData()
{
  ExecResultMsg msg;

  Wrench_data.clear();
  Quat_data.clear();

  msg.setType(ExecResultMsg::INFO);
  msg.setMsg("Cleared all recorded data!");
  return msg;
}

ExecResultMsg GravComp::calcCoM()
{
  ExecResultMsg msg;

  try{
    if (Wrench_data.size() < 3) throw std::runtime_error("At least 3 measurements are required!");

    tool_estimator.estimatePayload(Wrench_data, Quat_data);

    mass = tool_estimator.mass;
    CoM.resize(3);
    CoM(0) = tool_estimator.center_of_mass(0);
    CoM(1) = tool_estimator.center_of_mass(1);
    CoM(2) = tool_estimator.center_of_mass(2);

    std::ostringstream oss;
    oss << "mass: " << mass << "\n";
    oss << "CoM: " << CoM.t() << "\n";

    bool is_nan = false;
    for (int i=0; i<CoM.size(); i++)
    {
      if (isnan(CoM(i)))
      {
        is_nan = true;
        oss << "Warning: CoM(" << (QString::number(i)).toStdString() << ")=nan will be set to zero.\n";
        tool_estimator.center_of_mass(i) = CoM(i) = 0;
      }
    }

    is_CoM_calculated = true;

    if (is_nan) msg.setType(ExecResultMsg::WARNING);
    else msg.setType(ExecResultMsg::INFO);
    msg.setMsg("The CoM was calculated!\n" + oss.str());
    return msg;
  }
  catch(std::exception &e)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg(std::string("Error calculating CoM:\n") + e.what());
    return msg;
  }

}

ExecResultMsg GravComp::saveCoMData(const std::string &save_path)
{
  ExecResultMsg msg;

  if (!is_CoM_calculated)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("The tool dynamics have not been calculated!");
    return msg;
  }

  std::string suffix = "";
  int n = save_path.size();
  int i = n-1;
  for (; i>-1 && save_path[i]!='.'; i--);
  if (i<n-1) suffix = save_path.substr(i+1);

  int k;
  if (suffix.compare("bin")==0) k = 0;
  else if (suffix.compare("txt")==0) k = 1;
  else if (suffix.compare("yaml")==0 || suffix.compare("yml")==0) k = 2;
  else
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("Unknown file format \"" + suffix + "\"...\n");
    return msg;
  }

  std::ofstream out;
  if (k==0) out.open(save_path.c_str(), std::ios::out|std::ios::binary);
  else out.open(save_path.c_str(), std::ios::out);

  if (!out)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("Failed to create file \"" + save_path + "\".");
    return msg;
  }

  try
  {
    if (k==0)
    {
      io_::write_scalar(static_cast<double>(mass), out, true);
      io_::write_mat(CoM, out, true);
    }
    else if (k==1)
    {
      out << "mass: " << mass << "\n";
      out << "CoM: " << CoM(0) << " , " << CoM(1) << " , " << CoM(2) << "\n";
    }
    else if (k==2)
    {
      out << "mass: " << mass << "\n";
      out << "CoM: [ " << CoM(0) << " , " << CoM(1) << " , " << CoM(2) << " ]\n";
    }
    out.close();

    msg.setType(ExecResultMsg::INFO);
    msg.setMsg("The CoM data were successfully saved!");
    return msg;
  }
  catch (std::exception &e)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg(std::string("Error writing data:\n") + e.what());
    return msg;
  }

}

ExecResultMsg GravComp::loadCoMData(const std::string &path)
{
  ExecResultMsg msg;

  std::string suffix = "";
  int n = path.size();
  int i = n-1;
  for (; i>-1 && path[i]!='.'; i--);
  if (i<n-1) suffix = path.substr(i+1);

  int k;
  if (suffix.compare("bin")==0) k = 0;
  else if (suffix.compare("txt")==0) k = 1;
  else if (suffix.compare("yaml")==0 || suffix.compare("yml")==0) k = 2;
  else
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("Unknown file format \"" + suffix + "\"...\n");
    return msg;
  }

  std::ifstream in;
  if (k==0) in.open(path.c_str(), std::ios::in|std::ios::binary);
  else in.open(path.c_str(), std::ios::in);
  if (!in)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("Failed to open file \"" + path + "\".");
    return msg;
  }

  CoM.resize(3);

  try
  {
    if (k==0)
    {
      io_::read_scalar(mass, in, true);
      io_::read_mat(CoM, in, true);
    }
    else if (k==1)
    {
      std::string temp;
      in >> temp >> mass >> temp >> CoM(0) >> temp >> CoM(1) >> temp >> CoM(2);
    }
    else if (k==2)
    {
      std::string temp;
      in >> temp >> mass >> temp >> temp >> CoM(0) >> temp >> CoM(1) >> temp >> CoM(2);
    }
    is_CoM_calculated = true;
    in.close();

    std::ostringstream oss;
    oss << "mass: " << mass << "\n";
    oss << "CoM: " << CoM.t() << "\n";

    msg.setType(ExecResultMsg::INFO);
    msg.setMsg("The CoM data were successfully loaded!\n\n" + oss.str());
    return msg;
  }
  catch (std::exception &e)
  {
    in.close();
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("Error reading CoM data!\nMaybe the file is corrupted...");
    return msg;
  }
}
