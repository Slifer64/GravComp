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

  ros::NodeHandle("~").getParam("robot_type",robot_type);
  if (robot_type.compare("lwr4p")==0) robot.reset(new LWR4p_Robot(&tool_estimator));
  else if (robot_type.compare("lwr4p_sim")==0) robot.reset(new LWR4p_Sim_Robot(&tool_estimator));
  else throw std::runtime_error("Unsupported robot type \"" + robot_type + "\".");

  launchGUI();
}

void GravComp::launchGUI()
{
  std::thread([this]()
              {
                int argc = 0;
                char **argv = 0;
                QApplication app(argc, argv);
                this->gui = new MainWindow(this->robot.get());
                this->gui->show();
                this->start_sem.notify();
                app.exec();
                std::cerr << "[GravComp::launchGUI]: Notifying!\n";
                this->finish_sem.notify();
                delete (this->gui); // must be destructed in this thread!
              }).detach();

  start_sem.wait(); // wait for gui to be initialized
}

void GravComp::run()
{
  ctrl_running = false;

  readParams();

  while (gui->isRunning())
  {
    // =======> Check mode
    if (gui->getMode()==MainWindow::FREEDRIVE && robot->getMode()!=Robot::FREEDRIVE) setMode(Robot::FREEDRIVE);
    else if (gui->getMode()==MainWindow::IDLE && robot->getMode()!=Robot::IDLE) setMode(Robot::IDLE);

    // =======> Check if robot is ok
    if (!robot->isOk())
    {
      gui->terminateAppSignal("An error occured on the robot.\nThe program will terminate...");
      break;
    }

    if (gui->clearWrenchQuatData())
    {
      if (clearWrenchQuatData()) gui->sendclearWrenchQuatDataAck(true, "Reached start pose!");
      else gui->sendclearWrenchQuatDataAck(false, "Failed to reach start pose...");
    }

    if (gui->loadData())
    {
      if (! loadCoMData(gui->getLoadDataPath()) ) gui->sendLoadAck(false, getErrMsg().c_str());
      else gui->sendLoadAck(true, "The CoM data were successfully loaded!");
    }

    if (gui->saveData())
    {
      if (! saveCoMData(gui->getSaveDataPath()) ) gui->sendSaveAck(false, getErrMsg().c_str());
      else gui->sendSaveAck(true, "The CoM data were successfully saved!");
    }

    if (gui->calcCoM())
    {
      if (! calcCoM() ) gui->sendcalcCoMAck(false, getErrMsg().c_str());
      else gui->sendcalcCoMAck(true, QString("The CoM was calculated!\n") + getInfoMsg().c_str());
    }

    if (gui->recCurrentWrenchQuat())
    {
      if (! recordCurrentWrenchQuat() ) gui->sendRecCurrentWrenchQuatAck(false, getErrMsg().c_str());
      else gui->sendRecCurrentWrenchQuatAck(true, QString("Recorded current wrench and orientation!\n") + getInfoMsg().c_str());
    }

    // robot->update();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (robot->isOk()) robot->stop();

  std::cerr << "[GravComp::run]: Waiting to be notified...\n";
  finish_sem.wait(); // wait for gui to finish
  std::cerr << "[GravComp::launchGUI]: Got notification!\n";

}

void GravComp::setMode(Robot::Mode mode)
{
  ctrl_running = false;
  robot->setMode(mode);
  if (robot->isOk()) gui->modeChangedSignal(); // inform the gui that the robot's mode changed
}

bool GravComp::recordCurrentWrenchQuat()
{
  try
  {
    arma::vec wrench_temp = robot->getTaskWrench();
    Eigen::Map<Eigen::Vector6d> wrench(wrench_temp.memptr());

    arma::vec quat_temp = robot->getTaskOrientation();
    Eigen::Map<Eigen::Vector3d> quat(quat_temp.memptr());

    Wrench_data.push_back(wrench);
    Quat_data.push_back(Eigen::Quaterniond(quat(0),quat(1),quat(2),quat(3)));

    std::cerr << "Wrench_data.size() = " << Wrench_data.size() << "\n";
    std::cerr << "Quat_data.size() = " << Quat_data.size() << "\n";

    std::ostringstream oss;
    oss << "wrench = " << wrench_temp.t() << "\n";
    setInfoMsg(oss.str());
  }
  catch(std::exception &e)
  {
    setErrMsg(std::string("Error recording current wrench and orientation:\n") + e.what());
    return false;
  }

  return true;
}

bool GravComp::clearWrenchQuatData()
{
  Wrench_data.clear();
  Quat_data.clear();

  return true;
}

void GravComp::readParams()
{
  ros::NodeHandle nh("~");

  if (!nh.getParam("robot_type", robot_type)) throw std::ios_base::failure("[GravComp::readParams]: Failed to read \"robot_type\" param.");

  if (!nh.getParam("train_data_filename", train_data_filename)) train_data_filename="";
  if (!nh.getParam("sim_data_filename", sim_data_filename)) throw std::ios_base::failure("[GravComp::readParams]: Failed to read \"sim_data_filename\" param.");
}

bool GravComp::loadCoMData(const std::string &path)
{
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
    setErrMsg("Unknown file format \"" + suffix + "\"...\n");
    return false;
  }

  std::ifstream in;
  if (k==0) in.open(path.c_str(), std::ios::in|std::ios::binary);
  else in.open(path.c_str(), std::ios::in);
  if (!in)
  {
    setErrMsg("Failed to open file \"" + path + "\".");
    return false;
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

    // std::cerr << "mass = " << mass << "\n";
    // std::cerr << "CoM = " << CoM.t() << "\n";
    return true;
  }
  catch (std::exception &e)
  {
    setErrMsg("Error reading CoM data!\nMaybe the file is corrupted...");
    in.close();
    return false;
  }
}

bool GravComp::calcCoM()
{
  try{
    if (Wrench_data.size() < 3) throw std::runtime_error("At least 3 measurements are required!");

    tool_estimator.estimatePayload(Wrench_data, Quat_data);

    // CoM.resize(3);
    // CoM(0) = 0;
    // CoM(1) = 0;
    // CoM(2) = 0;
    // mass = 0;

    mass = tool_estimator.mass;
    CoM.resize(3);
    CoM(0) = tool_estimator.center_of_mass(0);
    CoM(1) = tool_estimator.center_of_mass(1);
    CoM(2) = tool_estimator.center_of_mass(2);

    std::ostringstream oss;
    oss << "mass: " << mass << "\n";
    oss << "CoM: " << CoM.t() << "\n";
    setInfoMsg(oss.str().c_str());

    is_CoM_calculated = true;
  }
  catch(std::exception &e)
  {
    setErrMsg(std::string("Error calculating CoM:\n") + e.what());
    return false;
  }

  return true;
}

bool GravComp::saveCoMData(const std::string &save_path)
{
  if (!is_CoM_calculated)
  {
    setErrMsg("The tool dynamics have not been calculated!");
    return false;
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
    setErrMsg("Unknown file format \"" + suffix + "\"...\n");
    return false;
  }

  std::ofstream out;
  if (k==0) out.open(save_path.c_str(), std::ios::out|std::ios::binary);
  else out.open(save_path.c_str(), std::ios::out);

  if (!out)
  {
    setErrMsg("Failed to create file \"" + save_path + "\".");
    return false;
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
    return true;
  }
  catch (std::exception &e)
  {
    setErrMsg(std::string("Error writing data:\n") + e.what());
    return false;
  }

}
