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
  if (robot_type.compare("lwr4p")==0) robot.reset(new LWR4p_Robot);
  else if (robot_type.compare("lwr4p_sim")==0) robot.reset(new LWR4p_Sim_Robot);
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
  q_start = robot->getJointsPosition();
  ctrl_running = false;

  readParams();

  while (gui->isRunning())
  {
    // =======> Check mode
    if (gui->getMode()==MainWindow::FREEDRIVE && robot->getMode()!=Robot::FREEDRIVE) setMode(Robot::FREEDRIVE);
    else if (gui->getMode()==MainWindow::IDLE && robot->getMode()!=Robot::IDLE) setMode(Robot::IDLE);
    else if (gui->getMode()==MainWindow::RUN_CONTROLLER && !ctrl_running)
    {
      setMode(Robot::FREEDRIVE);
      ctrl_running = true;
      std::thread(&GravComp::simulate_CartVelCtrl, this).detach();
    }

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

  std::string ctrl_mode;
  if (!nh.getParam("robot_run_ctrl_mode", ctrl_mode)) throw std::ios_base::failure("[GravComp::readParams]: Failed to read \"robot_run_ctrl_mode\" param.");
  if ( ctrl_mode.compare("CART_VEL_CTRL") == 0 )
  {
    robot_run_ctrl_mode = Robot::CART_VEL_CTRL;
    simulate = std::bind(&GravComp::simulate_CartVelCtrl, this);
  }

  if (!nh.getParam("robot_type", robot_type)) throw std::ios_base::failure("[GravComp::readParams]: Failed to read \"robot_type\" param.");

  if (!nh.getParam("train_data_filename", train_data_filename)) train_data_filename="";
  if (!nh.getParam("sim_data_filename", sim_data_filename)) throw std::ios_base::failure("[GravComp::readParams]: Failed to read \"sim_data_filename\" param.");

  std::vector<double> temp;

  if (!nh.getParam("Fext_dead_zone", temp)) Fext_dead_zone = arma::vec().zeros(6);
  else Fext_dead_zone = temp;

  if (!nh.getParam("a_fext_filt", a_fext_filt)) a_fext_filt = 0.0;
}

bool GravComp::loadCoMData(const std::string &path)
{
  std::ifstream in(path.c_str(), std::ios::in);
  if (!in)
  {
    // throw std::ios_base::failure("Failed to open file \"" + path + "\".");
    setErrMsg("Failed to open file \"" + path + "\".");
    return false;
  }

  try
  {
    arma::vec CoM_temp;
    io_::read_mat(CoM_temp, in, true);
    CoM(0) =  CoM_temp(0);
    CoM(1) =  CoM_temp(1);
    CoM(2) =  CoM_temp(2);
    is_CoM_calculated = true;
    in.close();
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
    if (Wrench_data.size() == 0) throw std::runtime_error("No data were recorded!");

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


void GravComp::simulate_CartVelCtrl()
{
  arma::vec wrench(6);
  Eigen::Map<Eigen::Vector6d> wrench_map(wrench.memptr());
  arma::vec quat(4);

  while (gui->getMode()==MainWindow::RUN_CONTROLLER)
  {

    robot->update();
    wrench = robot->getTaskWrench();
    quat = robot->getTaskOrientation();
    Eigen::Vector6d tool_wrench = tool_estimator.getGravityWrench(Eigen::Quaterniond(quat(0),quat(1),quat(2),quat(3)));
    wrench_map -= tool_wrench;
    
    std::cout << "wrench = " << wrench.t() << "\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  ctrl_running = false;
}

bool GravComp::saveCoMData(const std::string &save_path)
{
  if (!is_CoM_calculated)
  {
    setErrMsg("The execution data are empty!");
    return false;
  }

  // std::string path = save_path.empty() ? ros::package::getPath("GRAVITY_COMPENSATION") + "/data/" + sim_data_filename : save_path;
  std::ofstream out(save_path.c_str(), std::ios::out);
  if (!out)
  {
    setErrMsg("Failed to create file \"" + save_path + "\".");
    return false;
  } //throw std::ios_base::failure("[GravComp::saveCoMData]: Failed to create file \"" + save_path + "\".");

  try
  {
    arma::vec CoM_temp(3);
    CoM_temp(0) = CoM(0);
    CoM_temp(1) = CoM(1);
    CoM_temp(2) = CoM(2);
    io_::write_mat(CoM_temp, out, true);
    out.close();
    return true;
  }
  catch (std::exception &e)
  {
    setErrMsg(std::string("Error writing data:\n") + e.what());
    return false;
  }

}
