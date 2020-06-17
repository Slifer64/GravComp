#include <grav_comp/grav_comp.h>

#include <QApplication>
#include <QThread>

#include <exception>
#include <fstream>
#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <ros/package.h>

#include <grav_comp/robot/lwr4p_robot.h>
#include <grav_comp/robot/lwr4p_sim_robot.h>

#include <io_lib/io_utils.h>
#include <io_lib/xml_parser.h>

using namespace as64_;

GravComp::GravComp()
{

  is_CoM_calculated = false;

  ros::NodeHandle nh("~");
  if (!nh.getParam("robot_type", robot_type)) throw std::ios_base::failure("[GravComp::GravComp]: Failed to read \"robot_type\" param.");

  bool use_sim;
  if (!nh.getParam("use_sim", use_sim)) throw std::ios_base::failure("[MainController::MainController]: Failed to read \"use_sim\" param.");
  if (robot_type.compare("lwr4p")==0)
  {
    if (use_sim) robot.reset(new LWR4p_Sim_Robot());
    else robot.reset(new LWR4p_Robot());
  }
  else throw std::runtime_error("Unsupported robot type \"" + robot_type + "\".");

  robot->setToolEstimator("");

  std::vector<double> ee_tool_rot;
  if (nh.getParam("ee_tool_rot", ee_tool_rot))
  {
    arma::mat R_et(3,3);
    for (int i=0;i<3; i++){ for (int j=0;j<3; j++) R_et(i,j) = ee_tool_rot[i*3+j]; }
    this->robot->setEeToolRot(R_et);
  }
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
  QThread::currentThread()->setPriority(QThread::LowestPriority);
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

  std::vector<arma::vec> poses = gui->getPredefPoses();

  if (poses.size() == 0)
  {
    msg.setType(ExecResultMsg::WARNING);
    msg.setMsg("No predefined poses where specified...");
    return msg;
  }

  // =====  Move to each pose and record wrench-quat  =====
  Robot::Mode prev_mode = robot->getMode(); // store current robot mode
  robot->setMode(Robot::JOINT_POS_CONTROL);
  for (int k=0; k<poses.size(); k++)
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
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait so that the robot is at rest and no movement intervene in the sensor's measurements
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

  if (Wrench_data.size() < 3) return ExecResultMsg(ExecResultMsg::ERROR, "At least 3 measurements are required!");

  try{

    robot->tool_estimator->estimatePayload(Wrench_data, Quat_data);

    mass = robot->tool_estimator->getMass();
    CoM.resize(3);
    Eigen:: Vector3d est_CoM = robot->tool_estimator->getCoM();
    CoM(0) = est_CoM(0);
    CoM(1) = est_CoM(1);
    CoM(2) = est_CoM(2);

    std::ostringstream oss;
    oss << "mass: " << mass << "\n";
    oss << "CoM: " << CoM.t() << "\n";

    bool is_nan = false;
    for (int i=0; i<CoM.size(); i++)
    {
      if (std::isnan(CoM(i)))
      {
        is_nan = true;
        oss << "Warning: CoM(" << (QString::number(i)).toStdString() << ")=nan will be set to zero.\n";
        est_CoM(i) = CoM(i) = 0;
      }
    }
    robot->tool_estimator->setCoM(est_CoM);

    is_CoM_calculated = true;

    if (is_nan) msg.setType(ExecResultMsg::WARNING);
    else msg.setType(ExecResultMsg::INFO);
    msg.setMsg("The CoM was calculated!\n" + oss.str());
    return msg;
  }
  catch(std::exception &e)
  {
    return ExecResultMsg(ExecResultMsg::ERROR, std::string("Error calculating CoM:\n") + e.what());
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

  std::string suffix;
  FileFormat file_fmt = getFileFormat(save_path, &suffix);

  if (file_fmt == FileFormat::UNKNOWN)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("Unknown file format \"" + suffix + "\"...\n");
    return msg;
  }

  std::ofstream out;
  if (file_fmt == FileFormat::BIN) out.open(save_path.c_str(), std::ios::out|std::ios::binary);
  else out.open(save_path.c_str(), std::ios::out);

  if (!out)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("Failed to create file \"" + save_path + "\".");
    return msg;
  }

  try
  {
    if (file_fmt == FileFormat::BIN)
    {
      io_::write_scalar(static_cast<double>(mass), out, true);
      io_::write_mat(CoM, out, true);
    }
    else if (file_fmt == FileFormat::TXT)
    {
      out << "mass: " << mass << "\n";
      out << "CoM: " << CoM(0) << " , " << CoM(1) << " , " << CoM(2) << "\n";
    }
    else if (file_fmt == FileFormat::YAML)
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

  std::string suffix;
  FileFormat file_fmt = getFileFormat(path, &suffix);

  if (file_fmt == FileFormat::UNKNOWN)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("Unknown file format \"" + suffix + "\"...\n");
    return msg;
  }

  std::ifstream in;
  if (file_fmt == FileFormat::BIN) in.open(path.c_str(), std::ios::in|std::ios::binary);
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
    if (file_fmt == FileFormat::BIN)
    {
      io_::read_scalar(mass, in, true);
      io_::read_mat(CoM, in, true);
    }
    else if (file_fmt == FileFormat::TXT)
    {
      std::string temp;
      in >> temp >> mass >> temp >> CoM(0) >> temp >> CoM(1) >> temp >> CoM(2);
    }
    else if (file_fmt == FileFormat::YAML)
    {
      std::string temp;
      in >> temp >> mass >> temp >> temp >> CoM(0) >> temp >> CoM(1) >> temp >> CoM(2);
    }
    is_CoM_calculated = true;
    in.close();

    std::ostringstream oss;
    oss << "mass: " << mass << "\n";
    oss << "CoM: " << CoM.t() << "\n";

    Eigen::Vector3d est_CoM;
    est_CoM << CoM(0), CoM(1), CoM(2);
    robot->tool_estimator->setCoM(est_CoM);
    robot->tool_estimator->setMass(mass);

    msg.setType(ExecResultMsg::INFO);
    msg.setMsg("The CoM data were successfully loaded!\n\n" + oss.str());
    return msg;
  }
  catch (std::exception &e)
  {
    in.close();
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("Error reading CoM data!\nThe file may be corrupted...");
    return msg;
  }
}

ExecResultMsg GravComp::saveWrenchOrientData(const std::string &path)
{
  ExecResultMsg msg;

  if (Wrench_data.size() == 0)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("No data are recorded!");
    return msg;
  }

  std::string suffix;
  FileFormat file_fmt = getFileFormat(path, &suffix);

  if (file_fmt == FileFormat::UNKNOWN)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("Unknown file format \"" + suffix + "\"...\n");
    return msg;
  }

  std::ofstream out;
  if (file_fmt == FileFormat::BIN) out.open(path.c_str(), std::ios::out|std::ios::binary);
  else out.open(path.c_str(), std::ios::out);

  if (!out)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("Failed to create file \"" + path + "\".");
    return msg;
  }

  int N_data = Wrench_data.size();
  int wrench_size = Wrench_data[0].size();
  int orient_size = 4;
  arma::mat wrench(wrench_size, N_data);
  arma::mat quat(orient_size,N_data);

  for (int j=0; j<N_data; j++)
  {
    for (int i=0; i<wrench_size; i++) wrench(i,j) = Wrench_data[j](i);
    quat.col(j) = arma::vec({Quat_data[j].w(), Quat_data[j].x(), Quat_data[j].y(), Quat_data[j].z()});
  }

  try
  {
    if (file_fmt == FileFormat::BIN)
    {
      io_::write_mat(wrench, out, true);
      io_::write_mat(quat, out, true);
    }
    else if (file_fmt == FileFormat::TXT)
    {
      std::ostringstream oss;
      oss << "N: " << N_data << "\n";
      oss << "Wrench:\n";
      oss << wrench << "\n";
      oss << "Quaternion:\n";
      oss << quat << "\n";
      out << oss.str();
    }
    else if (file_fmt == FileFormat::YAML)
    {
      std::ostringstream oss;
      oss << "N: " << N_data << "\n";
      oss << "wrench: [";
      for (int i=0; i<wrench.n_rows; i++)
      {
        for (int j=0; j<wrench.n_cols-1; j++) oss << wrench(i,j) << ", ";
        oss << wrench(i, wrench.n_cols-1);
        if (i<wrench.n_rows-1) oss << "; ";
      }
      oss << "]\n";
      oss << "quaternion: [";
      for (int i=0; i<quat.n_rows; i++)
      {
        for (int j=0; j<quat.n_cols-1; j++) oss << quat(i,j) << ", ";
        oss << quat(i, quat.n_cols-1);
        if (i<quat.n_rows-1) oss << "; ";
      }
      oss << "]\n";
      out << oss.str();
    }
    out.close();

    msg.setType(ExecResultMsg::INFO);
    msg.setMsg("The wrench-orient data were successfully saved!");
    return msg;
  }
  catch (std::exception &e)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg(std::string("Error writing data:\n") + e.what());
    return msg;
  }
}

ExecResultMsg GravComp::loadWrenchOrientData(const std::string &path)
{
  ExecResultMsg msg;

  std::string suffix;
  FileFormat file_fmt = getFileFormat(path, &suffix);

  if (file_fmt == FileFormat::UNKNOWN)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("Unknown file format \"" + suffix + "\"...\n");
    return msg;
  }

  std::ifstream in;
  if (file_fmt==FileFormat::BIN) in.open(path.c_str(), std::ios::in|std::ios::binary);
  else in.open(path.c_str(), std::ios::in);
  if (!in)
  {
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("Failed to open file \"" + path + "\".");
    return msg;
  }

  arma::mat wrench;
  arma::mat quat;

  int wrench_size = 6;
  int orient_size = 4;

  try
  {
    if (file_fmt==FileFormat::BIN)
    {
      io_::read_mat(wrench, in, true);
      io_::read_mat(quat, in, true);
    }
    else if (file_fmt==FileFormat::TXT)
    {
      std::stringstream iss;
      iss << in.rdbuf();
      std::string label;
      int N_data;
      iss >> label >>  N_data;

      wrench.resize(wrench_size, N_data);
      iss >> label; // Wrench label
      for (int i=0; i<wrench_size; i++)
      {
        for (int j = 0; j < N_data; j++)
        {
          double f;
          iss >> f;
          wrench(i,j) = f;
        }
      }
      quat.resize(orient_size, N_data);
      iss >> label; // Quat label
      for (int i=0; i<orient_size; i++)
      {
        for (int j = 0; j < N_data; j++)
        {
          double f;
          iss >> f;
          quat(i,j) = f;
        }
      }
    }
    else if (file_fmt==FileFormat::YAML)
    {
      io_::XmlParser parser(path);

      int N_data;
      if (!parser.getParam("N",N_data))
      {
        msg.setType(ExecResultMsg::ERROR);
        msg.setMsg("Failed to read parameter \"N\".");
        return msg;
      }

      if (!parser.getParam("wrench",wrench))
      {
        msg.setType(ExecResultMsg::ERROR);
        msg.setMsg("Failed to read parameter \"wrench\".");
        return msg;
      }

      if (!parser.getParam("quaternion",quat))
      {
        msg.setType(ExecResultMsg::ERROR);
        msg.setMsg("Failed to read parameter \"quaternion\".");
        return msg;
      }
    }
    in.close();

    int N_data = wrench.n_cols;
    Wrench_data.resize(N_data);
    Quat_data.resize(N_data);
    for (int j=0; j<N_data; j++)
    {
      for (int i=0; i<6; i++) Wrench_data[j](i) = wrench(i,j);
      Quat_data[j] = Eigen::Quaterniond(quat(0,j), quat(1,j), quat(2,j), quat(3,j) );
    }

    std::ostringstream oss;
    oss << "Number of measurements = " << N_data << "\n";

    msg.setType(ExecResultMsg::INFO);
    msg.setMsg("The wrench-orient data were successfully loaded!\n\n" + oss.str());
    return msg;
  }
  catch (std::exception &e)
  {
    in.close();
    msg.setType(ExecResultMsg::ERROR);
    msg.setMsg("Error reading data!\nThe file may be corrupted...");
    return msg;
  }
}
