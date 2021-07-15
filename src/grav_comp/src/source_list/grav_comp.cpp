#include <grav_comp/grav_comp.h>

#include <QApplication>
#include <QThread>

#include <exception>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <csignal>

#include <ros/ros.h>
#include <ros/package.h>

#include <robot_wrapper/ur_robot.h>
#include <robot_wrapper/lwr4p_robot.h>

#include <io_lib/io_utils.h>
#include <io_lib/xml_parser.h>

using namespace as64_;

#define GravComp_fun_ std::string("[GravComp::") + __func__ + "]: "

GravComp::GravComp()
{
  is_CoM_calculated = false;

  ros::NodeHandle nh("~");

  if (!nh.getParam("robot_type", robot_type)) throw std::ios_base::failure(GravComp_fun_ + "Failed to read \"robot_type\" param.");
  bool use_sim;
  if (!nh.getParam("use_sim", use_sim)) throw std::ios_base::failure(GravComp_fun_ + "Failed to read \"use_sim\" param.");

  if (robot_type.compare("lwr4p")==0) robot.reset(new rw_::LWR4p_Robot(use_sim));
  else if (robot_type.compare("ur")==0) robot.reset(new rw_::Ur_Robot(use_sim));
  else throw std::runtime_error("Unsupported robot type \"" + robot_type + "\".");

  // check whether to publish joint states
  // bool pub_jstates_flag = false;
  // if (nh.getParam("pub_jstates_flag", pub_jstates_flag) && pub_jstates_flag)
  // {
  //   std::string pub_jstates_topic;
  //   if (!nh.getParam("publish_jstates_topic", pub_jstates_topic)) throw std::runtime_error(GravComp_fun_ + "Failed to load param \"pub_jstates_topic\"...");
  //   robot->publishJointStates(pub_jstates_topic);
  // }

  // =======  check whether to use ati-sensor  =======
  // bool use_ati_sensor = false;
  // if (nh.getParam("use_ati", use_ati_sensor) && use_ati_sensor)
  // {
  //   robot->useAtiSensor();
  //   bool set_wrench_bias = false;
  //   if (!nh.getParam("set_wrench_bias", set_wrench_bias) && set_wrench_bias) robot->setWrenchBias();
  // }

  // =======  Tool compensation  =======
  std::string tool_massCoM_file;
  std::string tool_param_name = "tool_massCoM_file";
  if (use_sim) tool_param_name = "dummy_tool_massCoM_file";
  if (nh.getParam(tool_param_name.c_str(), tool_massCoM_file))
  {
    tool_massCoM_file = ros::package::getPath("grav_comp") + "/" + tool_massCoM_file;
    robot->setToolEstimator(tool_massCoM_file);
  }

  // =======  Robot ee tf publisher  =======
  std::string base_link;
  if (!nh.getParam("base_link", base_link)) throw std::runtime_error(GravComp_fun_ + "Failed to load param \"base_link\"...");
  ee_tf_pub.reset( new rviz_::TfPosePublisher([this](){ return arma::join_vert(robot->getTaskPosition(), robot->getTaskOrientation());} , base_link,"robot-ee") );

  std::string tool_link;
  if (!nh.getParam("tool_link", tool_link)) throw std::runtime_error(GravComp_fun_ + "Failed to load param \"tool_link\"...");
  com_tf_pub.reset( new rviz_::TfPosePublisher([this](){ return arma::join_vert(tool_estimator.getCoM(true), arma::vec({1,0,0,0}));}, tool_link, "CoM") );

  // =======  register signal SIGINT and signal handler  =======
  signal(SIGINT, GravComp::closeGUI);

  // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

GravComp::~GravComp()
{
  if (robot->isOk()) robot->stop();

  std::cerr << GravComp_fun_ + "Waiting to be notified...\n";
  while (!gui_finished); //finish_sem.wait(); // wait for gui to finish
  std::cerr << GravComp_fun_ + "Got notification!\n";
}

MainWindow *GravComp::gui_ = 0;

void GravComp::closeGUI(int)
{
  emit GravComp::gui_->closeSignal();
}

QMainWindow *GravComp::createMainWindow()
{
  gui = new MainWindow(robot.get(), this);
  GravComp::gui_ = gui;
  return gui;
}

void GravComp::setMode(rw_::Mode mode)
{
  this->robot->setMode(mode);
}

ExecResultMsg GravComp::recPredefPoses()
{
  std::vector<arma::vec> poses = gui->getPredefPoses();

  if (poses.size() == 0) return ExecResultMsg(ExecResultMsg::ERROR, "No predefined poses where specified...");

  // =====  Move to each pose and record wrench-quat  =====
  rw_::Mode prev_mode = robot->getMode(); // store current robot mode
  robot->setMode(rw_::JOINT_POS_CONTROL);
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
      qref = rw_::get5thOrder(t, q0, qT, duration).col(0);
      robot->setJointsPosition(qref);
      robot->update(); // waits for the next tick
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait so that the robot is at rest and no movement intervene in the sensor's measurements
    // pose reached ...
    recordCurrentWrenchQuat(); // record wrench quat
  }
  robot->setMode(prev_mode); // restore previous robot mode

  return ExecResultMsg(ExecResultMsg::INFO, "Finished recording!");

  rec_poses_interrupt:
  return ExecResultMsg(ExecResultMsg::WARNING, "Recording was interrupted!");
}

ExecResultMsg GravComp::recordCurrentWrenchQuat()
{
  try
  {
    arma::vec wrench_temp = arma::vec().zeros(6);
    for (int i=0; i<20; i++)
    {
      wrench_temp += robot->getTaskWrench();
    }
    wrench_temp = wrench_temp/20; // to alleviate the noise
    arma::mat R = robot->getTaskRotMat();
    wrench_temp.subvec(0,2) = R.t()*wrench_temp.subvec(0,2);
    wrench_temp.subvec(3,5) = R.t()*wrench_temp.subvec(3,5);
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

    return ExecResultMsg(ExecResultMsg::INFO, std::string("Recorded current wrench and orientation!\n")+msg2);
  }
  catch(std::exception &e)
  { return ExecResultMsg(ExecResultMsg::ERROR, std::string("Error recording current wrench and orientation:\n") + e.what()); }
}

ExecResultMsg GravComp::clearWrenchQuatData()
{
  Wrench_data.clear();
  Quat_data.clear();
  return ExecResultMsg(ExecResultMsg::INFO, "Cleared all recorded data!");
}

ExecResultMsg GravComp::calcCoM()
{
  try{
    if (Wrench_data.size() < 3) throw std::runtime_error("At least 3 measurements are required!");

    tool_estimator.estimatePayload(Wrench_data, Quat_data);

    mass = tool_estimator.getMass();
    CoM = tool_estimator.getCoM(true);

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
        CoM(i) = 0;
      }
    }
    tool_estimator.setCoM(CoM);

    robot->setToolEstimator(tool_estimator);

    is_CoM_calculated = true;

    ExecResultMsg msg;
    if (is_nan) msg.setType(ExecResultMsg::WARNING);
    else msg.setType(ExecResultMsg::INFO);
    msg.setMsg("The CoM was calculated!\n" + oss.str());
    return msg;
  }
  catch(std::exception &e)
  { return ExecResultMsg(ExecResultMsg::ERROR, std::string("Error calculating CoM:\n") + e.what()); }
}

ExecResultMsg GravComp::saveCoMData(const std::string &save_path)
{
  if (!is_CoM_calculated) return ExecResultMsg(ExecResultMsg::ERROR, "The tool dynamics have not been calculated!");

  std::ofstream out(save_path.c_str(), std::ios::out);
  if (!out) return ExecResultMsg(ExecResultMsg::ERROR, "Failed to create file \"" + save_path + "\".");

  try
  {
    out << "mass: " << mass << "\n";
    out << "CoM: [ " << CoM(0) << "; " << CoM(1) << "; " << CoM(2) << " ]\n";
    out.close();
    return ExecResultMsg(ExecResultMsg::INFO, "The CoM data were successfully saved!");
  }
  catch (std::exception &e)
  { return ExecResultMsg(ExecResultMsg::ERROR, std::string("Error writing data:\n") + e.what()); }

}

ExecResultMsg GravComp::loadCoMData(const std::string &path)
{
  CoM.resize(3);

  try
  {
    io_::XmlParser parser(path);
    if (!parser.getParam("mass", mass)) throw std::runtime_error("Failed to read param \"mass\"...");
    if (!parser.getParam("CoM", CoM)) throw std::runtime_error("Failed to read param \"CoM\"...");

    is_CoM_calculated = true;

    Eigen::Vector3d est_CoM;
    est_CoM << CoM(0), CoM(1), CoM(2);
    tool_estimator.setCoM(est_CoM);
    tool_estimator.setMass(mass);

    robot->setToolEstimator(tool_estimator);

    std::ostringstream oss;
    oss << "mass: " << mass << "\n";
    oss << "CoM: " << CoM.t() << "\n";

    return ExecResultMsg(ExecResultMsg::INFO, "The CoM data were successfully loaded!\n\n" + oss.str());
  }
  catch (std::exception &e)
  { return ExecResultMsg(ExecResultMsg::ERROR, std::string("Error reading CoM data:") + e.what()); }
}

ExecResultMsg GravComp::saveWrenchOrientData(const std::string &path)
{
  if (Wrench_data.size() == 0) return ExecResultMsg(ExecResultMsg::ERROR, "No data are recorded!");

  std::ofstream out(path.c_str(), std::ios::out|std::ios::binary);
  if (!out) return ExecResultMsg(ExecResultMsg::ERROR, "Failed to create file \"" + path + "\".");

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
    io_::write_mat(wrench, out, true);
    io_::write_mat(quat, out, true);

    out.close();

    return ExecResultMsg(ExecResultMsg::INFO, "The wrench-orient data were successfully saved!");
  }
  catch (std::exception &e)
  {
    return ExecResultMsg(ExecResultMsg::ERROR, std::string("Error writing data:\n") + e.what());
  }
}

ExecResultMsg GravComp::loadWrenchOrientData(const std::string &path)
{
  std::ifstream in(path.c_str(), std::ios::in|std::ios::binary);
  if (!in) return ExecResultMsg(ExecResultMsg::ERROR, "Failed to open file \"" + path + "\".");

  arma::mat wrench;
  arma::mat quat;

  try
  {
    io_::read_mat(wrench, in, true);
    io_::read_mat(quat, in, true);
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
    return ExecResultMsg(ExecResultMsg::INFO, "The wrench-orient data were successfully loaded!\n\n" + oss.str());
  }
  catch (std::exception &e)
  {
    in.close();
    return ExecResultMsg(ExecResultMsg::ERROR, "Error reading data!\nThe file may be corrupted...");
  }
}
