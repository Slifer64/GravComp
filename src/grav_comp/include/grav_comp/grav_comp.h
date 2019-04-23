#ifndef GRAVITY_COMPENSATION_H
#define GRAVITY_COMPENSATION_H

#include <string>
#include <vector>
#include <memory>
#include <armadillo>
#include <Eigen/Dense>

#include <grav_comp/robot/robot.h>

#include <grav_comp/gui/mainwindow.h>

#include <grav_comp/tool_estimator.h>

class GravComp
{

public:
  GravComp();

  void run();

private:
  void launchGUI();


  void readParams();
  bool loadCoMData(const std::string &path);
  bool calcCoM();

  bool recordCurrentWrenchQuat();

  bool saveCoMData(const std::string &save_path="");
  bool clearWrenchQuatData();

  void setMode(Robot::Mode mode);

  std::string err_msg;
  void setErrMsg(const std::string &msg) { err_msg = msg; }
  std::string getErrMsg() const { return err_msg; }

  std::string info_msg;
  void setInfoMsg(const std::string &msg) { info_msg = msg; }
  std::string getInfoMsg() const { return info_msg; }

  std::vector<Eigen::Vector6d> Wrench_data;
  std::vector<Eigen::Quaterniond> Quat_data;
  arma::vec CoM;
  double mass;

  ToolEstimator tool_estimator;

  bool is_CoM_calculated;

  bool ctrl_running;

  // training data
  std::string train_data_filename; // name of the file containing the training data

  // simulation data
  std::string sim_data_filename; // name of the file where the simulation data will be stored

  arma::wall_clock timer;

  // robot
  std::string robot_type;
  std::shared_ptr<Robot> robot;

  // GUI
  MainWindow *gui;

  Semaphore start_sem;
  Semaphore finish_sem;

};

#endif // GRAVITY_COMPENSATION_H
