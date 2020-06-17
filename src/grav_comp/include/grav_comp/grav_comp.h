#ifndef GRAVITY_COMPENSATION_H
#define GRAVITY_COMPENSATION_H

#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <armadillo>
#include <Eigen/Dense>

#include <grav_comp/robot/robot.h>
#include <grav_comp/gui/mainwindow/mainwindow.h>

class GravComp
{

public:
  GravComp();
  ~GravComp();

  void launch();
protected:

  friend MainWindow;

  bool rec_predef_poses_run;
  ExecResultMsg recPredefPoses();
  ExecResultMsg recordCurrentWrenchQuat();
  ExecResultMsg clearWrenchQuatData();

  ExecResultMsg calcCoM();
  ExecResultMsg loadCoMData(const std::string &path);
  ExecResultMsg saveCoMData(const std::string &save_path);
  ExecResultMsg loadWrenchOrientData(const std::string &path);
  ExecResultMsg saveWrenchOrientData(const std::string &path);

  void checkRobot();
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
  bool is_CoM_calculated;

  // robot
  std::string robot_type;
  std::shared_ptr<Robot> robot;

  // GUI
  MainWindow *gui;

  Semaphore start_sem;
  Semaphore finish_sem;

};

#endif // GRAVITY_COMPENSATION_H
